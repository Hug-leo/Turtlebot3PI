"""
Warehouse Pick Handler — listens for QR scans, reports scan/pick signals
to the dashboard server. Navigation is driven by the dashboard via rosbridge.

Flow:
  1. Dashboard sends nav goals → robot moves to shelf (handled by Nav2 + rosbridge)
  2. Camera scans QR code → qr_scanner_node publishes on /qr_data
  3. THIS NODE receives /qr_data → POST /pick/scan to server (scan_complete)
  4. Server responds with pick_task_id → this node commands the pick mechanism
  5. Pick mechanism finishes → POST /pick/{id}/complete to server (pick_complete)
  6. Dashboard receives the signal, robot continues to next waypoint

Usage (after colcon build):
    ros2 run agv_controller warehouse_mission --ros-args \
        -p server_url:=http://192.168.1.57:8000 \
        -p robot_code:=AMR_01
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import requests
import threading
import time


class WarehousePickHandler(Node):
    def __init__(self):
        super().__init__("warehouse_pick_handler")

        # Parameters
        self.declare_parameter("server_url", "http://192.168.1.57:8000")
        self.declare_parameter("robot_code", "AMR_01")

        self.server_url = self.get_parameter("server_url").value
        self.robot_code = self.get_parameter("robot_code").value

        # Subscribe to QR scan data from qr_scanner_node
        self.create_subscription(String, "/qr_data", self.on_qr_scanned, 10)

        # Publisher: pick mechanism command (True = start picking)
        self.pick_cmd_pub = self.create_publisher(Bool, "/pick_mechanism/cmd", 10)

        # Subscriber: pick mechanism done signal (True = picking finished)
        self.create_subscription(Bool, "/pick_mechanism/done", self.on_pick_done, 10)

        # Publisher: status for monitoring
        self.status_pub = self.create_publisher(String, "/pick_handler/status", 10)

        # State
        self.picking = False
        self.current_pick_task_id = None
        self.cooldown_time = 3.0
        self.last_scan_time = 0.0

        self.get_logger().info(
            f"Pick Handler ready — server={self.server_url} robot={self.robot_code}"
        )

    # ── QR Scan Received ──────────────────────────────────────────────

    def on_qr_scanned(self, msg):
        """
        Called when qr_scanner_node detects a QR code.
        Reports 'scan complete' to the server.
        If there's an active pick task at this location, starts the pick mechanism.
        """
        qr_code = msg.data
        now = time.time()

        # Cooldown to prevent duplicate scans
        if now - self.last_scan_time < self.cooldown_time:
            return
        self.last_scan_time = now

        if self.picking:
            self.get_logger().info(f"Ignoring scan '{qr_code}' — pick in progress")
            return

        self.get_logger().info(f"QR scanned: {qr_code} → reporting to server")
        self.publish_status(f"SCAN_COMPLETE {qr_code}")

        # Report scan to server in a background thread (don't block the callback)
        threading.Thread(
            target=self._report_scan_and_pick, args=(qr_code,), daemon=True
        ).start()

    def _report_scan_and_pick(self, qr_code):
        """Send scan_complete to server, then trigger pick mechanism if applicable."""
        try:
            resp = requests.post(
                f"{self.server_url}/pick/scan",
                json={"qr_code": qr_code, "robot_code": self.robot_code},
                timeout=5,
            )
            resp.raise_for_status()
            data = resp.json()

            if data.get("status") == "scanned" and data.get("pick_task_id"):
                # Server confirmed an active pick task at this location
                pick_task_id = data["pick_task_id"]
                self.get_logger().info(
                    f"Pick task #{pick_task_id} confirmed — starting pick mechanism"
                )
                self.current_pick_task_id = pick_task_id
                self.picking = True
                self.publish_status(f"PICKING task={pick_task_id}")

                # Command the pick mechanism to start
                cmd = Bool()
                cmd.data = True
                self.pick_cmd_pub.publish(cmd)

            else:
                # No active pick task — just a normal scan log
                self.get_logger().info(
                    f"Scan logged (no active pick task): {data.get('message', '')}"
                )
                self.publish_status(f"SCAN_LOGGED {qr_code}")

        except Exception as e:
            self.get_logger().error(f"Scan report failed: {e}")
            self.publish_status(f"SCAN_ERROR {e}")

    # ── Pick Mechanism Done ───────────────────────────────────────────

    def on_pick_done(self, msg):
        """
        Called when the pick mechanism signals it has finished.
        Reports 'pick complete' to the server so the dashboard advances.
        """
        if not msg.data:
            return
        if not self.picking or self.current_pick_task_id is None:
            self.get_logger().warn("Pick done received but no active pick task")
            return

        pt_id = self.current_pick_task_id
        self.get_logger().info(
            f"Pick mechanism done → reporting pick_complete for task #{pt_id}"
        )
        self.publish_status(f"PICK_COMPLETE task={pt_id}")

        # Report to server in background thread
        threading.Thread(
            target=self._report_pick_complete, args=(pt_id,), daemon=True
        ).start()

    def _report_pick_complete(self, pick_task_id):
        """Send pick_complete to server."""
        try:
            resp = requests.post(
                f"{self.server_url}/pick/{pick_task_id}/complete",
                timeout=5,
            )
            resp.raise_for_status()
            data = resp.json()

            if data.get("order_complete"):
                self.get_logger().info("Order fully completed!")
                self.publish_status("ORDER_COMPLETE")
            else:
                self.get_logger().info(
                    f"Task #{pick_task_id} picked — waiting for next waypoint"
                )
                self.publish_status(f"PICKED task={pick_task_id}")

        except Exception as e:
            self.get_logger().error(f"Pick complete report failed: {e}")
            self.publish_status(f"PICK_ERROR {e}")
        finally:
            self.picking = False
            self.current_pick_task_id = None

    # ── Helpers ───────────────────────────────────────────────────────

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WarehousePickHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down pick handler")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
