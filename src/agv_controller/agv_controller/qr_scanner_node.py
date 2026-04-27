import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from pyzbar.pyzbar import decode
import time


class QRCodeScanner(Node):
    def __init__(self):
        super().__init__("qr_code_scanner")

        # 1. Create Publisher to broadcast QR data on topic '/qr_data'
        self.publisher_ = self.create_publisher(String, "/qr_data", 10)

        # 2. Initialize Camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open Camera! Check USB port.")
        else:
            self.get_logger().info(
                "Camera opened successfully! Running background scan..."
            )

        # 3. Create Timer for continuous image scanning (10 times/sec ~ 10Hz)
        self.timer = self.create_timer(0.1, self.scan_callback)

        # 4. Anti-spam variables (3-second cooldown)
        self.last_scanned_data = ""
        self.last_scan_time = 0.0
        self.cooldown_time = 3.0

    def scan_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        # Scan QR codes
        qr_codes = decode(frame)

        for qr in qr_codes:
            qr_data = qr.data.decode("utf-8")
            current_time = time.time()

            # Anti-spam: report if new code or cooldown elapsed
            if (qr_data != self.last_scanned_data) or (
                current_time - self.last_scan_time > self.cooldown_time
            ):
                self.get_logger().info(f"Package detected: {qr_data}")

                # Pack and publish on ROS 2 topic
                msg = String()
                msg.data = qr_data
                self.publisher_.publish(msg)

                # Update cache
                self.last_scanned_data = qr_data
                self.last_scan_time = current_time

    def destroy_node(self):
        # Safely release camera when node is destroyed
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeScanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown command received (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
