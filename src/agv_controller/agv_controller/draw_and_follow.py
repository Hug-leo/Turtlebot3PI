import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import threading
import time

# Global variable to store the list of plotted waypoints
waypoints = []


# ==============================================================
# CREATE AN AUXILIARY NODE SOLELY FOR RVIZ INTERACTION (AVOID CONFLICTS)
# ==============================================================
class UIBridge(Node):
    def __init__(self):
        super().__init__("ui_bridge_node")
        # Channel to publish drawn path to RViz2
        self.path_pub = self.create_publisher(Path, "/drawn_path", 10)
        # Channel to listen for mouse click events
        self.create_subscription(PointStamped, "/clicked_point", self.point_cb, 10)

    def point_cb(self, msg):
        """Runs automatically whenever you click 'Publish Point' in RViz2"""
        global waypoints

        # Convert PointStamped to PoseStamped (Nav2 requires Pose)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0  # Default orientation

        waypoints.append(pose)

        # Immediately draw connecting path between points in RViz2
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = waypoints
        self.path_pub.publish(path_msg)

        print(
            f"Waypoint {len(waypoints)} added: (x={msg.point.x:.2f}, y={msg.point.y:.2f})"
        )


# ==============================================================
# MAIN PROGRAM
# ==============================================================
def main():
    global waypoints
    rclpy.init()

    # 1. Initialize the UI Node for drawing
    ui_node = UIBridge()

    # 2. Initialize Nav2 API for robot control
    nav = BasicNavigator()

    # Only allow UI Node to spin in background to catch mouse events, keep nav separate
    spin_thread = threading.Thread(target=rclpy.spin, args=(ui_node,), daemon=True)
    spin_thread.start()

    print("\nConnecting to Nav2 system...")
    nav.waitUntilNav2Active()

    # Main loop
    while rclpy.ok():
        waypoints.clear()

        # Clear previous drawn path in RViz (if any)
        empty_path = Path()
        empty_path.header.frame_id = "map"
        empty_path.header.stamp = ui_node.get_clock().now().to_msg()
        ui_node.path_pub.publish(empty_path)

        print("\n" + "=" * 60)
        print("MODE: DRAW TRAJECTORY WITH MOUSE (WAYPOINT FOLLOWING)")
        print("1. Use the 'Publish Point' tool in RViz2.")
        print("2. Click multiple points to build a path.")
        print("3. Press ENTER in this terminal to start moving.")
        print("=" * 60 + "\n")

        # Wait for user to click points and confirm
        try:
            while True:
                ans = (
                    input(
                        "Press [ENTER] to follow trajectory | Type [C] to clear and redraw: "
                    )
                    .strip()
                    .lower()
                )

                if ans == "c":
                    print("Trajectory cleared. Please plot new waypoints from scratch!")
                    break  # Exit inner loop to reset waypoint list

                elif ans == "":
                    if len(waypoints) == 0:
                        print("No points clicked on RViz2 yet!")
                        continue

                    print(
                        f"\nCONFIRMED! Robot starting trajectory through {len(waypoints)} waypoints..."
                    )

                    # Call Nav2 API: goThroughPoses (no longer throws errors!)
                    nav.goThroughPoses(waypoints)

                    # Wait for robot to finish
                    while not nav.isTaskComplete():
                        # Get feedback to display progress (optional)
                        feedback = nav.getFeedback()
                        if feedback:
                            print(
                                f"Distance remaining: {feedback.distance_remaining:.2f} m",
                                end="\r",
                            )
                        time.sleep(0.5)

                    # Result
                    result = nav.getResult()
                    if result == TaskResult.SUCCEEDED:
                        print("\nEXCELLENT! Full trajectory completed!")
                    else:
                        print(
                            "\nWARNING: Failed mid-trajectory (obstacle or cancelled)."
                        )

                    break  # Finished, exit to start new drawing loop

        except KeyboardInterrupt:
            print("\nExiting program...")
            break

    # Clean up both nodes on exit
    nav.destroy_node()
    ui_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
