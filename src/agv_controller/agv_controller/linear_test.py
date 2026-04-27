#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import matplotlib.pyplot as plt
import math
import os


# --- ODOM DATA RECORDER NODE ---
class OdomRecorder(Node):
    def __init__(self):
        super().__init__("odom_recorder_final")
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.callback, 10
        )
        self.x_data = []
        self.y_data = []

    def callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)


def p(nav, x, y):
    """Create PoseStamped"""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()
    recorder = OdomRecorder()

    print("Waiting for Nav2 to become active...")
    nav.waitUntilNav2Active()

    # --- TARGET COORDINATES ---
    # We measure error at the final point (P1)
    goal_x, goal_y = -0.928, -0.307
    p1 = p(nav, goal_x, goal_y)
    p2 = p(nav, 1.013, -0.337)

    print("Starting error test route (P1 -> P2 -> P1)...")

    # Send waypoint navigation command
    nav.followWaypoints([p1, p2, p1])

    # --- MONITORING LOOP (no more distance_remaining error) ---
    while not nav.isTaskComplete():
        # Update data from OdomRecorder Node
        rclpy.spin_once(recorder, timeout_sec=0.1)

        feedback = nav.getFeedback()
        if feedback:
            # followWaypoints feedback only provides current_waypoint
            print(
                f"Robot moving to Waypoint: {feedback.current_waypoint + 1}", end="\r"
            )

    print("\n[DONE] Robot stopped. Calculating error...")

    # --- EUCLIDEAN ERROR CALCULATION ---
    if len(recorder.x_data) > 0:
        actual_x = recorder.x_data[-1]
        actual_y = recorder.y_data[-1]

        # Calculate distance between desired target and actual robot stop position
        error = math.sqrt((goal_x - actual_x) ** 2 + (goal_y - actual_y) ** 2)

        print("\n" + "=" * 40)
        print("          STOP POINT ERROR REPORT")
        print(f" Target coordinates:  x = {goal_x:.4f}, y = {goal_y:.4f}")
        print(f" Actual coordinates:  x = {actual_x:.4f}, y = {actual_y:.4f}")
        print(f" ABSOLUTE ERROR: {error*100:.2f} cm")
        print("=" * 40)

        # --- PLOT TRAJECTORY AND SAVE IMAGE ---
        plt.figure(figsize=(10, 6))
        # Plot actual path in blue
        plt.plot(
            recorder.x_data,
            recorder.y_data,
            "b-",
            linewidth=1.5,
            label="Actual trajectory",
        )
        # Mark target point (red X)
        plt.scatter(
            goal_x,
            goal_y,
            color="red",
            marker="x",
            s=150,
            label="Desired target (Goal)",
        )
        # Mark actual stop position (green dot)
        plt.scatter(
            actual_x,
            actual_y,
            color="green",
            s=100,
            label="Actual stop position",
            zorder=5,
        )

        plt.title("ROBOT NAVIGATION ERROR ANALYSIS", fontsize=14)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis("equal")
        plt.grid(True, linestyle="--", alpha=0.6)
        plt.legend()

        # Save file to user home directory
        save_path = os.path.expanduser("~/error_test_final.png")
        plt.savefig(save_path, dpi=300)
        print(f"Analysis chart saved at: {save_path}")
    else:
        print("Error: No coordinate data collected.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
