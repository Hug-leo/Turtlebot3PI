#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import matplotlib.pyplot as plt
import math
import time


class OdomRecorder(Node):
    def __init__(self):
        super().__init__("odom_recorder")
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.callback, 10
        )
        self.x_data = []
        self.y_data = []

    def callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)


def create_pose(navigator, x, y, theta):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()
    recorder = OdomRecorder()

    print("Initializing system...")
    nav.waitUntilNav2Active()

    # Waypoints
    goal_A = create_pose(nav, -0.546, -0.512, -math.pi / 2)
    goal_B = create_pose(nav, 0.997, -0.417, 0.0)
    goal_C = create_pose(nav, 1.010, 0.557, math.pi / 2)
    goal_D = create_pose(nav, -0.534, 0.278, math.pi)
    waypoints = [goal_B, goal_C, goal_D, goal_A]

    print("Starting 5 experimental laps...")

    for i in range(5):
        print(f"\n>>> LAP {i+1}...")
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            rclpy.spin_once(
                recorder, timeout_sec=0.1
            )  # Collect coordinates continuously

    print("\n[DONE] Preparing to export chart...")

    # --- Plot with actual trajectory only ---
    if len(recorder.x_data) > 0:
        plt.figure(figsize=(8, 8))
        # Plot actual path in blue
        plt.plot(
            recorder.x_data,
            recorder.y_data,
            "b-",
            linewidth=1.5,
            label="Actual AGV trajectory",
        )

        # Mark start point
        plt.scatter(
            recorder.x_data[0],
            recorder.y_data[0],
            color="green",
            s=100,
            label="Start point",
        )

        plt.title(
            "AGV MOVEMENT TRAJECTORY (ODOMETRY DATA)", fontsize=14, fontweight="bold"
        )
        plt.xlabel("X coordinate (m)")
        plt.ylabel("Y coordinate (m)")
        plt.axis("equal")  # Maintain aspect ratio to avoid distortion
        plt.grid(True, linestyle=":", alpha=0.7)
        plt.legend()

        # Save image and display
        plt.savefig("quy_dao_agv_thuc_te.png", dpi=300)
        print("Image 'quy_dao_agv_thuc_te.png' saved.")
        plt.show()
    else:
        print("No data received to plot.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
