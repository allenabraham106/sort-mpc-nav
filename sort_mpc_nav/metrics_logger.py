#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from sort_mpc_nav.msg import TrackedPedestrian, TrackedPedestrianArray
import numpy as np
import csv
import time 
import os
import math


class MetricsLogger(Node):
    def __init__(self):
        super().__init__("metrics_logger")

        # Timing lists
        self.perception_latencies = []
        self.tracking_latencies = []
        self.mpc_solve_times = []
        self.clearances = []

        # Timestampes of compute latency
        self.detection_stamp = None
        self.tracked_stamp = None

        # Goal tracking 
        self.goal_x = None
        self.goal_y = None
        self.goal_attempts = 0
        self.goal_success = 0

        # Robot Position 
        self.robot_x = 0.0
        self.robot_y = 0.0

        # CSV setup
        log_path = os.path.expanduser('~/metric_logger.csv')
        self.csv_file = open(log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'perception_latency_ms',
            'tracking_latency_ms', 
            'clearance_m',
            'goal_reached'
        ])

        # Subscriber
        self.create_subscription(
            PoseArray,
            '/pedestrian_detections',
            self.detection_callback,
            10
        )
        
        self.create_subscription(
            TrackedPedestrianArray, 
            '/tracked_states',
            self.tracked_callback, 
            10
        )
        
        self.create_subscription(
            Marker,
            '/robot_marker',
            self.robot_callback, 
            10
        )

        self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            10
        )

        # Timer checks if the goal has been reached
        self.create_timer(1.0, self.check_goal_reached)
        self.get_logger().info("Metrics logger startetd, writing to csv")

    def detection_callback(self, msg):
        self.detection_stamp = time.perf_counter()

    def tracked_callback(self, msg):
        now = time.perf_counter()
        if self.detection_stamp is not None:
            latency_ms = (now - self.detection_stamp) * 1000
            self.tracking_latencies.append(latency_ms)

            min_clearance = float('inf')
            for pedestrian in msg.pedestrian:
                dist = math.sqrt(
                    (pedestrian.px - self.robot_x)**2 + (pedestrian.py - self.robot_y)**2
                )
                if dist < min_clearance:
                    min_clearance = dist
            
            if min_clearance != float('inf'):
                self.clearances.append(min_clearance)

            self.csv_writer.writerow([
                now,
                latency_ms if self.detection_stamp is not None else '',
                min_clearance if min_clearance != float('inf') else '',
                False
            ])

        self.tracked_stamp = now

    def robot_callback(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y

    def goal_callback(self, msg):
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.goal_attempts += 1
        self.get_logger().info(f'Goal set: x={self.goal_x:.2f}, y={self.goal_y:.2f}')
    
    def check_goal_reached(self):
        if self.goal_x is None or self.goal_y is None:
            return

        distance = math.sqrt(
            (self.robot_x - self.goal_x)**2 + (self.robot_y - self.goal_y)**2
        )

        if distance < 0.3:
            self.goal_success += 1
            self.get_logger().info(f'Goal reached! Success rate: {self.goal_success}/{self.goal_attempts}')
            # Reset 
            self.goal_x = None
            self.goal_y = None

def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()  # important — flush and close CSV on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()