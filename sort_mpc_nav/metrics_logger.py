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