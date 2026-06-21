import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from filterpy.kalman import KalmanFilter
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

class SortTracker(Node):
    def __init__(self):
        super().__init__('sort_tracker')\

    def init_kalman_filter(self):
        kf = KalmanFilter(dim_x = 4, dim_z = 2)
        dt = 0.1 # 100ms to match our sim
    
        # Transition matrix F
        kf.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Measurement matrix H (only measures position)
        kf.H = np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, 0]
        ])

        # Measurement noise (only measures position)
        kf.R = np.eye(2) * 0.5

        # Process noise (how much can the velocity change)
        kf.Q = np.eye(4) * 0.1

        # Initial State Covariance (how uncertain are we at the start)
        kf.P = np.eye(4) * 10.0

        kf.x = np.zeros((4, 1))

        return kf



        
        