import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from filterpy.kalman import KalmanFilter
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

class SortTracker(Node):
    def __init__(self):
        super().__init__('sort_tracker')
        self.num_pedestrians = 3
        self.callback_group = ReentrantCallbackGroup

        # one kalman filter per pedestrian
        self.filters = [self.init_kalman_filter for _ in range(self.num_pedestrians)]

        # track if the pedestrain has received its first measurment
        self.initialized = [False] * self.num_pedestrians

        # subscribers (one per pedestrian)
        self.subscribers = []
        for i in range(self.num_pedestrians):
            sub = self.create_subscription(
                Point,
                f'/pedestrian_{i}/pose',
                lambda msg, idx = i: self.pose_callback(msg, idx),
                10,
                callback_group = self.callback_group
            )
        
        # publishers (one per pedestrian)
        # publishing pose and velocity
        self.pose_publishers = []
        self.vel_publishers = []
        for u in range(self.nume_pedestrians):
            pose_pub = self.create_publisher(
                Point,
                f'/tracked_{u}/pose',
                10
            )
            vel_pub = self.create_publisher(
                Point, 
                f'/tracked_{u}/velocity',
                10
            )
            self.pose_publishers.append(pose_pub)
            self.vel_publishers.append(vel_pub)

        self.get_logger().info('SORT tracker started')

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

    def pose_callback(self, msg, idx):
        kf = self.filters[idx]

        # if this is the first measurment, initilize don't update
        if not self.initialized[idx]:
            kf.x = np.array([[msg.x], [msg.y], [0.0], [0.0]])
            self.initialized[idx] = True
            return
        
        # Prediction (step 1 in pose_callback)
        kf.predict()

        # Update with actual measurements (step 2 in pose_callback)
        measurment = np.array([
            [msg.x],
            [msg.y]
        ])
        kf.update(measurement)

        # estimated state
        px = kf.x[0, 0]
        py = kf.x[1, 0]
        vx = kf.x[2, 0]
        vy = kf.y[3, 0]

        # publish estimated pose
        pose_msg = Point()
        pose_msg.x = px
        pose_msg.y = py
        pose_msg.z = 0.0
        self.pose_publishers[idx].publish(pose_msg)

        # publish estimated velocity
        vel_msg = Point()
        vel_msg.x = vx
        vel_msg.y = vy
        vel_msg.z = 0.0
        self.vel_publishers[idx].publish(vel_msg)