#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from filterpy.kalman import KalmanFilter
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point, PoseArray
from sort_mpc_nav.msg import TrackedPedestrian, TrackedPedestrianArray
import numpy as np

class SortTracker(Node):
    def __init__(self):
        super().__init__('sort_tracker')
        self.filters = {}
        self.initialized = {}

        # One subscription for all pedestrians
        self.create_subscription(
            PoseArray,
            '/pedestrian_detections',
            self.detections_callback,
            10
        )

        # One publisher for all pedestrians
        self.tracked_pub = self.create_publisher(
            TrackedPedestrianArray,
            '/tracked_states',
            10
        )

        self.get_logger().info('SORT tracker started')

    def init_kalman_filter(self):
        kf = KalmanFilter(dim_x = 4, dim_z = 2)
        dt = 0.1 # 100ms to match our sim
    
        # Transition matrix F
        kf.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
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

    def detections_callback(self, msg):
        tracked_array = TrackedPedestrianArray()
        for i, pose in enumerate(msg.poses):
            if i not in self.filters:
                self.filters[i] = self.init_kalman_filter()
                self.initialized[i] = False
            
            kf = self.filters[i]

            if not self.initialized[i]:
                kf.x = np.array([[pose.position.x], [pose.position.y], [0.0], [0.0]])
                self.initialized[i] = True
                continue
            else:
                kf.predict()
                measurement = np.array([
                    [pose.position.x],
                    [pose.position.y]
                ])
                kf.update(measurement)
                # estimated state
                px = kf.x[0, 0]
                py = kf.x[1, 0]
                vx = kf.x[2, 0]
                vy = kf.x[3, 0]

            # extract state
            px = kf.x[0, 0]
            py = kf.x[1, 0]
            vx = kf.x[2, 0]
            vy = kf.x[3, 0]

            # build TrackedPedestrian message
            tracked = TrackedPedestrian()
            tracked.id = i
            tracked.px = px
            tracked.py = py
            tracked.vx = vx
            tracked.vy = vy
            tracked_array.pedestrian.append(tracked)

        self.tracked_pub.publish(tracked_array)

def main(args = None):
    rclpy.init(args = args)
    node = SortTracker()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()