#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import casadi as ca # this is for trajectory planning

class MPC_Planner(Node):
    def __init__(self):
        super().__init__('mpc_planner')
        self.robot_x = 0.0 # where the robot is
        self.robot_y = 0.0
        self.goal_x = None # where the user clicked (none until clicked)
        self.goal_y = None
        self.ped_states = {} # latest positiion + velocity of the pedestrians
        self.num_pedestrians = 3

        # Subscribers for each pedestrians
        self.subscribers = []
        for i in range(self.num_pedestrians):
            pose_sub = self.create_subscription(
                Point, 
                f'/tracked_{i}/pose',
                lambda msg, idx=i: self.pose_callback(msg, idx),
                10     
            )
            vel_sub = self.create_subscription(
                Point, 
                f'/tracked_{i}/velocity',
                lambda msg, idx=i: self.vel_callback(msg, idx),
                10
            )
            self.subscribers.append(pose_sub)
            self.subscribers.append(vel_sub)
        
        # Clicked point subscriber for rviz
        clicked_sub = self.create_subscription(
            PointStamped,
            f'/clicked_point',
            self.goal_callback,
            10
        )


        self.path_pub = self.create_publisher(MarkerArray, '/mpc_path', 10)
        self.robot_pub = self.create_publisher(Marker, '/robot_marker', 10)

        self.get_logger().info('MPC working')


    def goal_callback(self, msg):
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.get_logger().info(f'New Goal Clicked: x={self.goal_x:.2f}, y={self.goal_y:.2f}')

    def pose_callback(self, msg, idx):
        if idx not in self.ped_states:
            self.ped_states[idx] = {'px': 0.0, 'py':0.0, 'vx':0.0, 'vy':0.0}
        self.ped_states[idx]['px'] = msg.x
        self.ped_states[idx]['py'] = msg.y

    def vel_callback(self, msg, idx):
        if idx not in self.ped_states:
            self.ped_states[idx] = {'px': 0.0, 'py':0.0, 'vx':0.0, 'vy':0.0}
        self.ped_states[idx]['vx'] = msg.x
        self.ped_states[idx]['vy'] = msg.y

    def setup_solver(self):
        self.N = 10 # how many steps to look ahead
        self.dt = 0.3 # seconds per step
        self.max_speed = 1.5 # max speed of robot

        octi = ca.Octi(). # initialize CasADi

        # Decision Variables, robots x,y position at n+1 timesteps
        X = opti.variables(2, self.N + 1)

        # Parameters 
        x0 = opti.parameter(2)
        goal = opti.parameter(2)
        ped_pred = opti.parameter(2, self.N * self.num_pedestrians)

        