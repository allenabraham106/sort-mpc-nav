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
        self.setup_solver()
        self.timer = self.create_timer(
            0.3, 
            self.plan
        )

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

        opti = ca.Opti() # initialize CasADi

        # Decision Variables, robots x,y position at n+1 timesteps
        X = opti.variable(2, self.N + 1)

        # Parameters 
        x0 = opti.parameter(2)
        goal = opti.parameter(2)
        ped_preds = opti.parameter(2, self.N * self.num_pedestrians)

        # Cost Function
        cost = 0
        for k in range(self.N + 1):
            cost += ca.sumsqr(X[:, k] - goal) # distance to the goal
        
        for k in range(self.N):
            cost += ca.sumsqr(X[:, k+1] - X[:, k]) * 0.1 # smoothness buffer

        for k in range(self.N):
            for p in range(self.num_pedestrians):
                ped_pos = ped_preds[:, k * self.num_pedestrians + p]
                dist_sq = ca.sumsqr(X[:, k + 1] - ped_pos)
                cost += 5.0 / (dist_sq + 0.1) # cost for going close to pedestrians

        opti.minimize(cost)

        # Constraints
        opti.subject_to(X[:, 0] == x0)
        for k in range(self.N):
            step = X[:, k + 1] - X[:, k]
            opti.subject_to(ca.sumsqr(step) <= (self.max_speed * self.dt) ** 2)
        opti.solver('ipopt', {'print_time': False, 'ipopt.print_level': 0})


        self.opti = opti
        self.X = X
        self.x0_param = x0
        self.goal_param = goal
        self.ped_preds_param = ped_preds

    def plan(self):
        # don't have a goal yet
        if self.goal_x is None or self.goal_y is None:
            return
        
        # don't have pedestrian data
        if len(self.ped_states) < self.num_pedestrians:
            return

        # project each pedestrian forward N steps
        ped_preds = np.zeros((2, self.N * self.num_pedestrians))
        for p in range(self.num_pedestrians):
            state = self.ped_states.get(p, {'px': 0.0, 'py': 0.0, 'vx': 0.0, 'vy': 0.0})
            for k in range(self.N):
                t = (k + 1) * self.dt
                col = k * self.num_pedestrians + p
                ped_preds[0, col] = state['px'] + state['vx'] * t
                ped_preds[1, col] = state['py'] + state['vy'] * t

        # set parameter values for the solve
        self.opti.set_value(self.x0_param, [self.robot_x, self.robot_y])
        self.opti.set_value(self.goal_param, [self.goal_x, self.goal_y])
        self.opti.set_value(self.ped_preds_param, ped_preds)

        # solve 
        try: 
            sol = self.opti.solve()
            path = sol.value(self.X)

            # update the robot position to the first step of the planned path
            self.robot_x = path[0,1]
            self.robot_y = path[1,1]

            # publish path as a marker array
            marker_array = MarkerArray()
            for k in range(self.N + 1):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'mpc_path'
                m.id = k
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = path[0, k]
                m.pose.position.y = path[1, k]
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0
                m.scale.x = 0.2
                m.scale.y = 0.2
                m.scale.z = 0.2
                m.color.r = 0.0 
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
                marker_array.markers.append(m)
            self.path_pub.publish(marker_array)

            # publish robot marker
            robot_marker = Marker()
            robot_marker.header.frame_id = 'map'
            robot_marker.header.stamp = self.get_clock().now().to_msg()
            robot_marker.ns = 'robot'
            robot_marker.id = 0
            robot_marker.type = Marker.CUBE
            robot_marker.action = Marker.ADD
            robot_marker.pose.position.x = self.robot_x
            robot_marker.pose.position.y = self.robot_y
            robot_marker.pose.position.z = 0.0
            robot_marker.pose.orientation.w = 1.0
            robot_marker.scale.x = 0.4
            robot_marker.scale.y = 0.4
            robot_marker.scale.z = 0.4
            robot_marker.color.r = 0.0
            robot_marker.color.g = 0.5
            robot_marker.color.b = 1.0
            robot_marker.color.a = 1.0
            self.robot_pub.publish(robot_marker)

        except Exception as e:
            self.get_logger().warn(f'MPC solve failed: {e}')

def main(args = None):
    rclpy.init(args = args)
    node = MPC_Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()