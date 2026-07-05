#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from sort_mpc_nav.msg import TrackedPedestrian, TrackedPedestrianArray
import numpy as np
import casadi as ca # this is for trajectory planning

class MPC_Planner(Node):
    def __init__(self):
        super().__init__('mpc_planner')
        self.robot_x = 0.0 # where the robot is
        self.robot_y = 0.0
        self.current_num_peds = 0
        self.goal_x = None # where the user clicked (none until clicked)
        self.goal_y = None
        self.tracked_states = []  # list of tracked states

        self.create_subscription(
            TrackedPedestrianArray,
            '/tracked_states',
            self.tracked_callback,
            10
        )
        
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
        self.timer = self.create_timer(
            0.3, 
            self.plan
        )
        self.publish_initial_robot()

    def goal_callback(self, msg):
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.get_logger().info(f'New Goal Clicked: x={self.goal_x:.2f}, y={self.goal_y:.2f}')

    def tracked_callback(self, msg):
        self.tracked_states = msg.pedestrians

    def setup_solver(self, num_peds):
        self.N = 10 # how many steps to look ahead
        self.dt = 0.3 # seconds per step
        self.max_speed = 1.5 # max speed of robot

        opti = ca.Opti() # initialize CasADi

        # Decision Variables, robots x,y position at n+1 timesteps
        X = opti.variable(2, self.N + 1)

        # Parameters 
        x0 = opti.parameter(2)
        goal = opti.parameter(2)
        ped_preds = opti.parameter(2, self.N * num_peds)

        # Cost Function
        cost = 0
        for k in range(self.N + 1):
            cost += ca.sumsqr(X[:, k] - goal) # distance to the goal
        
        for k in range(self.N):
            cost += ca.sumsqr(X[:, k+1] - X[:, k]) * 3.0 # smoothness buffer

        for k in range(self.N):
            for p in range(num_peds):
                ped_pos = ped_preds[:, k * num_peds + p]
                dist_sq = ca.sumsqr(X[:, k + 1] - ped_pos)
                cost += 2.0 / (dist_sq + 0.1) # cost for going close to pedestrians

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
        if len(self.tracked_states) == 0:
            return

        num_peds = len(self.tracked_states)
        
        if num_peds != self.current_num_peds:
            self.setup_solver(num_peds)
            self.current_num_peds = num_peds

        ped_preds = np.zeros((2, self.N * num_peds))
        for p, peds in enumerate(self.tracked_states):
            for k in range(self.N):
                t = (k+ 1) * self.dt
                col = k * num_peds + p
                ped_peds[0, col] = ped.px + ped.vx * t
                ped_peds[1, col] = ped.py + ped.vy * t
            
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

    def publish_initial_robot(self):
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