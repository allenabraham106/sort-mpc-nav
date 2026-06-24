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