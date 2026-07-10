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
    