import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor, environment="sim"):

        super().__init__("localizer")
        
        # Create the QoS profile dependant on the environment
        if environment == 'sim':
            odom_qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST)
        else:
            odom_qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        # Part 3: subscribe to the position sensor topic (Odometry)
        if localizationType == rawSensor:
            self.create_subscription(odom, '/odom', self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg):
        # Part 3: Read x,y, theta, and record the stamp
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y

        quat = [
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w
        ]

        theta = euler_from_quaternion(quat)

        # Log the data
        self.loc_logger.log_values([x, y, theta, Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
    
