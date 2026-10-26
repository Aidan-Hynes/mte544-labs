import argparse
import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0
class localization(Node):
    
    def __init__(self, odom_qos, localizationType=rawSensor):

        super().__init__("localizer")
        
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

# Part 3: Guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
    
if __name__ == "__main__":
    argParser = argparse.ArgumentParser(description="Localization node")
    argParser.add_argument("--env", default="sim", help="Environment to run in (sim or real)")
    args = argParser.parse_args()

    init()
    localizer=localization(rawSensor, args.env)

    try:
        spin(localizer)
    
    except KeyboardInterrupt:
        pass

    finally:
        localizer.loc_logger.save_log()
        localizer.destroy_node()
