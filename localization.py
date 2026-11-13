import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):
    
    def __init__(self, type, dt, q=0.5, r=0.5, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay", "kf_vx", "kf_w", "odom_x", "odom_y", "kf_x", "kf_y", "stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt, q, r)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt, q, r):
        
        # Part 3: Set up the quantities for the EKF 
        
        # Initial State: [x, y, theta, w, v, vdot]
        x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Process noise covariance Q
        Q = np.eye(6) * q

        R = np.eye(4) * r
        
        P = np.eye(6) * 1.0 # initial covariance
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        v_meas = odom_msg.twist.twist.linear.x
        w_meas = odom_msg.twist.twist.angular.z
        ax_meas = imu_msg.linear_acceleration.x
        ay_meas = imu_msg.linear_acceleration.y
        
        z = np.array([v_meas, w_meas, ax_meas, ay_meas])
        
        # Implement the two steps for estimation
        self.kf.predict()
        self.kf.update(z)
        
        # Get the estimate
        xhat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        self.pose=np.array([xhat[0], xhat[1], xhat[2], odom_msg.header.stamp])

        # Part 4: log your data
        stamp_sec = odom_msg.header.stamp.sec + (odom_msg.header.stamp.nanosec * 1e-9)
        
        # Calculate predicted ay from the EKF states (ay = v * w)
        kf_ay = xhat[4] * xhat[3]
        
        self.loc_logger.log_values([
            ax_meas,      # imu_ax
            ay_meas,      # imu_ay
            xhat[5],      # kf_ax (vdot)
            kf_ay,        # kf_ay (v*w)
            xhat[4],      # kf_vx (v)
            xhat[3],      # kf_w (w)
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            xhat[0],      # kf_x (x)
            xhat[1],      # kf_y (y)
            stamp_sec     # stamp
        ])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
