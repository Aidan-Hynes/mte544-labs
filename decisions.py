# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner

# You may add any other imports you may need/want to use below
from controller import controller, trajectoryController, P, PD, PI, PID


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos, goalPoint, rate=10, motion_type=POINT_PLANNER, trajectory_type="parabola", ctrl_type=P):

        super().__init__("decision_maker")

        # Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos)

        publishing_period=1/rate

        self.motion_type = motion_type
        self.tolerance = 0.08 # tighter tolerance for final destination
        
        # Instantiate the controller
        # Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6, ctrl_type=ctrl_type)
            self.planner=planner(POINT_PLANNER)
            self.goal = self.planner.plan(goalPoint)
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.1, klv=0.5, kap=1.2, kav=0.6, ctrl_type=ctrl_type)
            self.planner=planner(TRAJECTORY_PLANNER, traj_type=trajectory_type)
            self.goal = self.planner.plan()
        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(qos, rawSensor)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # Part 3: Run the localization node
        spin_once(self.localizer)

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # Part 3: Check if you reached the goal
        current_pose = self.localizer.getPose()
        x, y = current_pose[0], current_pose[1]

        # Determine the final stopping point to check for arrival
        if self.motion_type == TRAJECTORY_PLANNER:
            final_target = self.goal[-1]
        else:
            final_target = self.goal
        

        dist_to_final = ((x - final_target[0]) ** 2 + (y - final_target[1]) ** 2) ** 0.5

        # Check if destination is reached
        if dist_to_final < self.tolerance:
            print("reached goal")
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)

            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()

            if hasattr(self, 'timer') and self.timer is not None:
                self.timer.cancel()

            raise SystemExit

        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        print(f"Dist to goal: {dist_to_final:.3f} m | Cmd Vel: {velocity:.3f}")

        #TODO Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = float(velocity)
        vel_msg.angular.z = float(yaw_rate)
        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    controller_types = {
    "P": P,
    "PD": PD,
    "PI": PI,
    "PID": PID
}

    ctrl_type = controller_types[args.controller]

    if args.env == 'sim':
        qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
    else:
        qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot

    # set the cmd_vel topic name
    cmd_topic = "/cmd_vel"

    # define a target goal point
    goal_point = [1.0, 1.0]

    if args.motion.lower() == "point":
        DM = decision_maker(
            publisher_msg=Twist,
            publishing_topic=cmd_topic,
            qos=qos_profile,
            goalPoint=goal_point,
            motion_type=POINT_PLANNER,
            trajectory_type= args.traj,
            ctrl_type=ctrl_type
        )
    elif args.motion.lower() == "trajectory":
        DM = decision_maker(
            publisher_msg=Twist,
            publishing_topic=cmd_topic,
            qos=qos_profile,
            goalPoint=None,
            motion_type=TRAJECTORY_PLANNER,
            trajectory_type = args.traj,
            ctrl_type=ctrl_type
        )
    else:
        print("invalid motion type", file=sys.stderr)        
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory, and environment") 
    argParser.add_argument("--motion", type=str, default="point")
    argParser.add_argument("--traj", type=str, default="parabola", choices=["parabola", "sigmoid"])
    argParser.add_argument("--controller", type=str.upper, default="P", choices=["P", "PD", "PI", "PID"], help="Controller type: P, PD, PI, or PID")
    argParser.add_argument("--env", type=str, default="sim", help="Environment to run in (sim or real)")
    args = argParser.parse_args()

    main(args)
