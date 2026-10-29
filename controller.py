import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, ctrl_type=P):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(ctrl_type, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(ctrl_type, kap, kav, kai, filename_="angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # Part 4: Add saturation limits for the robot linear and angular velocity

        # for simulation
        v_max = 0.22
        w_max = 2.84

        v_max_robot=0.31
        w_max_robot=1.9

        linear_vel = v_max if linear_vel > v_max else (-v_max if linear_vel < -v_max else linear_vel)
        angular_vel = w_max if angular_vel > w_max else (-w_max if angular_vel < -w_max else angular_vel)
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, ctrl_type=P):
        
        super().__init__(klp, klv, kli, kap, kav, kai, ctrl_type)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # Part 5: Add saturation limits for the robot linear and angular velocity

        # Saturation limits (TurtleBot3 Burger: 0.22 m/s, 2.84 rad/s; TurtleBot 4: 0.31 m/s, 1.90 rad/s)
        v_max = 0.22
        w_max = 2.84

        v_max_robot=0.31
        w_max_robot=1.9

        linear_vel = v_max if linear_vel > v_max else (-v_max if linear_vel < -v_max else linear_vel)
        angular_vel = w_max if angular_vel > w_max else (-w_max if angular_vel < -w_max else angular_vel)
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
