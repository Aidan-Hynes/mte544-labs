import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, linear_kp=0.2, linear_kv=0.2, linear_ki=0.2,
                 angular_kp=0.2, angular_kv=0.2, angular_ki=0.2,
                 ctrl_type=P, env="sim"):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(
            controller_type=ctrl_type,
            kp=linear_kp,
            kv=linear_kv,
            ki=linear_ki,
            filename="linear.csv"
        )
        self.PID_angular=PID_ctrl(
            controller_type=ctrl_type,
            kp=angular_kp,
            kv=angular_kv,
            ki=angular_ki,
            filename="angular.csv"
        )

        self.env = env
    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # Part 4: Add saturation limits for the robot linear and angular velocity

        # for simulation
        if self.env == "sim":
            v_max = 0.22
            w_max = 2.84

        elif self.env == "real":
            v_max=0.31
            w_max=1.9

        linear_vel = v_max if linear_vel > v_max else (-v_max if linear_vel < -v_max else linear_vel)
        angular_vel = w_max if angular_vel > w_max else (-w_max if angular_vel < -w_max else angular_vel)
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, linear_kp=0.2, linear_kv=0.2, linear_ki=0.2,
                 angular_kp=0.2, angular_kv=0.2, angular_ki=0.2,
                 ctrl_type=P, env="sim"):
        super().__init__(
            linear_kp=linear_kp,
            linear_kv=linear_kv,
            linear_ki=linear_ki,
            angular_kp=angular_kp,
            angular_kv=angular_kv,
            angular_ki=angular_ki,
            ctrl_type=ctrl_type,
            env=env
        )
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # Part 5: Add saturation limits for the robot linear and angular velocity

        # Saturation limits (TurtleBot3 Burger: 0.22 m/s, 2.84 rad/s; TurtleBot 4: 0.31 m/s, 1.90 rad/s)
        if self.env == "sim":
            v_max = 0.22
            w_max = 2.84

        elif self.env == "real":
            v_max=0.31
            w_max=1.9

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
