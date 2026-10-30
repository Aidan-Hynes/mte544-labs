import numpy as np

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1
DEFAULT_GOAL_POINT = (1.0, 1.0)



class planner:
    def __init__(self, type_, traj_type="parabola"):

        self.type=type_
        self.traj_type = traj_type

    
<<<<<<< HEAD
    def plan(self, goalPoint=[1.0, 0]):
=======
    def plan(self, goalPoint=DEFAULT_GOAL_POINT):
>>>>>>> 8e96f1e (cleaned up formatting and argument parsing)
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        waypoints = []

        if self.traj_type == "parabola":
            # Parabola: y = x^2 for x in [0.0, 1.5]
            # 50 to 100 points provides smooth resolution
            x_vals = np.linspace(0.0, 1.5, num=75)
            y_vals = x_vals ** 2

        elif self.traj_type == "sigmoid":
            # Sigmoid: y = 2 / (1 + exp(-2*x)) - 1 for x in [0.0, 2.5]
            x_vals = np.linspace(0.0, 2.5, num=100)
            y_vals = (2.0 / (1.0 + np.exp(-2.0 * x_vals))) - 1.0

        else:
            raise ValueError(f"Unknown trajectory type: {self.traj_type}")

        for x, y in zip(x_vals, y_vals):
            waypoints.append([float(x), float(y)])

        return waypoints

