# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

from math import e


class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
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
        mode = "parabola"
        # Need to figure out how to choose one to return...
        if mode == "parabola":
            ## For parabola:
            x = [el / 100 for el in range(0, 150 + 5, 5)]
            y = []
            for el in x:
                y.append(round( el**2 , 4))
            points = list(zip(x, y))

        else:
            # For sigmoid:
            x = [el / 100 for el in range(0, 250 + 5, 5)]
            y = []
            for el in x:
                y.append(round( (2 / (1 + e ** (-2 * x))) - 1 , 4)) 
            points = list(zip(x, y))
        
        return points

