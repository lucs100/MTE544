# Type of planner
POINT_PLANNER=0; PARABOLA_PLANNER=1; SIGMOID_PLANNER=2 

from math import e


class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==PARABOLA_PLANNER:
            return self.parabola_planner()

        elif self.type==SIGMOID_PLANNER:
            return self.sigmoid_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return (x, y)

    # DONE Part 6: Implement the trajectories here
    def parabola_planner(self):
        x = [el / 100 for el in range(0, 150 + 5, 5)]
        points = []
        for el in x:
            points.append((el, round( el**2 , 4)))
            
        return points

    def sigmoid_planner(self):
        x = [el / 100 for el in range(0, 250 + 5, 5)]
        points = []
        for el in x:
            points.append((el, round( (2 / (1 + e ** (-2 * x))) - 1 , 4))) 
        
        return points

