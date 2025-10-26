# Type of planner
POINT_PLANNER=0; PARABOLA_PLANNER=1; SIGMOID_PLANNER=2 

from math import e as M_E


class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self):
        # Return the planner corresponding with the selected motion type
        if self.type==POINT_PLANNER:
            return self.point_planner()
        
        elif self.type==PARABOLA_PLANNER:
            return self.parabola_planner()

        elif self.type==SIGMOID_PLANNER:
            return self.sigmoid_planner()


    def point_planner(self, goalPoint=[0, 0]):
        # A single point; should still be a nested list
        x = goalPoint[0]
        y = goalPoint[1]
        target = [x, y]
        return [target]

    # DONE Part 6: Implement the trajectories here
    def parabola_planner(self):
        # Calculates a parabola by creating a list of x,y pairs 
        x = [el / 100 for el in range(0, 150 + 5, 5)]
        points = []
        for _x in x: #_x is the current element
            y = _x**2
            points.append([_x, round(y, 4)])
            
        return points

    def sigmoid_planner(self):
        # Calculates a sigmoid by creating a list of x,y pairs 
        x = [el / 100 for el in range(0, 250 + 5, 5)]
        points = []
        for _x in x:
            y = (2 / (1 + M_E ** (-2 * _x))) - 1
            points.append([_x, round(  y, 4)]) 
        
        return points

