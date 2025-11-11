
from mapUtilities import *
from utilities import *
from numpy import cos, sin
import numpy as np


class particle:

    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def motion_model(self, v, w, dt):
        #DONE: Implement the motion model for the particle
        """
        v: linear velocity
        w: angular velocity
        dt: time step
        """
        theta = self.pose[2]
        self.pose[0] += (-v/w)*sin(theta) + (v/w)*sin(theta + w * dt) #x
        self.pose[1] += (+v/w)*cos(theta) - (v/w)*cos(theta + w * dt) #y
        self.pose[2] += w * dt #theta

    # DONE?: You need to explain the following function to TA
    def calculateParticleWeight(self, scanOutput: LaserScan, mapManipulatorInstance: mapManipulator, laser_to_ego_transformation: np.array):

        # Multiply the current orientation matrix with the laser_to_ego_transformation array
        # This will convert the results of the laser scan from the world frame to the robot frame
        T = np.matmul(self.__poseToTranslationMatrix(), laser_to_ego_transformation)

        # Convert the scan to cartesian coordinates, discard the R2 results, and take the homogenous (R3) ones
        # Then, use the dot product to project the homogenous results onto the corresponding vectors, to get laser travel distances
        _, scanCartesianHomo = convertScanToCartesian(scanOutput)
        scanInMap = np.dot(T, scanCartesianHomo.T).T

        # Get the likelihood field (current occupancy map)
        likelihoodField = mapManipulatorInstance.getLikelihoodField()
        # Convert the positions from scanInMap to cells
        cellPositions = mapManipulatorInstance.position_2_cell(
            scanInMap[:, 0:2])

        # Get the limits (map size) in the x- and y- direction
        lm_x, lm_y = likelihoodField.shape

        # AND the following four conditions:
            # Position X is positive
            # Position Y is positive (negate it first)
            # Position X is less than the y-limit
            # Position Y is less less than the x-limit (negate it first)
        # ie. if any of the above conditions are NOT met, then set the cell to 0; if all are met, set it to 1
        cellPositions = cellPositions[np.logical_and.reduce(
                (cellPositions[:, 0] > 0, -cellPositions[:, 1] > 0, cellPositions[:, 0] < lm_y,  -cellPositions[:, 1] < lm_x))]

        # Take the logarithm of the cell position values, sum them, 
        # then take the exponential to "undo" the logarithm and get our final weight
        log_weights = np.log(
            likelihoodField[-cellPositions[:, 1], cellPositions[:, 0]])
        log_weight = np.sum(log_weights)
        weight = np.exp(log_weight)
        weight += 1e-10 # Add an epsilon value to the weight so that it is never exactly zero

        # Set the current pose's weight to the calculated value
        self.setWeight(weight)

    def setWeight(self, weight):
        self.weight = weight

    def getWeight(self):
        return self.weight

    def setPose(self, pose):
        self.pose = pose

    def getPose(self):
        return self.pose[0], self.pose[1], self.pose[2]

    def __poseToTranslationMatrix(self):
        x, y, th = self.getPose()

        translation = np.array([[cos(th), -sin(th), x],
                                [sin(th), cos(th), y],
                                [0, 0, 1]])

        return translation
