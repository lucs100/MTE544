import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry as Odom #capital since it's a class

from rclpy import init, spin

SIM_MODE = False

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # DONE Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        if SIM_MODE:
            odom_qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        else:
            odom_qos = QoSProfile(
                depth=50,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # DONE Part 3: subscribe to the position sensor topic (Odometry)
            self.odom_sub = self.create_subscription(Odom, "/odom", self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg: Odom):
        
        # DONE Part 3: Read x,y, theta, and record the stamp
        poseX = pose_msg.pose.pose.position.x
        poseY = pose_msg.pose.pose.position.y
        poseTheta = euler_from_quaternion(pose_msg.pose.pose.orientation)
        poseStamp = Time.from_msg(pose_msg).nanoseconds
        self.pose=[poseX, poseY, poseTheta, poseStamp]
        
        # Log the data
        self.loc_logger.log_values(self.pose)
    
    def getPose(self):
        return self.pose

# DONE Part 3:
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py

def main(args=None):

    init(args=args)

    my_localization_node = localization()

    spin(my_localization_node)

    # No need to destroy the node

if __name__ == "__main__":
    main()