# Imports

import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import POINT_PLANNER, PARABOLA_PLANNER, SIGMOID_PLANNER, planner
from controller import controller, trajectoryController, SIM_MODE

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        #DONE Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher = self.create_publisher(
            msg_type=publisher_msg,
            topic=publishing_topic,
            qos_profile=qos_publisher
        )

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.5, klv=0.8, kap=0.5, kav=2)
            self.planner=planner(POINT_PLANNER)    
    
        elif motion_type==PARABOLA_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(PARABOLA_PLANNER)
    
        elif motion_type==SIGMOID_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(SIGMOID_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)

        self.goalIdx = 0

        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint removed for compatibility
        self.goal=self.planner.plan()

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # DONE Part 3: Run the localization node
        spin_once(self.localizer)   # Remember that this file is already running the decision_maker node.

        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        lin_error_thresh = 0.02
        # We only need to check the linear error - we don't really care about what direction the robot is facing at the goal

        # DONE Part 3: Check if you reached the goal
        if type(self.goal) == list:
            # Goal is a list of points, so let's check against the current point
            currentGoal = self.goal[self.goalIdx]
            print(f"Current goal [index #{self.goalIdx}]: {currentGoal}")
            err_linear = abs(calculate_linear_error(self.localizer.getPose(), currentGoal))
            
            # If we've reached the current goal point, then advance the goal index
            if err_linear <= lin_error_thresh:
                self.goalIdx += 1
            # If we've reached the final goal point, then set the reached_goal flag
            reached_goal = self.goalIdx >= len(self.goal)
        else: 
            # Goal is a single point, so let's check if we're there
            # Extract as currentGoal so we can reuse code below
            currentGoal = self.goal
            err_linear = abs(calculate_linear_error(self.localizer.getPose(), currentGoal))
            reached_goal = err_linear <= lin_error_thresh
        
        if reached_goal:
            # Complete, publish an empty Twist message and save the logs
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #DONE Part 3: exit the spin
            raise SystemExit
        
        # If we haven't reached the goal, then update our speeds using the PID controller
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), currentGoal, True)
        print(f"Setting velocity to {velocity} and yaw rate to {yaw_rate}")

        #DONE Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity #just forward velocity
        vel_msg.angular.z = yaw_rate #turn rate 
        self.publisher.publish(vel_msg)

import argparse

def main(args=None):
    
    init() #inits ros

    # DONE Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    if SIM_MODE:
        odom_qos=QoSProfile(reliability=1, durability=1, history=1, depth=10) #For simulation
    else:
        odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10) #For lab

    # DONE Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    # The only difference between the modes is the motionType argument so we can pull this out
    if args.motion.lower() == "point":
        motionType = POINT_PLANNER
    elif args.motion.lower() == "parabola":
        motionType = PARABOLA_PLANNER
    elif args.motion.lower() == "sigmoid":
        motionType = SIGMOID_PLANNER
    else:
        print("invalid motion type", file=sys.stderr) 
        exit()
    
    # Instantiate the decision maker using the chosen motion type
    DM = decision_maker(
        publisher_msg=Twist,
        publishing_topic="/cmd_vel",
        qos_publisher=odom_qos,
        rate=10,
        motion_type=motionType
    )
    
    try:
        # Run the decision maker
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point, parabola, or sigmoid") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
