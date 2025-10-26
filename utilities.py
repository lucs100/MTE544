from math import atan2, acos, sqrt
from geometry_msgs.msg import Quaternion

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename
        with open(self.filename, 'w') as file:
            header_str = ", ".join(headers) + "\n"
            file.write(header_str)

    def log_values(self, values_list):
        with open(self.filename, 'a') as file:
            # DONE Part 5: Write the values from the list to the file
            vals_str = " " + ", ".join(str(x) for x in values_list) + "\n"
            file.write(vals_str)

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        self.filename = filename
    
    def read_file(self):
        read_headers=False
        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line
            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table

# DONE Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    assert isinstance(quat, Quaternion), f"You must pass a Quaternion. Got {type(quat)}."
    #x, y, z, w = quat

    # assume phi = [ 0 0 1 ]', so x = 0, y = 0, z = sin(yaw/2), w = cos(yaw/2)  
    # yaw = acos(quat.w)*2
    yaw = acos(quat.w)*2
    # experimentally, it looks like the phi direction swaps every full revolution?
    # we need to check the sign of z - if z is negative, then "invert" the yaw
    if quat.z < 0:
        yaw = 2*M_PI - yaw

    # print(f"Yaw: {yaw:.2f} rad / {(yaw/M_PI):.2f}pi rad / {(yaw*180/M_PI):.2f} deg \t [{quat.x:.2f}, {quat.y:.2f}, {quat.z:.2f}, {quat.w:.2f}]")
    return yaw

    # experimental code archived below for posterity; not useful here but shown as proof

    # odom_orientation = {'x': msg.pose.pose.orientation.x,
    #                     'y': msg.pose.pose.orientation.y, 
    #                     'z': msg.pose.pose.orientation.z, 
    #                     'w': msg.pose.pose.orientation.w}
    # q = ['dummy', odom_orientation['x'], odom_orientation['y'], odom_orientation['z'], odom_orientation['w']]
    # Ra = [+q[1]**2-q[2]**2-q[3]**2+q[4]**2,   2*(q[1]*q[2] - q[4]*q[3]),    2*(q[2]*q[4] + q[1]*q[3])]
    # Rb = [2*(q[4]*q[3] - q[1]*q[2]),   -q[1]**2+q[2]**2-q[3]**2+q[4]**2,    2*(q[2]*q[3] - q[1]*q[4])]
    # Rc = [2*(q[1]*q[3] - q[4]*q[2]),   2*(q[4]*q[1] + q[2]*q[3]),    -q[1]**2-q[2]**2+q[3]**2+q[4]**2]

    # print("\nRotation matrix:")
    # print("\n".join([', '.join([f'{x:.2f}' for x in r]) for r in [Ra, Rb, Rc]]))

    # angle = acos(q[1]**2-q[2]**2-q[3]**2+q[4]**2)
    # print(f"\t-> Rotation angle: {angle:.2f} [{(angle/pi):.2f}pi]\n")

    # odom_x_pos = msg.pose.pose.position.x
    # odom_y_pos = msg.pose.pose.position.y
    # angle_z = 2*asin(odom_orientation['z'])
    # angle_w = 2*acos(odom_orientation['w'])

    # if odom_orientation['z'] < 0:
    #     true_angle = 2*pi - angle_w
    # else:
    #     true_angle = angle_w


#DONE Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose: list, goal_pose: list):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    error_linear = (
                        (current_pose[0] - goal_pose[0])**2 +
                        (current_pose[1] - goal_pose[1])**2
                    )**0.5

    return error_linear

#DONE Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the *angular error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal

    bestHeading = atan2((goal_pose[1]-current_pose[1]), (goal_pose[0]-current_pose[0]))
    currentHeading = current_pose[2]
    error_angular = bestHeading - currentHeading

    # Remember to handle the cases where the angular error might exceed the range [-π, π]
    # If outside the range, then add or subtract 2pi to "wrap around"
    if error_angular > M_PI:
        error_angular -= 2*M_PI
    elif error_angular < -M_PI:
        error_angular += 2*M_PI

    # Clamp the error: 
    error_angular = min(M_PI, error_angular)
    error_angular = max(-M_PI, error_angular)
    
    # Print some debug info
    print(f"\nXY: Current: ({(current_pose[0]):.2f}, {(current_pose[1]):.2f}) Goal: ({(goal_pose[0]):.2f}, {(goal_pose[1]):.2f}): "
          f"To goal: ({(goal_pose[0]-current_pose[0]):.2f}, {(goal_pose[1]-current_pose[1]):.2f}) Error: {calculate_linear_error(current_pose, goal_pose):.2f}\n"
          f"Theta: To goal: {bestHeading:.2f} "
          f"Current: {currentHeading:.2f} "
          f"Error: {error_angular:.2f} ")
    
    return error_angular
