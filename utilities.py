from math import atan2, asin, sqrt
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

    # assume phi = [ 0 0 1 ]', so x = 0, y = 0, z = cos(yaw/2), w = sin(yaw/2)  
    # yaw = asin(quat.w)*2
    a = 2 * (quat.w * quat.z + quat.x + quat.y)
    b = 1 - (2 * (quat.y ** 2 + quat.z ** 2))
    yaw = atan2(a, b)

    ... # just unpack yaw
    return yaw


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
    
    print(f"""Current coords: {(round(current_pose[0], 2), round(current_pose[1], 2))} 
          Goal vector: {(round(goal_pose[1]-current_pose[1], 2), round(goal_pose[1]-current_pose[1], 2))} 
          Best heading: {round(bestHeading, 2)} 
          Current heading: {round(currentHeading, 2)} 
          Error: {error_angular}""")
    
    return error_angular
