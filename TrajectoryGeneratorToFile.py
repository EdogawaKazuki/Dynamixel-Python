import time
import socket

# No need to change this function
def udp_client(server_ip, server_port, message):
    # Create a UDP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Send the message to the server
        client_socket.sendto(message.encode(), (server_ip, server_port))
        print(f"Command sent to {server_ip}:{server_port}, command: {message}")

    except Exception as e:
        print(f"An error occurred: {e}")
        finally:
    # Close the socket
    client_socket.close()

# No need to change this function
def send_command(server_ip, server_port, angle_list, color_list):
    '''
    this function is used to send the command to the robot. If you are using the simulator, you need to set the robot to Server Mode in the setting page.
    angle_list: list of angles
    color_list: list of colors
    '''
    message = ",".join([str(i) for i in angle_list])
    message += ","
    message += ",".join([str(i) for i in color_list])
    udp_client(server_ip, server_port, message)

# No need to change this function
def concatenate_trajectories(traj_list_1, traj_list_2):
    '''
    this function is used to concatenate the trajectories
    traj_list_1: list of list, each list is a trajectory
    traj_list_2: list of list, each list is a trajectory
    '''
    assert len(traj_list_1) == len(traj_list_2)
    traj_list = [[] for _ in range(3)]
    for i in range(len(traj_list_1)):
        traj_list[i] = traj_list_1[i] + traj_list_2[i]
    return traj_list

# No need to change this function
def trajectory_to_file(traj_list, color_list, prefix="angle"):
    '''
    this function is used to save the trajectory to a file
    traj_list: list of list, each list is a trajectory
    color_list: list of colors
    prefix: prefix of the file name
    '''
    result = prefix + ";"
    for point in traj_list:
        result += ",".join([str(i) for i in point])
        result += ";"
    for color in color_list:
        result += ",".join([str(i) for i in color])
        result += ";"
    f = open("trajectory_joint_space_with_color.txt", mode="w")
    f.write(result)
    f.close()

# Example 1: Linear Trajectory
def linear_trajectory(start_pos, end_pos, end_time, frequency=50):
    '''
    this function is used to generate a linear trajectory
    start_pos: list of start position (or angles)
    end_pos: list of end position (or angles)
    end_time: time spend of the trajectory (in seconds)
    frequency: frequency of the trajectory
    '''
    # Get the trajectory
    # Define the result
    result = [[], [], []]

    # List of time for each frame
    time_list = []
    for i in range(end_time * frequency):
        time_list.append(i / frequency)

    # Repeat the process for each joint
    for i in range(3):
        # Get the coefficients
        a0 = start_pos[i]
        a1 = (end_pos[i] - start_pos[i]) / end_time

        # Generate the trajectory
        for t in time_list:
            # linear interpolation
            result[i].append(a0 + a1 * t)
    return result

# Task 1: Cubic Trajectory for Two Points
def cubic_trajectory_for_two_points(start_pos, end_pos, end_time, frequency=50):
'''
this function is used to generate a cubic trajectory
start_pos: list of start position 
end_pos: list of end position 
end_time: time spend of the trajectory (in seconds)
frequency: frequency of the trajectory
'''

# Get the coefficients
a0 = <fill_in_your_code>
a1 = <fill_in_your_code>
a2 = <fill_in_your_code>
a3 = <fill_in_your_code>

# Get the trajectory
# Define the result
result = [[], [], []]

# List of time for each frame
time_list = []
for i in range(end_time * frequency):
time_list.append(i / frequency)

# Generate the trajectory
for t in time_list:
for i in range(3):
# cubic interpolation
result[i].append(<fill_in_your_code>)
return result

# Task 2: Cubic Trajectory with Mid Point
def cubic_trajectory_with_mid_point(start_pos, mid_pos, end_pos, mid_velocity, mid_time, end_time, frequency=50):
'''
this function is used to generate a cubic trajectory
start_pos: list of start position 
mid_pos: list of mid position 
end_pos: list of end position 
mid_velocity: velocity of the mid point 
time_spend: time spend of the trajectory (in seconds)
frequency: frequency of the trajectory
'''
# Define the result
result = [[], [], []]

# First part
# Get the coefficients for the first part
a0 = <fill_in_your_code>
a1 = <fill_in_your_code>
a2 = <fill_in_your_code>
a3 = <fill_in_your_code>

# List of time for each frame for the first part
time_list = []
for i in range(mid_time * frequency):
time_list.append(i / frequency)

# Generate the trajectory for the first part
for t in time_list:
for i in range(3):
result[i].append(<fill_in_your_code>)

# Second part
# Get the coefficients for the second part
a0 = <fill_in_your_code>
a1 = <fill_in_your_code>
a2 = <fill_in_your_code>
a3 = <fill_in_your_code>

# List of time for each frame for the second part
time_list = []
for i in range(end_time * frequency):
time_list.append(i / frequency)

# Generate the trajectory for the second part
for t in time_list:
for i in range(3):
result[i].append(<fill_in_your_code>)

return result

# Task 3: Inverse Kinematics
def inverse_kinematics(task_space_pos):
'''
this function is used to generate the joint space position from the task space position
task_space_pos: task space position 
'''
# Get the joint space position
theta_1 = <fill_in_your_code>
theta_2 = <fill_in_your_code>
theta_3 = <fill_in_your_code>

return [theta_1, theta_2, theta_3]

# Example 2: Static Color List
def static_color_list(color, time_spend, frequency=50):
'''
this function is used to generate a list of static color
color: list of color
time_spend: time spend of the trajectory (in seconds)
frequency: frequency of the trajectory
'''
# Define the result
result = []

# List of time for each frame
time_list = []
for i in range(time_spend * frequency):
result.append(color)
return result

# Task 4(Optional): Generate the color trajectory
def generate_linear_color_list(start_color, end_color, end_time, frequency=50):
'''
this function is used to generate a list color that changes linearly from start_color to end_color
start_color: list of start color
end_color: list of end color
end_time: time spend of the trajectory (in seconds)
frequency: frequency of the trajectory
'''
# Define the result
result = [[], [], []]

# Repeat the process for each red, green, blue component
for i in range(3):
# Get the coefficients
a0 = <fill_in_your_code>
a1 = <fill_in_your_code>

# List of time for each frame
time_list = []
for i in range(end_time * frequency):
time_list.append(i / frequency)

# Generate the trajectory
for t in time_list:
result[i].append(<fill_in_your_code>)
return result


# Task 4: Generate the trajectories
if __name__ == "__main__":
# Set the frequency
frequency = 50
# Generate the task space trajectory
task_space_trajectory_1 = linear_trajectory([0, 0, 0], [90, 90, 90], 2, frequency)
task_space_trajectory_2 = linear_trajectory([90, 90, 90], [0, 0, 0], 2, frequency)

color_trajectory_1 = static_color_list([0, 0, 0], 2, frequency)
color_trajectory_2 = static_color_list([255, 255, 255], 2, frequency)

# Concatenate the two trajectories
task_space_trajectory = concatenate_trajectories(task_space_trajectory_1, task_space_trajectory_2)

color_trajectory = concatenate_trajectories(color_trajectory_1, color_trajectory_2)

# Generate the joint space trajectory
joint_space_trajectory = []
for i in range(len(task_space_trajectory)):
joint_space_trajectory.append(inverse_kinematics(task_space_trajectory[i]))

# Generate the trajectory file for simulator
trajectory_to_file(joint_space_trajectory, color_trajectory, "angle")

# No need to change the following code
# Send the command to the robot
# Please make sure the robot is in Server Mode in the setting page of the simulator
command_length = len(joint_space_trajectory[0])

server_ip = "localhost" 
server_port = 5000 # Replace with the server's port
for i in range(command_length):
angle_list = [joint_space_trajectory[0][i], joint_space_trajectory[1][i], joint_space_trajectory[2][i]]
color_list = [color_trajectory[0][i], color_trajectory[1][i], color_trajectory[2][i]]
send_command(server_ip, server_port, angle_list, color_list)
time.sleep(1/frequency)