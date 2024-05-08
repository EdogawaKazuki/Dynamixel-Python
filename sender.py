import time
import socket


def geneerate_trajectory():
    # your code here!!
    # use your code in Task 1 adn 2  to generate the trajectory
    joint_0_list = []
    joint_1_list = []
    joint_2_list = []
    return [joint_0_list, joint_1_list, joint_2_list,]


# You can also use this function to directly read the trajectory from a file
def read_trajectory_file(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
        joint_trajectory_str = content.split(';')
        if joint_trajectory_str[0] == 'task_space' or joint_trajectory_str[0] == "joint_space":
            joint_trajectory_str = joint_trajectory_str[1:]
        joint_trajectory = [[float(x) for x in traj.split(',') if len(x) > 0]
                            for traj in joint_trajectory_str if len(traj) > 0]
        return joint_trajectory


cuter_ip = "127.0.0.1"
cuter_port = 9998
freq = 50
joint_number = 3
client_addr = (cuter_ip, cuter_port)
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# trajectory = geneerate_trajectory()
trajectory = read_trajectory_file('traj.txt')

# send trajectory
for i in range(len(trajectory[0])):
    # print(f"Sending trajectory {i}...")
    servo_angle_list = [trajectory[x][i] for x in range(joint_number)]
    send_data = []
    for angle in servo_angle_list[:joint_number]:
        send_data .append(str(angle))
    udp_socket.sendto(",".join(send_data).encode("utf-8"), client_addr)
    time.sleep(1 / freq)

udp_socket.close()
print("Finished. Exiting...")


