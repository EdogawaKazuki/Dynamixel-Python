import matplotlib.pyplot as plt

def read_traj(filename):
    with open(filename, 'r') as file:
        content = file.read()
        content = content.split(';')[1:-1]
        for i in range(len(content)):
            content[i] = content[i].split(',')
            for j in range(len(content[i])):
                content[i][j] = float(content[i][j])
        return content


traj_file_path = "trajectory_joint_space_with_color.txt"

trajs = read_traj(traj_file_path)
joint_trajs = trajs[:3]
color_trajs = trajs[3:]

plt.figure(figsize=(10, 15))

for i in range(3):
    plt.subplot(3, 1, i + 1)
    index_list = [i for i in range(len(joint_trajs[i]))]
    plt.plot(index_list, joint_trajs[i])
    plt.xlabel('Time')
    plt.ylabel('Joint Angle')
    plt.title(f'Joint {i} Trajectory')
# plt.tight_layout()
plt.show()






