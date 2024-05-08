def save_traj(trajectories, filename):
    result = ""
    for trajectory in trajectories:
        result += ",".join([str(x) for x in trajectory])
        result += ";"

    f = open(filename, mode="w")
    f.write(result)
    f.close()