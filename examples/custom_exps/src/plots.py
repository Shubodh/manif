import numpy as np
import matplotlib.pyplot as plt


# plot all trajectories using matplotlib
def draw_traj_all(poses_opt, poses_gt, poses_init):
    # fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(25, 6))
    assert poses_opt.shape == poses_gt.shape == poses_init.shape, "array dimensions mismatch"
    ax = plt.subplot(111)

    # Plot Ground truth
    ax.plot(poses_gt[:, 0], poses_gt[:, 1], 'c-', label='Ground Truth')
    for i in range(0, len(poses_gt[:, 2]), 5):
        x2 = 0.25*np.cos(poses_gt[i, 2]) + poses_gt[i, 0]
        y2 = 0.25*np.sin(poses_gt[i, 2]) + poses_gt[i, 1]
        ax.plot([poses_gt[i, 0], x2], [poses_gt[i, 1], y2], 'c>')

    # Plot Initial estimate
    ax.plot(poses_init[:, 0], poses_init[:, 1], 'g-', label='Initial Estimate')
    for i in range(0, len(poses_init[:, 2]), 5):
        x2 = 0.25*np.cos(poses_init[i, 2]) + poses_init[i, 0]
        y2 = 0.25*np.sin(poses_init[i, 2]) + poses_init[i, 1]
        ax.plot([poses_init[i, 0], x2], [poses_init[i, 1], y2], 'g>')

    # Plot optimised trajectory
    ax.plot(poses_opt[:, 0], poses_opt[:, 1],
            'r-', label='Optimised Trajectory')
    for i in range(0, len(poses_opt[:, 2]), 5):
        x2 = 0.25*np.cos(poses_opt[i, 2]) + poses_opt[i, 0]
        y2 = 0.25*np.sin(poses_opt[i, 2]) + poses_opt[i, 1]
        ax.plot([poses_opt[i, 0], x2], [poses_opt[i, 1], y2], 'r>')

    plt.legend()
    plt.show()


def read_vertex(fileName):
    f = open(fileName, 'r')
    A = f.readlines()
    f.close()

    x_arr = []
    y_arr = []
    theta_arr = []

    for line in A:
        if "VERTEX_SE2" in line:
            (ver, ind, x, y, theta) = line.split()
            x_arr.append(float(x))
            y_arr.append(float(y))
            theta_arr.append(float(theta.rstrip('\n')))

    return np.array([x_arr, y_arr, theta_arr])


if __name__ == "__main__":
    gt_file_path = "gt.txt"
    init_file_path = "init.txt"
    opt_file_path = "opt.txt"
    poses_gt = read_vertex(gt_file_path).T
    poses_init = read_vertex(init_file_path).T
    poses_opt = read_vertex(opt_file_path).T
    draw_traj_all(poses_opt, poses_gt, poses_init)
