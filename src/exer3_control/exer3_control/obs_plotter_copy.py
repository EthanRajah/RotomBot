#!/usr/bin/env python3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# import numpy as np
import csv


import matplotlib.pyplot as plt
import numpy as np

def plot_vector_from_position(ax, position, vector, color='r', length=0.2, label=None):
    """
    Plot a 3D vector from a given position on the provided matplotlib 3D axis.

    Args:
        ax (Axes3D): A 3D axis object (e.g., from fig.add_subplot(..., projection='3d')).
        position (tuple or list): (x, y, z) starting point of the vector.
        vector (np.ndarray): 3D direction vector (will be normalized).
        color (str): Color of the arrow.
        length (float): Length to scale the vector to.
        label (str, optional): Label for the arrow (optional).
    """
    x, y, z = position
    v = vector * length  # normalize and scale vector

    ax.quiver(x, y, z, v[0], v[1], v[2], color=color)

    # if label:
    #     ax.text(x + v[0], y + v[1], z + v[2], label, color=color)


def quaternion_to_vector(x, y, z, w):
    """
    Convert a quaternion into a forward direction vector.
    
    Args:
        x, y, z, w (float): Quaternion components.

    Returns:
        np.ndarray: A 3D unit vector representing the forward direction.
    """
    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),           1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
    ])

    # Assuming "forward" is along the local z-axis
    forward_local = np.array([0, 0, 1])
    forward_world = R @ forward_local

    return forward_world / np.linalg.norm(forward_world)

def plot_circle(ax, center, radius, z_height, color = 'b'):

    theta = np.linspace(0, 2*np.pi, 100)
    x = center[0] + radius*np.cos(theta)
    y = center[1] + radius*np.sin(theta)
    z = np.full_like(x, z_height)

    ax.plot(x,y,z, color = color, markersize = 1)

def plot(csv_file):
    obstacles = np.array([[1.75, 1.75], [1.75, -1.75], [-1.75, 1.75], [-1.75, -1.75]])
    waypoints = np.array([[1.75, -2.211, 0.3], [-1.2, -1.661, 0.3]])
    '''Function to read csv file containing (t, x_rs, y_rs, z_rs, q_x_rs, q_y_rs, q_z_rs, q_w_rs, x_v, y_v, z_v, q_x_v, q_y_v, q_z_v, q_w_v) coordinates and plot z vs time on one plot'''
    # timestamp,realsense_pos_x,realsense_pos_y,realsense_pos_z,realsense_orient_x,realsense_orient_y,realsense_orient_z,realsense_orient_w,vicon_pos_x,vicon_pos_y,vicon_pos_z,vicon_orient_x,vicon_orient_y,vicon_orient_z,vicon_orient_w

    # Read csv files
    csv_data = csv.reader(open(csv_file, 'r'), delimiter=',')
    # Skip header row 
    next(csv_data)
    # Initialize lists to store time and z coordinates
    vicon_x = []
    vicon_y = []
    vicon_z = []
    
    vicon_or_x = []
    vicon_or_y = []
    vicon_or_z = []
    
    # Read data from each file
    for row in csv_data:
        vicon_x.append(float(row[1]))
        vicon_y.append(float(row[2]))
        vicon_z.append(float(row[3]))
        
        vec = quaternion_to_vector(row[4], row[5], row[6], row[7])
        vicon_or_x.append(vec[1])
        vicon_or_y.append(vec[2])
        vicon_or_z.append(vec[3])
        
    # Plot data
    # plt.plot(time, vicon_z, label='Vicon')
    # plt.plot(time, realsense_z, label='Realsense')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.set_box_aspect([1,1,1])
    
    for obs in obstacles:
        ax.plot([obs[0]]*100, [obs[1]]*100, np.linspace(0, 2, 100), marker='.', color='r') 
        plot_circle(ax, obs, 0.65, 0.3)
        plot_circle(ax, obs, 0.15, 0.3, color = 'y')

    ax.plot(vicon_x, vicon_y, vicon_z, marker='.',color= 'g', markersize = 1)
    # ax.plot(waypoints[:,0], waypoints[:,1], waypoints[:,2], marker='.',color= 'y', markersize = 1)
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")        
    plt.show()

    ax.set_title("3D Trajectory Plot")
    plt.savefig('Vicon_Traj.png')
    ax.view_init(elev=90, azim=-90)
    plt.savefig('Vicon_Traj_BEV.png')
    ax.view_init(elev=0, azim=90)
    plt.savefig('Vicon_Traj_xy_plane.png')
    ax.view_init(elev=0, azim=90)
    plt.savefig('Vicon_Traj_yz_plane.png')
    ax.view_init(elev=0, azim=0)
    print("Done Plotting")

if __name__ == '__main__':
    print('Plotting data for realsense and vicon...')
    plot('vicon_data_obs.csv')   