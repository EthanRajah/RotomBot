#!/usr/bin/env python3
import matplotlib.pyplot as plt
# import numpy as np
import csv

def plot(csv_file):
    '''Function to read csv file containing (t, x_rs, y_rs, z_rs, q_x_rs, q_y_rs, q_z_rs, q_w_rs, x_v, y_v, z_v, q_x_v, q_y_v, q_z_v, q_w_v) coordinates and plot z vs time on one plot'''
    # timestamp,realsense_pos_x,realsense_pos_y,realsense_pos_z,realsense_orient_x,realsense_orient_y,realsense_orient_z,realsense_orient_w,vicon_pos_x,vicon_pos_y,vicon_pos_z,vicon_orient_x,vicon_orient_y,vicon_orient_z,vicon_orient_w

    # Read csv files
    csv_data = csv.reader(open(csv_file, 'r'), delimiter=',')
    # Skip header row 
    next(csv_data)
    # Initialize lists to store time and z coordinates
    time = []
    vicon_z = []
    realsense_z = []
    # Read data from each file
    for row in csv_data:
        time.append(float(row[0]))
        vicon_z.append(float(row[3]))
        realsense_z.append(float(row[10]))
    # Plot data
    # plt.plot(time, vicon_z, label='Vicon')
    # plt.plot(time, realsense_z, label='Realsense')
    plt.plot(realsense_z, vicon_z, label = 'Realsense vs Vicon' )
    plt.xlabel('Realsense')
    plt.ylabel('Vicon')
    plt.title('Vicon vs Realsense')
    plt.legend()
    plt.savefig('Vicon_Vs_Realsense.png')
    print("Done Plotting")

if __name__ == '__main__':
    print('Plotting data for realsense and vicon...')
    plot('combined_data.csv')   