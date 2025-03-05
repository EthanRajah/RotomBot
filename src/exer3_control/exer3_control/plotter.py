#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import csv

def plot(csv):
    '''Function to read csv file containing (t, x_rs, y_rs, z_rs, q_x_rs, q_y_rs, q_z_rs, q_w_rs, x_v, y_v, z_v, q_x_v, q_y_v, q_z_v, q_w_v) coordinates and plot z vs time on one plot'''
    # Read csv files
    csv_data = csv.reader(csv)
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
    plt.plot(time, vicon_z, label='Vicon')
    plt.plot(time, realsense_z, label='Realsense')
    plt.xlabel('Time (s)')
    plt.ylabel('Z (m)')
    plt.title('Z vs Time')
    plt.legend()
    plt.s

if __name__ == '__main__':
    print('Plotting data for realsense and vicon...')
    plot('combined_data.csv')