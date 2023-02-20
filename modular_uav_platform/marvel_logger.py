"""
Author: Chi Chu
Date: 1/10/2023

This script maintains the log class of the modular uav swarm.
Class Logger: 

"""

import os
import csv
import time
import numpy as np
import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('Agg')

class Logger():
    def __init__(self, folder_name='./logs'):
        self.folder_name = folder_name
        self.log_memory = [["timestamp", "dt",
                            "pos_x", "pos_y", "pos_z",
                            "vel_x", "vel_y", "vel_z",
							"roll", "pitch", "yaw",
							"agv_x", "agv_y", "agv_z",
							"x_ref", "y_ref", "z_ref",
							"roll_ref", "pitch_ref", "yaw_ref",
							"x_vel_ref", "y_vel_ref", "z_vel_ref",
							"agv_x_ref", "agv_y_ref", "agv_z_ref"]]

    def log_append(self, timestamp, dt, 
                        pos, vel, rpy, agv, 
                        pos_ref, rpy_ref, vel_ref, agv_ref):
        self.log_memory.append([timestamp, dt,
                                pos[0], pos[1], pos[2],
                                vel[0], vel[1], vel[2],
                                rpy[0], rpy[1], rpy[2],
                                agv[0], agv[1], agv[2],
                                pos_ref[0], pos_ref[1], pos_ref[2], 
								rpy_ref[0], rpy_ref[1], rpy_ref[2], 
								vel_ref[0], vel_ref[1], vel_ref[2],
								agv_ref[0], agv_ref[1], agv_ref[2]])

    def openCSVMatrix(self, filename = 'filename'):
        rows = []
        with open(filename) as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                newrow = []
                for cell in row:
                    newcell = self.conv(cell)
                    newrow.append(newcell)
                rows.append(newrow)
        M = np.mat(rows)
        return M

    def savelog(self):
        try:
			# Create target Directory
            os.mkdir(self.folder_name)
            print("Directory \"" + self.folder_name +  "\" Created ") 
        except:
			# print("Directory " + self.folder_name +  " already exists")
            pass
		
        with open(self.folder_name + '/log_' + time.strftime("%m%d_%H%M%S") + '.txt', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.log_memory)
            print("CSV file: " + "log_" + time.strftime("%m%d_%H%M%S") + ".txt" + " created")
    
    def plot(self):
        n = len(self.log_memory)
        log_memory_array = np.asarray(self.log_memory[1:n])
        timestamp = log_memory_array[:, 0] - log_memory_array[0, 0]
        i = 2
        pos_x = log_memory_array[:, i]; i+=1
        pos_y = log_memory_array[:, i]; i+=1
        pos_z = log_memory_array[:, i]; i+=1
        vel_x = log_memory_array[:, i]; i+=1
        vel_y = log_memory_array[:, i]; i+=1
        vel_z = log_memory_array[:, i]; i+=1
        roll = log_memory_array[:, i]; i+=1
        pitch = log_memory_array[:, i]; i+=1
        yaw = log_memory_array[:, i]; i+=1
        agv_x = log_memory_array[:, i]; i+=1
        agv_y = log_memory_array[:, i]; i+=1
        agv_z = log_memory_array[:, i]; i+=1
        x_ref = log_memory_array[:, i]; i+=1
        y_ref = log_memory_array[:, i]; i+=1
        z_ref = log_memory_array[:, i]; i+=1
        roll_ref = log_memory_array[:, i]; i+=1
        pitch_ref = log_memory_array[:, i]; i+=1
        yaw_ref = log_memory_array[:, i]; i+=1
        x_vel_ref = log_memory_array[:, i]; i+=1
        y_vel_ref = log_memory_array[:, i]; i+=1
        z_vel_ref = log_memory_array[:, i]; i+=1
        agv_x_ref = log_memory_array[:, i]; i+=1
        agv_y_ref = log_memory_array[:, i]; i+=1
        agv_z_ref = log_memory_array[:, i]; i+=1
        print('i=%s ' %i)

        plt.subplot(2,2,1)
        plt.plot(timestamp, vel_x, 'r', timestamp, vel_y, 'g', timestamp, vel_z, 'b', timestamp, x_vel_ref, 'y--', timestamp, y_vel_ref, 'm--', timestamp, z_vel_ref, 'k--')
        plt.ylabel('vel')
        plt.grid(True)
        plt.subplot(2,2,2)
        plt.plot(timestamp, agv_x, 'r', timestamp, agv_y, 'g', timestamp, agv_z, 'b', timestamp, agv_x_ref, 'y--', timestamp, agv_y_ref, 'm--', timestamp, agv_z_ref, 'k--')
        plt.ylabel('agv')
        plt.grid(True)
        plt.subplot(2,2,3)
        plt.plot(timestamp, pos_x, 'r', timestamp, pos_y, 'g', timestamp, pos_z, 'b', timestamp, x_ref, 'y--', timestamp, y_ref, 'm--', timestamp, z_ref, 'k--')
        plt.ylabel('pos')
        plt.grid(True)
        plt.subplot(2,2,4)
        plt.plot(timestamp, roll, 'r', timestamp, pitch, 'g', timestamp, yaw, 'b', timestamp, roll_ref, 'y--', timestamp, pitch_ref, 'm--', timestamp, yaw_ref, 'k--')
        plt.ylabel('rpy')
        plt.grid(True)
		
        plt.show()