"""
Author: Chi Chu
Date: 2/21/2023

This is keyboard control class of the modular uav platform swarm, which is used as the 
input interactive interface.
Class KeyboardInput: Send command by keyboard, include take off and stop.

"""

import time
import tkinter as tk

import numpy as np

class KeyboardInput():
	"""
		Interactive input interface
	"""
	def __init__(self, control_mode='position'):
		self.command = tk.Tk()
		self.command.bind_all("<Key>", self.key_input)
		self.control_mode = control_mode
		print("------Keyboard Is Ready------")

		self.pos = [0.0,0.0,1.0] # x y z
		self.vel = [0.0,0.0,0.0] # velocity
		self.rpy = [0.0,0.0,0.0] # roll pitch yaw
		self.agv = [0.0,0.0,0.0] # angular velocity

		self.cmd = [0.0,0.0,0.2,0.0] # x y z, yaw used for single marvel

		#Maybe no use for marvel
		self.angle_step = 0.1
		self.pos_step = 0.1
		self.x_step, self.y_step, self.z_step = 0.1, 0.1, 0.1

		self.stop = 0
		self.switch_mode = 0
		self.take_off = 0

	def key_input(self, event):
		key_press = event.keysym.lower()
		print(key_press)
		# Add command here
		if key_press == 'space':
			self.take_off = 1
		elif key_press == 'return':
			if self.switch_mode == 0:
				self.switch_mode = 1
			elif self.switch_mode == 1:
				self.switch_mode == 0
		elif key_press == 'escape':
			self.stop = 1
		elif key_press == 'w':
			self.pos[0] += self.x_step
		elif key_press == 'a':
			self.pos[1] += self.y_step
		elif key_press == 's':
			self.pos[0] -= self.x_step
		elif key_press == 'd':
			self.pos[1] -= self.y_step
		elif key_press == 'q':
			self.pos[2] += self.z_step
		elif key_press == 'e':
			self.pos[2] -= self.z_step
		
if __name__ == '__main__':
	keyboardcontrol = KeyboardInput()
	while keyboardcontrol.stop == False:
		keyboardcontrol.command.update()
		time.sleep(0.01)
