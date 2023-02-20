"""
Author: Chi Chu
Last Update Date: 2/16/2023

This is the main script of the modular uav platform swarm.
Class Swarm: The class of marvel swarms.
Class Master: Main class, consist of keyboard, logger and swarm, etc.
Function main:  Obviously, the main function :D.

This project uses multiprocessing, 3 processes in total.
Process 1(main): collect vicon data, keyboard input, manage them.
Process 2: send command to marvel swarm, collect marvel data, and log marvel data+vicon data.
Process 3: high level controller

User inputs:
	refer to gimbal_platform_keyboard.py for keyboard control
	Press ESC for emergency stop

"""

import time
import logging
import multiprocessing as mp
import cflib.crtp
import config  

# from vicon import Vicon
from marvel import Marvel
from marvel_swarm import Swarm
from marvel_keyboard import KeyboardInput
from marvel_logger import Logger

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class Master():
    def __init__(self, num=config.MARVEL_NUM, mode=True, control_mode='position'):
        self.start_time = 0
        self.update_time = 0.005    # 200Hz
        self.marvel_num = num
        self.mode = mode # True refers to swarm, false refers to combinedPlatform
        self.control_mode = control_mode

        self._init_var()
        self._init_keyboard()
        self._init_highlevel_controller()
        self._init_swarm()
        self._init_vicon()
        self._init_logger()
        time.sleep(0.25)
    
    def run(self):
        self.start_time = time.time()
        self.last_loop_time = self.start_time
        print("Start time:", self.start_time)
        while 1:
            if self.keyboard.stop == 1:
                print("Program has been safely stopped.")
                break
            current_time = time.time()
            if current_time - self.last_loop_time > self.update_time:
                self._share_vicon_data()
                # Update keyboard values
                self.keyboard.command.update()
                self._share_command()
                self._record(current_time)
                self.last_loop_time = current_time
            else:
                time.sleep(0.0001)
        self._stop()
    
    def _init_var(self):
        # Reference variables, get from keyboard
        self.pos_ref_shared, self.rpy_ref_shared = mp.Array('f',3), mp.Array('f',3)
        self.vel_ref_shared, self.agv_ref_shared = mp.Array('f',3), mp.Array('f',3)
        self.cmd_ref_shared = mp.Array('f',4)
        # These command value get from keyboard
        self.take_off_shared = mp.Value('i', 0)
        self.stop_shared = mp.Value('i', 0)
        # Ground truth varaibles, get from vicon
        self.pos_shared, self.vel_shared = mp.Array('f',3), mp.Array('f',3)
        self.quat_shared, self.omega_shared = mp.Array('f',4), mp.Array('f',3)
        self.quat_shared[:] = [1, 0, 0, 0]

    def _init_keyboard(self):
        # Initialize keyboard 
        self.keyboard = KeyboardInput(control_mode='position')

    def _init_highlevel_controller(self):
        # Used only for combinedPlatform 
        #self.controller = Controller()
        #self.p_control = mp.Process()
        pass

    def _init_swarm(self):
        self.swarm = Swarm(num=self.marvel_num, mode=self.mode, control_mode=self.control_mode)
        print("------First Step Initialization Completed------")
        self.p_swarm = mp.Process(target=self.swarm.run, args=(self.pos_ref_shared, self.rpy_ref_shared, 
                                                                self.vel_ref_shared, self.agv_ref_shared,
                                                                self.cmd_ref_shared,
                                                                self.pos_shared, self.vel_shared,
                                                                self.quat_shared, self.omega_shared,
                                                                self.take_off_shared, self.stop_shared))

    def _init_vicon(self):
        # self.vicon = Vicon()
        pass

    def _init_logger(self):
        self.logger = Logger(folder_name='./logs')

    def _share_vicon_data(self):
        pass
        # Share vicon value
        # self.pos_shared[:] = self.vicon.position
        # self.vel_shared[:] = self.vicon.velocity
        # self.quat_shared[:] = self.vicon.rotation
        # self.omega_shared[:] = self.vicon.rotation_rate

    def _share_command(self):
        # Share command values from keyboard
        for i in range(self.marvel_num):
            if self.mode == True:
                self.cmd_ref_shared[:] = self.keyboard.cmd
            elif self.mode == False:
                self.pos_ref_shared[:] = self.keyboard.pos
                self.rpy_ref_shared[:] = self.keyboard.rpy
                self.vel_ref_shared[:] = self.keyboard.vel
                self.agv_ref_shared[:] = self.keyboard.agv
        self.take_off_shared.value = self.keyboard.take_off
        self.stop_shared.value = self.keyboard.stop
    
    def _record(self, current_time):
        if self.swarm.mode == False:
            self.logger.log_append(int(round((current_time-self.start_time) * 1000)), int(round((current_time-self.last_loop_time) * 1000)),
                                    self.pos_shared[:], self.vel_shared[:], self.quat_shared[:], self.omega_shared[:],
                                    self.pos_ref_shared[:], self.rpy_ref_shared[:],
                                    self.vel_ref_shared[:], self.agv_ref_shared[:])

    def _stop(self):
        self.keyboard.command.quit()
        self.keyboard.command.destroy()
        # self.logger.savelog()
        # self.logger.plot()

        # if self.swarm.mode == 0:
        #     self.swarm.logger.savelog()
        #     self.logger.plot()
        # elif self.swarm.mode == 1:
        #     self.logger.savelog()
        #     self.logger.plot()
        time.sleep(0.1)

def main():
    master = Master()
    master.p_swarm.start()
    sleepTime = 6
    for i in range(sleepTime):
        print("Wait for Launch......:{}".format(sleepTime-i))
        time.sleep(1)
    # print("Verify handle num:{}".format(len(master.swarm.marvel_swarm_handle)))    
    master.run()

    master.p_swarm.join()

if __name__ == '__main__':
    main()
