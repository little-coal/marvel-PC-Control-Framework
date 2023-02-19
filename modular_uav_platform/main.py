"""
Author: Chi Chu
Last Update Date: 2/16/2023

This is the main script of the modular uav platform swarm.
Class Swarm: The class of marvel swarms.
Class Master: Main class, consist of keyboard, logger and swarm
Function main:  Obviously, the main function :).

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
from marvel_keyboard import KeyboardInput
from marvel_logger import Logger

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class Swarm():
    def __init__(self):
        """
            Swarm has 2 mode: Swarm and CombinedPlatform
        """
        self.marvel_num = config.MARVEL_NUM
        self.marvel_swarm_handle = []
        self.mode = 0   #0 ref swarm, 1 ref combinedFrame
        self.control_mode = 'position'  #'position': x, y, z, yaw

        self.marvel_setup_time = 0.25
        self.update_time = 0.01 # 100Hz
        self.last_loop_time = time.time()
        self.current_time = self.last_loop_time
        self.dt = 0
        self.log = []

        self.logger = Logger(folder_name='./logs')

    def run(self, pos_ref_shared, rpy_ref_shared, vel_ref_shared, agv_ref_shared, cmd_ref_shared,
                pos_shared, vel_shared, quat_shared, omega_shared, take_off_shared, stop_shared):
        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers()
		# Scan for Crazyflies and initialize marvels
        for i in range(self.marvel_num):
            marvel = Marvel('radio://0/100/2M/E7E7E7E7E{}'.format(i), i)
            self.marvel_swarm_handle.append(marvel)
            # print("Find MARVEL num:{}".format(len(self.marvel_swarm_handle)))
        print("------MARVEL Connection Completed------")
        self.start_time = time.time()
        while 1:
            if stop_shared.value == 1:
                break
            self.current_time = time.time()
            # Transmit and record values
            if self.mode == 0:
                pos_ref_shared[:] = self.marvel_swarm_handle[i].target_pos_and_rot[0:3]
                rpy_ref_shared[:] = self.marvel_swarm_handle[i].target_pos_and_rot[3:6]
                vel_ref_shared[:] = self.marvel_swarm_handle[i].target_velocity[0:3]
                agv_ref_shared[:] = self.marvel_swarm_handle[i].target_velocity[3:6]
                self.logger.log_append(int(round((self.current_time-self.start_time) * 1000)), 
                                    int(round((self.current_time-self.last_loop_time) * 1000)),
                                            pos_shared[:], vel_shared[:], quat_shared[:], omega_shared[:],
                                            pos_ref_shared[:], rpy_ref_shared[:],
                                            vel_ref_shared[:], agv_ref_shared[:])
            if self.current_time - self.last_loop_time > self.update_time:
                if self.mode == 0:
                    if take_off_shared.value == 1:
                        for i in range(len(self.marvel_swarm_handle)):
                            self.marvel_swarm_handle[i].send_extpose(
                                    pos_shared[0], pos_shared[1], pos_shared[2], 
                                    quat_shared[1], quat_shared[2], quat_shared[3], quat_shared[0]) #x, y, z, w
                            if self.control_mode == 'position':
                                self.marvel_swarm_handle[i].send_position_setpoint( 
                                        cmd_ref_shared[0], cmd_ref_shared[1], cmd_ref_shared[2], cmd_ref_shared[3])
                self.dt = self.current_time - self.last_loop_time
                self.last_loop_time = self.current_time
                self.log.append(self.dt)
            else:
                time.sleep(0.001)
        self.stop()
    
    def stop(self):
        for i in range(len(self.marvel_swarm_handle)):
            self.marvel_swarm_handle[i]._stop_crazyflie()

class Master():
    def __init__(self):
        self.start_time = 0
        self.update_time = 0.005

        # Reference variables, get from keyboard or single marvel
        self.pos_ref_shared, self.rpy_ref_shared = mp.Array('f',3), mp.Array('f',3)
        self.vel_ref_shared, self.agv_ref_shared = mp.Array('f',3), mp.Array('f',3)
        self.cmd_ref_shared = mp.Array('f',4)
        # Ground truth varaibles, get from vicon
        self.pos_shared, self.vel_shared = mp.Array('f',3), mp.Array('f',3)
        self.quat_shared, self.omega_shared = mp.Array('f',4), mp.Array('f',3)
        self.quat_shared[:] = [1, 0, 0, 0]
        # These command value get from keyboard
        self.take_off_shared = mp.Value('i', 0)
        self.stop_shared = mp.Value('i', 0)
        # Initialize keyboard 
        self.keyboard = KeyboardInput(control_mode='position')
        self.logger = Logger(folder_name='./logs')
        #In this framework, there is no high-level controller now
        #self.controller = Controller()
        #self.p_control = mp.Process()
        self.swarm = Swarm()
        print("------First Step Initialization Completed------")
        self.p_swarm = mp.Process(target=self.swarm.run, args=(self.pos_ref_shared, self.rpy_ref_shared, 
                                                                self.vel_ref_shared, self.agv_ref_shared,
                                                                self.cmd_ref_shared,
                                                                self.pos_shared, self.vel_shared,
                                                                self.quat_shared, self.omega_shared,
                                                                self.take_off_shared, self.stop_shared))
        # self.vicon = Vicon()
        time.sleep(0.25)
    
    def run(self):
        self.start_time = time.time()
        self.last_loop_time = self.start_time
        print("Start time:", self.start_time)
        while 1:
            # print(self.pos_shared[:])
            if self.keyboard.stop == 1:
                print("Program has been safely stopped.")
                break
            current_time = time.time()
            if current_time - self.last_loop_time > self.update_time:
                # Share vicon value
                # self.pos_shared[:] = self.vicon.position
                # self.vel_shared[:] = self.vicon.velocity
                # self.quat_shared[:] = self.vicon.rotation
                # self.omega_shared[:] = self.vicon.rotation_rate
                # Share keyboard values
                self.keyboard.command.update()
                # Share command value
                for i in range(config.MARVEL_NUM):
                    if self.swarm.mode == 0:
                        self.cmd_ref_shared[:] = self.keyboard.cmd
                    elif self.swarm.mode == 1:
                        self.pos_ref_shared[:] = self.keyboard.pos
                        self.rpy_ref_shared[:] = self.keyboard.rpy
                        self.vel_ref_shared[:] = self.keyboard.vel
                        self.agv_ref_shared[:] = self.keyboard.agv
                self.take_off_shared.value = self.keyboard.take_off
                self.stop_shared.value = self.keyboard.stop
                # Log values
                if self.swarm.mode == 1:
                    self.logger.log_append(int(round((current_time-self.start_time) * 1000)), int(round((current_time-self.last_loop_time) * 1000)),
                                            self.pos_shared[:], self.vel_shared[:], self.quat_shared[:], self.omega_shared[:],
                                            self.pos_ref_shared[:], self.rpy_ref_shared[:],
                                            self.vel_ref_shared[:], self.agv_ref_shared[:])
                self.last_loop_time = current_time
            else:
                time.sleep(0.0001)
        self.stop()

    def stop(self):
        self.keyboard.command.quit()
        self.keyboard.command.destroy()
        if self.swarm.mode == 0:
            self.swarm.logger.savelog()
            self.logger.plot()
        elif self.swarm.mode == 1:
            self.logger.savelog()
            self.logger.plot()
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
