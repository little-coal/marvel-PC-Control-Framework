import time
import config
import logging
import cflib.crtp
import multiprocessing as mp

from marvel import Marvel
from marvel_keyboard import KeyboardInput
from marvel_logger import Logger

class Swarm():
    def __init__(self, num=config.MARVEL_NUM, mode=True, control_mode='position'):
        """
            Swarm has 2 mode: Swarm and CombinedPlatform
        """
        self.marvel_num = num
        self.mode = mode   #true refers to swarm, false refers to conbinedFrame
        self.control_mode = control_mode  #'position': x, y, z, yaw
        self._init_time()
        self._init_logger()

    def run(self, pos_ref_shared=None, rpy_ref_shared=None, vel_ref_shared=None, agv_ref_shared=None, cmd_ref_shared=[0.0, 0.0, 0.2, 0.0],
                pos_shared=[0.0, 0.0, 0.0], vel_shared=[0.0, 0.0, 0.0], quat_shared=[1.0, 0.0, 0.0, 0.0], omega_shared=[0.0, 0.0, 0.0], 
                take_off_shared=None, stop_shared=None):
        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers()
        self._connect()
        self.start_time = time.time()
        while 1:
            if stop_shared.value == 1:
                break
            self.current_time = time.time()
            # Record value
            # if self.mode:
            #     pos_ref_shared[:] = self.marvel_swarm_handle[i].target_pos_and_rot[0:3]
            #     rpy_ref_shared[:] = self.marvel_swarm_handle[i].target_pos_and_rot[3:6]
            #     vel_ref_shared[:] = self.marvel_swarm_handle[i].target_velocity[0:3]
            #     agv_ref_shared[:] = self.marvel_swarm_handle[i].target_velocity[3:6]
            if self.current_time - self.last_loop_time > self.update_time:
                if self.mode:
                    if take_off_shared.value == 1:
                        self._send_extpose(pos_shared, quat_shared)
                        self._send_cmd(cmd_ref_shared)        
                else:
                    pass
                self._record(pos_shared, vel_shared, quat_shared, omega_shared)
                self.dt = self.current_time - self.last_loop_time
                self.last_loop_time = self.current_time
                # self.log.append(self.dt)
            else:
                time.sleep(0.001)
        self._stop()

    def _init_time(self):
        self.marvel_setup_time = 0.25
        self.update_time = 0.01 # 100Hz
        self.last_loop_time = time.time()
        self.current_time = self.last_loop_time
        self.dt = 0
        # self.log = []

    def _init_logger(self, path='./logs'):
        self.logger = Logger(folder_name=path)
    
    def _connect(self):
        self.marvel_swarm_handle = []
        # Scan for Crazyflies and initialize marvels
        for i in range(self.marvel_num):
            marvel = Marvel('radio://0/100/2M/E7E7E7E7E{}'.format(i), i)
            self.marvel_swarm_handle.append(marvel)
            # print("Find MARVEL num:{}".format(len(self.marvel_swarm_handle)))
        print("------MARVEL Connection Completed------")

    def _send_extpose(self, pos_shared, quat_shared):
        for i in range(self.marvel_num):
            self.marvel_swarm_handle[i].send_extpose(
                pos_shared[0], pos_shared[1], pos_shared[2], 
                quat_shared[1], quat_shared[2], quat_shared[3], quat_shared[0]) #x, y, z, w
    
    def _send_cmd(self, cmd_ref_shared):
        for i in range(self.marvel_num):
            if self.control_mode == 'position':
                self.marvel_swarm_handle[i].send_position_setpoint( 
                        cmd_ref_shared[0], cmd_ref_shared[1], cmd_ref_shared[2], cmd_ref_shared[3])
    
    def _record(self, pos_shared, vel_shared, quat_shared, omega_shared):
        for i in range(self.marvel_num):
            pos_ref_record = self.marvel_swarm_handle[i].target_pos_and_rot[0:3]
            rpy_ref_record = self.marvel_swarm_handle[i].target_pos_and_rot[3:6]
            vel_ref_record = self.marvel_swarm_handle[i].target_velocity[0:3]
            agv_ref_record = self.marvel_swarm_handle[i].target_velocity[3:6]
        self.logger.log_append(int(round((self.current_time-self.start_time) * 1000)), 
                            int(round((self.current_time-self.last_loop_time) * 1000)),
                            pos_shared[:], vel_shared[:], quat_shared[:], omega_shared[:],
                            pos_ref_record, rpy_ref_record, vel_ref_record, agv_ref_record)

    def _stop(self):
        for i in range(self.marvel_num):
            self.marvel_swarm_handle[i]._stop_crazyflie()
        self.logger.savelog()
        self.logger.plot()

if __name__ == "__main__":
    takeoff = mp.Value('i', 0)
    stop = mp.Value('i', 1)
    
    swarm = Swarm()
    swarm.run(take_off_shared=takeoff, stop_shared=stop)