"""
Author: Chi Chu
Last Update Date: 1/30/2023

This is the class of marvel, a single modular uav platform.
Marvel: Class of the single modular UAV.

"""

import time
import logging
import numpy as np

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

from marvel_logger import Logger

logging.basicConfig(level=logging.ERROR)

class Marvel:
    def __init__(self, link_uri, index):
        """ Initialize"""

        self._cf = Crazyflie(rw_cache='./cache')

        # Add some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)
        self.index = index
        
        # Log's variables
        self.timestamp = np.zeros([1])
        self.motor = np.zeros([4])              
        self.pos_and_rot = np.zeros([6])        
        self.velocity = np.zeros([6])           
        self.target_pos_and_rot = np.zeros([6])
        self.target_velocity = np.zeros([6])

    def _connected(self, link_uri):
        """Print uri and initialize a logger"""
        
        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True
        print('Connected to %s' % link_uri)

        # Define logconfig
        # Attention: each logconfig object can add max to 6 variables
        self._lg_vbattery = LogConfig(name='vbattery', period_in_ms=10)
        self._lg_vbattery.add_variable('pm.vbat', 'FP16')
        self._lg_pos_and_rot = LogConfig(name='pos_and_rot', period_in_ms=10)
        self._lg_pos_and_rot.add_variable('stateEstimate.x', 'float')
        self._lg_pos_and_rot.add_variable('stateEstimate.y', 'float')
        self._lg_pos_and_rot.add_variable('stateEstimate.z', 'float') 
        self._lg_pos_and_rot.add_variable('stateEstimate.roll', 'float')
        self._lg_pos_and_rot.add_variable('stateEstimate.pitch', 'float')
        self._lg_pos_and_rot.add_variable('stateEstimate.yaw', 'float')
        self._lg_velocity = LogConfig(name='velocity', period_in_ms=10)
        self._lg_velocity.add_variable('stateEstimate.vx', 'float')
        self._lg_velocity.add_variable('stateEstimate.vy', 'float')
        self._lg_velocity.add_variable('stateEstimate.vz', 'float')
        self._lg_velocity.add_variable('controller.r_roll', 'float')
        self._lg_velocity.add_variable('controller.r_pitch', 'float')
        self._lg_velocity.add_variable('controller.r_yaw', 'float')
        self._lg_target_pos_and_rot = LogConfig(name='target_pos_and_rot', period_in_ms=10)
        self._lg_target_pos_and_rot.add_variable('ctrltarget.x', 'float')
        self._lg_target_pos_and_rot.add_variable('ctrltarget.y', 'float')
        self._lg_target_pos_and_rot.add_variable('ctrltarget.z', 'float')
        self._lg_target_pos_and_rot.add_variable('controller.roll', 'float')
        self._lg_target_pos_and_rot.add_variable('controller.pitch', 'float')
        self._lg_target_pos_and_rot.add_variable('controller.yaw', 'float')
        self._lg_target_velocity = LogConfig(name='target_velocity', period_in_ms=10)
        self._lg_target_velocity.add_variable('ctrltarget.vx', 'float')
        self._lg_target_velocity.add_variable('ctrltarget.vy', 'float')
        self._lg_target_velocity.add_variable('ctrltarget.vz', 'float')
        self._lg_target_velocity.add_variable('controller.rollRate', 'float')
        self._lg_target_velocity.add_variable('controller.pitchRate', 'float')
        self._lg_target_velocity.add_variable('controller.yawRate', 'float')
        self._lg_motor = LogConfig(name='motor', period_in_ms=10)
        self._lg_motor.add_variable('motor.m1', 'uint32_t')
        self._lg_motor.add_variable('motor.m2', 'uint32_t')
        self._lg_motor.add_variable('motor.m3', 'uint32_t')
        self._lg_motor.add_variable('motor.m4', 'uint32_t')
        # self._lg_pid_rate_roll = LogConfig(name='pid_rate_roll', period_in_ms=10)
        # self._lg_pid_rate_roll.add_variable('pid_rate.roll_outP', 'float')
        # self._lg_pid_rate_roll.add_variable('pid_rate.roll_outI', 'float')
        # self._lg_pid_rate_roll.add_variable('pid_rate.roll_outD', 'float')
        # self._lg_pid_rate_pitch = LogConfig(name='pid_rate_pitch', period_in_ms=10)
        # self._lg_pid_rate_pitch.add_variable('pid_rate.pitch_outP', 'float')
        # self._lg_pid_rate_pitch.add_variable('pid_rate.pitch_outI', 'float')
        # self._lg_pid_rate_pitch.add_variable('pid_rate.pitch_outD', 'float')
        # self._lg_pid_rate_yaw = LogConfig(name='pid_rate_yaw', period_in_ms=10)
        # self._lg_pid_rate_yaw.add_variable('pid_rate.yaw_outP', 'float')
        # self._lg_pid_rate_yaw.add_variable('pid_rate.yaw_outI', 'float')
        # self._lg_pid_rate_yaw.add_variable('pid_rate.yaw_outD', 'float')
        # self._lg_pid_atittude_roll = LogConfig(name='pid_atittude_roll', period_in_ms=10)
        # self._lg_pid_atittude_roll.add_variable('pid_attitude.roll_outP', 'float')
        # self._lg_pid_atittude_roll.add_variable('pid_attitude.roll_outI', 'float')
        # self._lg_pid_atittude_roll.add_variable('pid_attitude.roll_outD', 'float')
        # self._lg_pid_atittude_pitch = LogConfig(name='pid_atittude_pitch', period_in_ms=10)
        # self._lg_pid_atittude_pitch.add_variable('pid_attitude.pitch_outP', 'float')
        # self._lg_pid_atittude_pitch.add_variable('pid_attitude.pitch_outI', 'float')
        # self._lg_pid_atittude_pitch.add_variable('pid_attitude.pitch_outD', 'float')
        # self._lg_pid_atittude_yaw = LogConfig(name='pid_atittude_yaw', period_in_ms=10)
        # self._lg_pid_atittude_yaw.add_variable('pid_attitude.yaw_outP', 'float')
        # self._lg_pid_atittude_yaw.add_variable('pid_attitude.yaw_outI', 'float')
        # self._lg_pid_atittude_yaw.add_variable('pid_attitude.yaw_outD', 'float')

        try:
            self._cf.log.add_config(self._lg_vbattery)
            self._cf.log.add_config(self._lg_pos_and_rot)
            self._cf.log.add_config(self._lg_velocity)
            self._cf.log.add_config(self._lg_target_pos_and_rot)
            self._cf.log.add_config(self._lg_target_velocity)
            self._cf.log.add_config(self._lg_motor)
            # This callback will receive the data
            self._lg_vbattery.data_received_cb.add_callback(self._battery_log_data)
            self._lg_pos_and_rot.data_received_cb.add_callback(self._pos_and_rot_log_data)
            self._lg_velocity.data_received_cb.add_callback(self._velocity_log_data)
            self._lg_target_pos_and_rot.data_received_cb.add_callback(self._target_pos_and_rot_log_data)
            self._lg_target_velocity.data_received_cb.add_callback(self._target_velocity_log_data)
            self._lg_motor.data_received_cb.add_callback(self._motor_log_data)
            # This callback will be called on errors
            self._lg_vbattery.error_cb.add_callback(self._log_error)
            self._lg_pos_and_rot.error_cb.add_callback(self._log_error)
            self._lg_velocity.error_cb.add_callback(self._log_error)
            self._lg_target_pos_and_rot.error_cb.add_callback(self._log_error)
            self._lg_target_velocity.error_cb.add_callback(self._log_error)
            self._lg_motor.error_cb.add_callback(self._log_error)
            # Start the logging
            self._lg_vbattery.start()
            self._lg_pos_and_rot.start()
            self._lg_velocity.start()
            self._lg_target_pos_and_rot.start()
            self._lg_target_velocity.start()
            self._lg_motor.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _battery_log_data(self, timestamp, data, logconf):
        """Report battery voltage information once"""
        battery_data = round(data['pm.vbat'], 1)
        print('Battery voltage of |CF %s| is: |3.1V(E)| --- %s V --- |4.2V(F)|' % (self.index, battery_data))
        self._lg_vbattery.data_received_cb.remove_callback(self._battery_log_data)

    def _pos_and_rot_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.pos_and_rot[0] = data['stateEstimate.x']
        self.pos_and_rot[1] = data['stateEstimate.y']
        self.pos_and_rot[2] = data['stateEstimate.z']
        self.pos_and_rot[3] = data['stateEstimate.roll']
        self.pos_and_rot[4] = data['stateEstimate.pitch']
        self.pos_and_rot[5] = data['stateEstimate.yaw']

    def _velocity_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.velocity[0] = data['stateEstimate.vx']
        self.velocity[1] = data['stateEstimate.vy']
        self.velocity[2] = data['stateEstimate.vz']
        self.velocity[3] = data['controller.r_roll']
        self.velocity[4] = data['controller.r_pitch']
        self.velocity[5] = data['controller.r_yaw']
    
    def _target_pos_and_rot_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.target_pos_and_rot[0] = data['ctrltarget.x']
        self.target_pos_and_rot[1] = data['ctrltarget.y']
        self.target_pos_and_rot[2] = data['ctrltarget.z']
        self.target_pos_and_rot[3] = data['controller.roll']
        self.target_pos_and_rot[4] = data['controller.pitch']
        self.target_pos_and_rot[5] = data['controller.yaw']
    
    def _target_velocity_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.target_velocity[0] = data['ctrltarget.vx']
        self.target_velocity[1] = data['ctrltarget.vy']
        self.target_velocity[2] = data['ctrltarget.vz']
        self.target_velocity[3] = data['controller.rollRate']
        self.target_velocity[4] = data['controller.pitchRate']
        self.target_velocity[5] = data['controller.yawRate']

    def _motor_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.motor[0] = data['motor.m1']
        self.motor[1] = data['motor.m2']
        self.motor[2] = data['motor.m3']
        self.motor[3] = data['motor.m4']

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False
    
    def take_off(self, x, y, z, yaw):
        self._cf.commander.send_position_setpoint(x, y, z, yaw)

    def _stop_crazyflie(self):
        self._cf.commander.send_stop_setpoint()

    def activate_kalman_estimator(self):
        # Set estimator to kalman estimator
        self._cf.param.set_value('stabilizer.estimator', '2')

        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        self._cf.param.set_value('locSrv.extQuatStdDev', 0.06)

    def send_extpos(self, x, y, z):
        self._cf.extpos.send_extpos(x, y, z)

    def send_extpose(self, x, y, z, qx, qy, qz, qw):
        self._cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)

    def send_position_setpoint(self, x, y, z, yaw):
        self._cf.commander.send_position_setpoint(x=x, y=y, z=z, yaw=yaw)
    
    def send_setpoint(self, roll, pitch, yawrate, thrust):
        self._cf.commander.send_setpoint(roll=roll, pitch=pitch, yawrate=yawrate, thrust=thrust)
        
    def set_pid(self):
        self._cf.param.set_value('posCtlPid.thrustBase', 36000) #36000
        #attitude
        self._cf.param.set_value('pid_attitude.roll_kp', 6)   #6
        self._cf.param.set_value('pid_attitude.roll_ki', 3)   #3
        self._cf.param.set_value('pid_attitude.roll_kd', 0)
        self._cf.param.set_value('pid_attitude.pitch_kp', 6)
        self._cf.param.set_value('pid_attitude.pitch_ki', 3)
        self._cf.param.set_value('pid_attitude.pitch_kd', 0)
        self._cf.param.set_value('pid_attitude.yaw_kp', 6)    #6
        self._cf.param.set_value('pid_attitude.yaw_ki', 1)    #1
        self._cf.param.set_value('pid_attitude.yaw_kd', 0.35)    #0.35
        #angular v
        self._cf.param.set_value('pid_rate.roll_kp', 250.0)   #250
        self._cf.param.set_value('pid_rate.roll_ki', 500.0)   #500
        self._cf.param.set_value('pid_rate.roll_kd', 2.5)   #2.5
        self._cf.param.set_value('pid_rate.pitch_kp', 250.0) 
        self._cf.param.set_value('pid_rate.pitch_ki', 500.0)
        self._cf.param.set_value('pid_rate.pitch_kd', 2.5)
        self._cf.param.set_value('pid_rate.yaw_kp', 120.0)   #120
        self._cf.param.set_value('pid_rate.yaw_ki', 16.7)   #16.7
        self._cf.param.set_value('pid_rate.yaw_kd', 0.0)
        #position
        self._cf.param.set_value('velCtlPid.vxKp', 25.0) #25
        self._cf.param.set_value('velCtlPid.vxKi', 1.0) #1
        self._cf.param.set_value('velCtlPid.vxKd', 0.0) #0
        self._cf.param.set_value('velCtlPid.vyKp', 25.0)
        self._cf.param.set_value('velCtlPid.vyKi', 1.0)
        self._cf.param.set_value('velCtlPid.vyKd', 0.0)
        self._cf.param.set_value('velCtlPid.vzKp', 3.0) #3
        self._cf.param.set_value('velCtlPid.vzKi', 1.0) #1
        self._cf.param.set_value('velCtlPid.vzKd', 1.5) #1.5
        #vel
        self._cf.param.set_value('posCtlPid.xKp', 2.0) #2
        self._cf.param.set_value('posCtlPid.xKi', 0.0) #0
        self._cf.param.set_value('posCtlPid.xKd', 0.0) #0
        self._cf.param.set_value('posCtlPid.yKp', 2.0)
        self._cf.param.set_value('posCtlPid.yKi', 0.0)
        self._cf.param.set_value('posCtlPid.yKd', 0.0)
        self._cf.param.set_value('posCtlPid.zKp', 2.0) #2
        self._cf.param.set_value('posCtlPid.zKi', 0.5) #0.5
        self._cf.param.set_value('posCtlPid.zKd', 0.0) #0

if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    
    link_uri = 'radio://0/100/2M/E7E7E7E7E0'
    marvel = Marvel(link_uri, 0)
    logger = Logger()
    current_time, last_loop_time, start_time = time.time(), time.time(), time.time()
    update_time = 0.01

    pos_shared, vel_shared = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    rpy_shared, agv_shared = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    while 1:
        current_time = time.time()
        if current_time - last_loop_time > update_time:
            marvel.send_position_setpoint(0.0, 0.0, 0.2, 0.0)
            pos_ref_shared = marvel.target_pos_and_rot[0:3]
            rpy_ref_shared = marvel.target_pos_and_rot[3:6]
            vel_ref_shared = marvel.target_velocity[0:3]
            agv_ref_shared = marvel.target_velocity[3:6]
            # Ture time from second to milisecond
            logger.log_append(int(round((current_time-start_time) * 1000)), int(round((current_time-last_loop_time) * 1000)),
                                    pos_shared, vel_shared, rpy_shared, agv_shared,
                                    pos_ref_shared, rpy_ref_shared,
                                    vel_ref_shared, agv_ref_shared)
            last_time = current_time
        else:
            time.sleep(0.001)
        if current_time - start_time > 5:
            marvel._stop_crazyflie()
            logger.savelog()
            logger.plot()
            break
