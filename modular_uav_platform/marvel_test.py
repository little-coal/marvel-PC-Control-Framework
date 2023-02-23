import time
import cflib
from marvel import Marvel



def send_twoD_setpoint_test(marvel):
    index = 0
    w, x, y, z  = 1.0, 0.0, 0.0, 0.0
    alpha, beta, thrust = 0.0, 0.0, 0.0
    current_time = time.time()
    last_time = current_time
    while 1:
        if current_time - last_time > 5:
            break
        marvel.send_twoD_setpoint(index, w, x, y, z, alpha, beta, thrust)
        current_time = time.time()    
    marvel._stop_crazyflie()
    print('Stop')

def send_position_setpoint_test(marvel):
    x, y, z, yaw = 0.0, 0.0, 0.2, 0.0
    current_time = time.time()
    last_time = current_time
    while 1:
        if current_time - last_time > 5:
            break
        marvel.send_position_setpoint(x, y, z, yaw)
        current_time = time.time()    
    marvel._stop_crazyflie()
    print('Stop')

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    link_uri = 'radio://0/100/2M/E7E7E7E7E0'
    
    marvel = Marvel(link_uri, 0)
    time.sleep(1)
    send_twoD_setpoint_test(marvel)
    time.sleep(1)
    send_position_setpoint_test(marvel)