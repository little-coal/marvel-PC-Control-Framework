from cProfile import label
import matplotlib.pyplot as plt

log_x, log_y, log_z, log_roll, log_pitch, log_yaw = [], [], [], [], [], []
log_vx, log_vy, log_vz, log_wroll, log_wpitch, log_wyaw = [], [], [], [], [], []
log_m1, log_m2, log_m3, log_m4 = [], [], [], []
log_pid_attitude_roll_outP, log_pid_attitude_roll_outI, log_pid_attitude_roll_outD = [], [], []
log_pid_attitude_pitch_outP, log_pid_attitude_pitch_outI, log_pid_attitude_pitch_outD = [], [], []
log_pid_attitude_yaw_outP, log_pid_attitude_yaw_outI, log_pid_attitude_yaw_outD = [], [], []
log_pid_rate_roll_outP, log_pid_rate_roll_outI, log_pid_rate_roll_outD = [], [], []
log_pid_rate_pitch_outP, log_pid_rate_pitch_outI, log_pid_rate_pitch_outD = [], [], []
log_pid_rate_yaw_outP, log_pid_rate_yaw_outI, log_pid_rate_yaw_outD = [], [], []
log_target_x, log_target_y, log_target_z, log_target_roll, log_target_pitch, log_target_yaw = [],[],[],[],[],[]
log_target_vx, log_target_vy, log_target_vz = [], [], []
log_target_wroll, log_target_wpitch, log_target_wyaw = [], [], []

def plot_log_pose():
    with open('./x.txt', 'r') as file:
        line = file.readline()
        while line:
            log_x.append(float(line))
            line = file.readline()
    
    with open('./y.txt', 'r') as file:
        line = file.readline()
        while line:
            log_y.append(float(line))
            line = file.readline()
    
    with open('./z.txt', 'r') as file:
        line = file.readline()
        while line:
            log_z.append(float(line))
            line = file.readline()

    with open('./roll.txt', 'r') as file:
        line = file.readline()
        while line:
            log_roll.append(float(line))
            line = file.readline()

    with open('./pitch.txt', 'r') as file:
        line = file.readline()
        while line:
            log_pitch.append(float(line))
            line = file.readline()

    with open('./yaw.txt', 'r') as file:
        line = file.readline()
        while line:
            log_yaw.append(float(line))
            line = file.readline()
    
    with open('./vx.txt', 'r') as file:
        line = file.readline()
        while line:
            log_vx.append(float(line))
            line = file.readline()
    
    with open('./vy.txt', 'r') as file:
        line = file.readline()
        while line:
            log_vy.append(float(line))
            line = file.readline()
    
    with open('./vz.txt', 'r') as file:
        line = file.readline()
        while line:
            log_vz.append(float(line))
            line = file.readline()
    
    with open('./ctrltarget_x.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_x.append(float(line))
            line = file.readline()

    with open('./ctrltarget_y.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_y.append(float(line))
            line = file.readline()
    
    with open('./ctrltarget_z.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_z.append(float(line))
            line = file.readline()

    with open('./ctrltarget_roll.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_roll.append(float(line))
            line = file.readline()

    with open('./ctrltarget_pitch.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_pitch.append(float(line))
            line = file.readline()

    with open('./ctrltarget_yaw.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_yaw.append(float(line))
            line = file.readline()

    with open('./ctrltarget_vx.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_vx.append(float(line))
            line = file.readline()

    with open('./ctrltarget_vy.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_vy.append(float(line))
            line = file.readline()

    with open('./ctrltarget_vz.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_vz.append(float(line))
            line = file.readline()
    
    with open('./ctrltarget_r_roll.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_wroll.append(float(line))
            line = file.readline()

    with open('./ctrltarget_r_pitch.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_wpitch.append(float(line))
            line = file.readline()

    with open('./ctrltarget_r_yaw.txt', 'r') as file:
        line = file.readline()
        while line:
            log_target_wyaw.append(float(line))
            line = file.readline()


    max_len = max(len(log_x), len(log_vx), len(log_target_x), len(log_target_vx))
    sequence_length = max_len

    while(len(log_x) < max_len):
        log_x.append(0)
    while(len(log_y) < max_len):
        log_y.append(0)
    while(len(log_z) < max_len):
        log_z.append(0)
    while(len(log_roll) < max_len):
        log_roll.append(0)
        log_pitch.append(0)
        log_yaw.append(0)
        # log_wroll.append(0)
        # log_wpitch.append(0)
        # log_wyaw.append(0)


    while(len(log_vx) < max_len):
        log_vx.append(0)
        log_vy.append(0)
        log_vz.append(0)

    while(len(log_target_x) < max_len):
        log_target_x.append(0)
        log_target_y.append(0)
        log_target_z.append(0)
        log_target_roll.append(0)
        log_target_pitch.append(0)
        log_target_yaw.append(0)
    
    while(len(log_target_vx) < max_len):
        log_target_vx.append(0)
    while(len(log_target_vy) < max_len):
        log_target_vy.append(0)
    while(len(log_target_vz)< max_len):
        log_target_vz.append(0)
    
    while(len(log_target_roll) < max_len):
        log_target_roll.append(0)
    while(len(log_target_pitch) < max_len):
        log_target_pitch.append(0)
    while(len(log_target_yaw) < max_len):
        log_target_yaw.append(0)

    dt = 0.01
    t = [i * dt for i in range(sequence_length)]
    log_row_old = 0
    log_pitch_old = 0
    log_yaw_old = 0
    for i in range(sequence_length):
        log_wroll.append(log_roll[i] - log_row_old)
        log_wpitch.append(log_pitch[i] - log_pitch_old)
        log_wyaw.append(log_yaw[i] - log_yaw_old)
        log_row_old = log_roll[i]
        log_pitch_old = log_pitch[i]
        log_yaw_old = log_yaw[i]
    

   

    plt.subplot(4, 3, 1)
    plt.plot(t, log_x, label='x')
    plt.plot(t, log_target_x, label='target_x')
    plt.title('x')
    plt.subplot(4, 3, 2)
    plt.plot(t, log_y)
    plt.plot(t, log_target_y, label='target_y')
    plt.title('y')
    plt.subplot(4, 3, 3)
    plt.plot(t, log_z)
    plt.plot(t, log_target_z, label='target_z')
    plt.title('z')
    plt.subplot(4, 3, 4)
    plt.plot(t, log_roll)
    plt.plot(t, log_target_roll, label='target_roll')
    plt.title('roll')
    plt.subplot(4, 3, 5)
    plt.plot(t, log_pitch)
    plt.plot(t, log_target_pitch, label='target_pitch')
    plt.title('pitch')
    plt.subplot(4, 3, 6)
    plt.plot(t, log_yaw)
    plt.plot(t, log_target_yaw, label='target_yaw')
    plt.title('yaw')
    plt.subplot(4, 3, 7)
    plt.plot(t, log_vx)
    plt.plot(t, log_target_vx, label='target_vx')
    plt.title('vx')
    plt.subplot(4, 3, 8)
    plt.plot(t, log_vy)
    plt.plot(t, log_target_vy, label='target_vy')
    plt.title('vy')
    plt.subplot(4, 3, 9)
    plt.plot(t, log_vz)
    plt.plot(t, log_target_vz, label='target_vz')
    plt.title('z')
    plt.subplot(4, 3, 10)
    plt.plot(t, log_wroll)
    # plt.plot(t, log_target_wroll)
    plt.title('wrow')
    plt.subplot(4, 3, 11)
    plt.plot(t, log_wpitch)
    # plt.plot(t, log_target_wpitch)
    plt.title('wpitch')
    plt.subplot(4, 3, 12)
    plt.plot(t, log_wyaw)
    # plt.plot(t, log_target_wyaw)
    plt.title('wyaw')

    plt.subplots_adjust(wspace=0.6, hspace=0.6)
    plt.suptitle('Log Position, Rotation, Velocity and Angular Velocity')
    plt.show()

def plot_log_thrust():
    with open('./m1.txt', 'r') as file:
        line = file.readline()
        while line:
            log_m1.append(float(line))
            line = file.readline()
    
    with open('./m2.txt', 'r') as file:
        line = file.readline()
        while line:
            log_m2.append(float(line))
            line = file.readline()

    with open('./m3.txt', 'r') as file:
        line = file.readline()
        while line:
            log_m3.append(float(line))
            line = file.readline()

    with open('./m4.txt', 'r') as file:
        line = file.readline()
        while line:
            log_m4.append(float(line))
            line = file.readline()

    sequence_length = len(log_m1)
    dt = 0.01
    t = [i * dt for i in range(sequence_length)]
    plt.subplot(2, 2, 1)
    plt.plot(t, log_m1)
    plt.title('log m1')
    plt.subplot(2, 2, 2)
    plt.plot(t, log_m2)
    plt.title('log m2')
    plt.subplot(2, 2, 3)
    plt.plot(t, log_m3)
    plt.title('log m3')
    plt.subplot(2, 2, 4)
    plt.plot(t, log_m4)
    plt.title('log m4')
    plt.subplots_adjust(wspace=0.6, hspace=0.6)
    plt.suptitle('Log Thrust Command')
    plt.show()

def plot_log_pid_attitude():
    pass

def plot_log_pid_attitude_rate():
    pass

def plot_log_pid_target():
    pass

def plot_log_pid_ctl():
    pass

if __name__ == '__main__':
    plot_log_pose()
    plot_log_thrust()