import time
import sysv_ipc
from utils import *

class Env(object):
    def __init__(self, shm_sz=640*480*3+4*1+26*4+4*2*4*10*4, vision=False):
        self.shm_sz = shm_sz
        self.vision = vision
        self.first_launch = True
        os.system("pkill torcs")
        time.sleep(0.5)
        os.system("torcs &")
        time.sleep(0.5)
        os.system("sh autostart.sh")
        time.sleep(0.5)
        self.shared_memory = sysv_ipc.SharedMemory(2345)
        # check if launch successfully
        self.check_launch()

    def step(self, a):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                ob, r, term, info = self.read_(shared)
                self.write_(a)
                return ob, r, term, info

    def step_lqr(self, a):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                self.write_(a)
                break

    def step_pid(self, a):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                self.write_(a)
                break

    def step_nmpc(self, a):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                self.write_(a)
                break

    def step_dc(self, a = 0):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                self.write_(a)
                break

    def read_(self, shared):
        speed = read_speed(shared)  # [0, 50]
        to_track_middle = read_to_track_middle(shared)  # [-1, 1]
        angle_ = read_angle(shared)
        angle = angle_/3.1416  # [-1, 1]
        to_track_middle_m = read_dist_raced(shared)
        # steer = read_steer(shared)
        info = {"speed": speed, "to_track_middle": to_track_middle,
                "angle": angle_, "to_middle_m": to_track_middle_m}
        if self.vision:
            ob = read_img(shared)
        else:
            lsr = [angle, to_track_middle, speed*3.6/70]
            # img = read_img(shared)
            ob = lsr
            # print "dist to middle:", to_track_middle

        if abs(to_track_middle) <= 1.0:
            term = False
        else:
            term = True

        reward = np.cos(angle) - np.sin(np.abs(angle)) - np.abs(to_track_middle)
        if term:
            reward = -1.0
        # print "reward: %.4f\tdist: %.3f\tangle: %.3f | %s" % (reward, to_track_middle, angle, case)
        return ob, reward, term, info

    def read_lqr_x(self):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                x1 = read_lqr_x1(shared) # to middle
                # x2 = read_lqr_x2(shared)
                x3 = read_lqr_x3(shared) # angle
                # x4 = read_lqr_x4(shared)
                x5 = read_lqr_x5(shared) # normed to middle
                x6 = read_lqr_x6(shared) # speed_x
                return [x1, x3, x5, x6]
    
    def read_pid_x(self):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                x1 = read_pid_x1(shared)
                x2 = read_pid_x2(shared)
                x3 = read_pid_x3(shared)
                return [x1, x2, x3]

    def read_nmpc_x(self):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                x1 = read_nmpc_x1(shared)
                x2 = read_nmpc_x2(shared)
                return [x1, x2]

    def read_host_data(self):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                return read_host_data(shared)

    def read_dc1_data(self):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                return read_dc1_data(shared)

    def read_dc2_data(self):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                return read_dc2_data(shared)

    def read_dc3_data(self):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                return read_dc3_data(shared)

    def write_(self, a):
        write_steer(self.shared_memory, a)
        write_written(self.shared_memory)

    def reset(self, relaunch = False):
        if not self.first_launch:
            if not relaunch:
                write_restart(self.shared_memory)
                write_written(self.shared_memory)
                # Loop till restart complete
                while True:
                    shared = self.shared_memory.read(self.shm_sz)
                    written = read_written(shared)
                    if written == 1:
                        ob, r, term, info = self.read_(shared)
                        write_written(self.shared_memory)
                        if not term:
                            break
            else:
                # relaunch here
                write_relaunch(self.shared_memory)
                write_written(self.shared_memory)
                time.sleep(0.5)
                os.system("pkill torcs")
                time.sleep(0.5)
                os.system("torcs &")
                time.sleep(0.5)
                os.system("sh autostart.sh")
                time.sleep(0.5)
                self.check_launch()

        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                ob, r, term, info = self.read_(shared)
                break
        self.first_launch = False
        return ob

    def end(self):
        os.system("pkill torcs")

    def check_launch(self):
        written = 0
        while True:
            for i in range(3):
                time.sleep(1)
                shared = self.shared_memory.read(self.shm_sz)
                written = read_written(shared)
                if written != 1:
                    print("Count down:", (3 - i))
                else:
                    break
            if written == 1:
                break
            os.system("pkill torcs")
            time.sleep(0.5)
            os.system("torcs &")
            time.sleep(0.5)
            os.system("sh autostart.sh")
            time.sleep(0.5)
