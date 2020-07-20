import time
import sysv_ipc
from utils import *

class Env(object):
    def __init__(self, shm_sz=640*480*3+4*1+54*4, vision=False):
        self.shm_sz = shm_sz
        self.vision = vision
        self.first_launch = True
        os.system("pkill torcs")
        time.sleep(0.5)
        os.system("torcs &")
        time.sleep(0.5)
        os.system("sh autostart.sh")
        time.sleep(0.5)
        self.shared_memory = sysv_ipc.SharedMemory(934)
        # check if launch successfully
        self.check_launch()
        self.steps_ = 0

    def step(self, a):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                ob, r, term, info = self.read_(shared)
                self.write_(a)
                self.steps_ += 1
                return ob, r, term, info

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
            ob = [angle, to_track_middle, speed*3.6/70]
            # img = read_img(shared)
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
            for i in range(5):
                time.sleep(1)
                shared = self.shared_memory.read(self.shm_sz)
                written = read_written(shared)
                print('written = ', written)
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
