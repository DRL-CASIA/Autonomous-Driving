import time
import sysv_ipc
from utils import *

class Env(object):
    def __init__(self, shm_sz=640*480*3+4*1+20*4, vision=False):
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

    def step(self, a):
        while True:
            shared = self.shared_memory.read(self.shm_sz)
            written = read_written(shared)
            if written == 1:
                ob, r, term, info = self.read_(shared)
                self.write_(a)
                return ob, r, term, info

    def read_(self, shared):
        speed = read_speed(shared)  # [0, 50]
        to_track_middle = read_to_track_middle(shared)  # [-1, 1]
        angle = read_angle(shared)  # [-pi, pi]
        dist_raced = read_dist_raced(shared)  # to_track_middle_m
        sl_x = read_sl_x(shared)
        sl_y = read_sl_y(shared)
        sr_x = read_sr_x(shared)
        sr_y = read_sr_y(shared)
        track_coord = [sl_x, sl_y, sr_x, sr_y]
        car_x = read_car_x(shared)
        car_y = read_car_y(shared)        
        car_pos = [car_x, car_y]
        info = {"speed": speed, "to_track_middle": to_track_middle,
                "angle": angle, "dist_raced": dist_raced,
                "track_coord": track_coord, "car_pos": car_pos}
        if self.vision:
            ob = read_img(shared)
        else:
            lsr = [angle/3.1416, to_track_middle, speed*3.6/70]
            img = read_img(shared)
            ob = [lsr, img]

        if abs(to_track_middle) <= 1.0:
            term = False
        else:
            term = True
        if term:
            reward = -1.0
        else:
            reward = np.cos(angle) - np.sin(angle) - np.abs(to_track_middle)
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
        return ob, info

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
                    print "Count down:", (3 - i)
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
