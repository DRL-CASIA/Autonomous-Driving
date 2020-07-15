# import cv2
import numpy as np
from struct import pack, unpack
import os

IMG_SIZE = 640*480*3
F_OFFSET = 4
F_SIZE = 4
UCH_SIZE = 1

# mean_file = np.load("./mean_file.npy", "r")

# def prepross(img):
#     r = img[:,:,0].astype("float") - mean_file[0,:,:]
#     g = img[:,:,1].astype("float") - mean_file[1,:,:]
#     b = img[:,:,2].astype("float") - mean_file[2,:,:]
#     img_ = np.array([r, g, b]) # (C,H,W)
#     return np.transpose(img_, (1,2,0)) # (H, W, C)

# class Render(object):
#     """docstring for Render"""
#     def __init__(self):
#         self.window = cv2.namedWindow("TORCS", cv2.WINDOW_AUTOSIZE)

#     def update(self, img):
#         cv2.imshow("TORCS", img)
#         cv2.waitKey(3)

# Read from shared memory
'''
struct shared_use_st
{
    // N.B. always put float behind uint8 to save memory.
    uint8_t data[image_width*image_height*3];
    uint8_t written;        // read & write
    uint8_t restart;        // only write
    uint8_t relaunch;       // only write
    uint8_t placeholder_2;  // no ops
    float steer;            // only read
    float acc;
    float brake;
    float speed;            // only write
    float to_track_middle;  // only write
    float angle;            // only write
    float dist_raced;       // only write 
    float toMarking_L;
    float toMarking_M;
    float toMarking_R;
    float toMarking_LL;
    float toMarking_ML;
    float toMarking_MR;
    float toMarking_RR;
    // Data collection
    float host_car_x;
    float host_car_y;
    float host_dist_to_middle;
    float dc1_car_x;
    float dc1_car_y;
    float dc1_dist_to_middle;
    float dc2_car_x;
    float dc2_car_y;
    float dc2_dist_to_middle;
    float dc3_car_x;
    float dc3_car_y;
    float dc3_dist_to_middle;
    float host_track_vertex[2*4*10]; // 20 seg, 4 point (x,y) for each
    float dc1_track_vertex[2*4*10];
    float dc2_track_vertex[2*4*10];
    float dc3_track_vertex[2*4*10];
};
'''
def read_img(shared):
    img = np.fromstring(shared[:IMG_SIZE], dtype = np.uint8)
    img = img.reshape(480, 640, 3)
    # crop = img[480-320:, :, :]  # Cut out sky part
    # res = cv2.resize(img, (280, 210)) # 134
    # res = cv2.cvtColor(res, cv2.COLOR_RGB2GRAY)
    # assert res.shape == (210, 280), "image shape must be (210, 280)"
    return img

def read_steer(shared):
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 0*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 0*F_OFFSET + F_SIZE])[0]

def read_written(shared):
    # python2
    # return np.fromstring(shared[IMG_SIZE], dtype = np.uint8)[0]
    # python3
    return np.asarray(shared[IMG_SIZE], dtype=np.uint8)

def read_speed(shared):
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 3*F_OFFSET:
    IMG_SIZE + 4 * UCH_SIZE + 3 * F_OFFSET + F_SIZE])[0]

def read_to_track_middle(shared):
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 4*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 4*F_OFFSET + F_SIZE])[0]

def read_angle(shared):
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 5*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 5*F_OFFSET + F_SIZE])[0]

def read_dist_raced(shared):
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 6*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 6*F_OFFSET + F_SIZE])[0]

def read_lqr_x1(shared): # err_to_middle
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 4*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 4*F_OFFSET + F_SIZE])[0]

def read_lqr_x2(shared): # err_to_middle_rate
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 3*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 3*F_OFFSET + F_SIZE])[0]

def read_lqr_x3(shared): # err_angle
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 5*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 5*F_OFFSET + F_SIZE])[0]

def read_lqr_x4(shared): # err_angle_rate
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 6*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 6*F_OFFSET + F_SIZE])[0]

def read_lqr_x5(shared): # normalized to_middle, used to compute r
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 7*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 7*F_OFFSET + F_SIZE])[0]

def read_lqr_x6(shared):
    return unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + 8 * F_OFFSET:
    IMG_SIZE + 4 * UCH_SIZE + 8 * F_OFFSET + F_SIZE])[0]

def read_pid_x1(shared): # err_angle
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 5*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 5*F_OFFSET + F_SIZE])[0]

def read_pid_x2(shared): # err_to_middle
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 4*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 4*F_OFFSET + F_SIZE])[0]

def read_pid_x3(shared): # to middle, unit: m
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 6*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 6*F_OFFSET + F_SIZE])[0]
        
def read_nmpc_x1(shared): # err_angle
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 5*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 5*F_OFFSET + F_SIZE])[0]

def read_nmpc_x2(shared): # err_to_middle
    return unpack("f", shared[IMG_SIZE + 4*UCH_SIZE + 4*F_OFFSET:
        IMG_SIZE + 4*UCH_SIZE + 4*F_OFFSET + F_SIZE])[0]

def read_host_data(shared):
    idx = 14
    img = read_img(shared)
    basics = [] # car_x, car_y, car_dist_to_middle
    for i in range(3):
        x = unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET:
                               IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET + F_SIZE])[0]
        basics.append(x)
        idx += 1
    vertex = read_dc1_seg_vertex(shared)
    return {"image": img, "basics": np.asarray(basics), "vertex": np.asarray(vertex)}

def read_dc1_data(shared):
    idx = 17
    basics = []
    for i in range(3):
        x = unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET:
                               IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET + F_SIZE])[0]
        basics.append(x)
        idx += 1
    vertex = read_dc1_seg_vertex(shared)
    return {"basics": np.asarray(basics), "vertex": np.asarray(vertex)}

def read_dc2_data(shared):
    idx = 20
    basics = []
    for i in range(3):
        x = unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET:
                               IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET + F_SIZE])[0]
        basics.append(x)
        idx += 1
    vertex = read_dc1_seg_vertex(shared)
    return {"basics": np.asarray(basics), "vertex": np.asarray(vertex)}

def read_dc3_data(shared):
    idx = 23
    basics = []
    for i in range(3):
        x = unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET:
                               IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET + F_SIZE])[0]
        basics.append(x)
        idx += 1
    vertex = read_dc1_seg_vertex(shared)
    return {"basics": np.asarray(basics), "vertex": np.asarray(vertex)}

def read_host_seg_vertex(shared):
    idx = 26
    data = []
    for i in range(2*4*10): # 2 points, 2 coordinates, 10 seg
        x = unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET:
                               IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET + F_SIZE])[0]
        data.append(x)
        idx += 1
    return data

def read_dc1_seg_vertex(shared):
    idx = 26 + 1*80
    data = []
    for i in range(2 * 4 * 10):  # 2 points, 2 coordinates, 10 seg
        x = unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET:
                               IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET + F_SIZE])[0]
        data.append(x)
        idx += 1
    return data

def read_dc2_seg_vertex(shared):
    idx = 26 + 2*80
    data = []
    for i in range(2 * 4 * 10):  # 2 points, 2 coordinates, 10 seg
        x = unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET:
                               IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET + F_SIZE])[0]
        data.append(x)
        idx += 1
    return data

def read_dc3_seg_vertex(shared):
    idx = 26 + 3*80
    data = []
    for i in range(2 * 4 * 10):  # 2 points, 2 coordinates, 10 seg
        x = unpack("f", shared[IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET:
                               IMG_SIZE + 4 * UCH_SIZE + idx * F_OFFSET + F_SIZE])[0]
        data.append(x)
        idx += 1
    return data

def write_written(memory):
    memory.write("0", offset = IMG_SIZE)

def write_restart(memory):
    memory.write("1", offset = IMG_SIZE + 1 * UCH_SIZE)

def write_relaunch(memory):
    memory.write("1", offset = IMG_SIZE + 2 * UCH_SIZE)

def write_steer(memory, s):
    s_ = pack("f", s)
    memory.write(s_, offset = IMG_SIZE + 4 * UCH_SIZE)

def write_acc(memory, acc=0.2):
    acc_ = pack('f', acc)
    memory.write(acc_, offset = IMG_SIZE + 4 * UCH_SIZE + 1 * F_OFFSET)

def write_brake(memory, brake=0.0):
    brake_ = pack('f', brake)
    memory.write(brake_, offset = IMG_SIZE + 4 * UCH_SIZE + 2 * F_OFFSET)
