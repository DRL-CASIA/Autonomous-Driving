import h5py
import scipy.misc
import random
import os
import numpy as np

xs = []
ys = []

#points to the end of the last batch
train_batch_pointer = 0
val_batch_pointer = 0

h5_count=3


root1 = 'Aalborg_lane_2_dc_1.h5'
root2 = 'Alpine1_lane_2_dc_1.h5'
root3 = 'Alpine2_lane_2_dc_1.h5'
root=[root1,root2,root3]

#read data.txt
for i in range(h5_count):
    f = h5py.File(root[i], 'r')
    hset = f['host_data']
    for name in hset:
        s = 'host_data' + '/' + name + '/' + 'steer'
        im = 'host_data' + '/' + name + '/' + 'image'
        xs.append(f[im].value)
        ys.append(f[s].value)

#get number of images
num_images = len(xs)
print(num_images)

#shuffle list of images
c = list(zip(xs, ys))
random.shuffle(c)
xs, ys = zip(*c)

#分割训练集测试集
train_xs = xs[:int(len(xs) * 0.8)]
train_ys = ys[:int(len(xs) * 0.8)]

val_xs = xs[-int(len(xs) * 0.2):]
val_ys = ys[-int(len(xs) * 0.2):]

num_train_images = len(train_xs)
num_val_images = len(val_xs)

#加载训练集
def LoadTrainBatch(batch_size):
    global train_batch_pointer
    x_out = []
    y_out = []
    for i in range(0, batch_size):
        x_out.append(scipy.misc.imresize(train_xs[int((train_batch_pointer+i)%num_train_images)],[66, 200])/255.0)
        # scipy.misc.imresize输入固定大小调整shape
        #除255归一化
        #scipy.misc.imread返回The array obtained by reading the image.
        y_out.append([train_ys[int((train_batch_pointer+i)%num_train_images)]])
    train_batch_pointer+=batch_size
    return x_out, y_out

#加载测试集
def LoadValBatch(batch_size):
    global val_batch_pointer
    x_out = []
    y_out = []
    for i in range(0, batch_size):
        x_out.append(scipy.misc.imresize(val_xs[(val_batch_pointer + i) % num_val_images], [66, 200]) / 255.0)
        y_out.append([val_ys[(val_batch_pointer + i) % num_val_images]])
    val_batch_pointer += batch_size
    return x_out, y_out