import numpy as np
import h5py
import scipy
from PIL import Image
from scipy import ndimage
import tensorflow as tf
import scipy.misc
import model
import cv2
from subprocess import call
import matplotlib.pyplot as plt

sess = tf.InteractiveSession()
saver = tf.train.Saver()
saver.restore(sess, "C:\\Users\\Chen\\Desktop\\New\\save\\model.ckpt")

img = cv2.imread('steering_wheel_image.jpg',0)
rows,cols = img.shape
smoothed_angle = 0

#truth_normalized=[]
truth=[]
step=[]
count=0

#G:\\ThinkPad_backup\\torcs\\codebase\\data_collector\\dataset\\Brondehach_lane_2_dc_1.h5
f = h5py.File('Aalborg_lane_2_dc_1.h5', 'r')
hset = f['host_data']
for name in hset:
    s = 'host_data' + '/' + name + '/' + 'steer'
#    truth_normalized.append(f[s].value* 180.0 / scipy.pi)
    truth.append(f[s].value)
    step.append(count)
    count=count+1

print(count)

prediction_list=[]
degrees_list=[]

i = 0
for i in range(count):
#    while(cv2.waitKey(10) != ord('q')):
        full_image = 'host_data/frame_'+str(i)+'/image'
        image = scipy.misc.imresize(f[full_image].value, [66, 200]) / 255.0
        prediction = model.y.eval(feed_dict={model.x: [image], model.keep_prob: 1.0})[0][0]
        degrees = prediction * 180.0 / scipy.pi
        prediction_list.append(prediction)
        degrees_list.append(degrees)
        call("cls", shell=True)
        print("Predicted steering angle: " + str(degrees) + " degrees")
        cv2.imshow("frame", cv2.cvtColor(f[full_image].value, cv2.COLOR_RGB2BGR))
        #make smooth angle transitions by turning the steering wheel based on the difference of the current angle
        #and the predicted angle
        smoothed_angle += 0.2 * pow(abs((degrees - smoothed_angle)), 2.0 / 3.0) * (degrees - smoothed_angle) / abs(degrees - smoothed_angle)
        M = cv2.getRotationMatrix2D((cols/2,rows/2),-smoothed_angle,1)
        dst = cv2.warpAffine(img,M,(cols,rows))
        cv2.imshow("steering wheel", dst)
        i = i+1

cv2.destroyAllWindows()

error = []
for i in range(count):
    error.append(truth[i] - prediction_list[i])

print("Errors: ", error)
print(error)

squaredError = []
absError = []
for val in error:
    squaredError.append(val * val)  # target-prediction之差平方
    absError.append(abs(val))  # 误差绝对值

print("Square Error: ", squaredError)
print("Absolute Value of Error: ", absError)

print("MSE = ", sum(squaredError) / len(squaredError))  # 均方误差MSE


plt.figure('Line fig')
ax = plt.gca()
#设置x轴、y轴名称
ax.set_xlabel('x')
ax.set_ylabel('y')

#画连线图，以x_list中的值为横坐标，以y_list中的值为纵坐标
#参数c指定连线的颜色，linewidth指定连线宽度，alpha指定连线的透明度
#ax.plot(step, truth, color='r', linewidth=1, alpha=0.6)
#ax.plot(step, truth_normalized, color='coral', linewidth=1, alpha=0.6)
#ax.plot(step, prediction_list, color='b', linewidth=1, alpha=0.6)
ax.plot(step, degrees_list, color='g', linewidth=1, alpha=0.6)

plt.show()

