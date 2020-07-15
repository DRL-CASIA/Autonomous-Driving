import tensorflow as tf
import scipy

#初始化权值
def weight_variable(shape):
  initial = tf.truncated_normal(shape, stddev=0.1)
  #从截断的正态分布中输出随机值。 shape表示生成张量的维度，mean是均值，stddev是标准差。
  return tf.Variable(initial)
  #变量(tf.Variable)用于保存,更新神经网络的参数张量

#初始化偏置值
def bias_variable(shape):
  initial = tf.constant(0.1, shape=shape)
  #生成一个给定值的常量
  return tf.Variable(initial)

def conv2d(x, W, stride):
  return tf.nn.conv2d(x, W, strides=[1, stride, stride, 1], padding='VALID')
'''
tf.nn.conv2d(input, filter, strides, padding, use_cudnn_on_gpu=None, name=None)
除去name参数用以指定该操作的name，与方法有关的一共五个参数：

第一个参数input：指需要做卷积的输入图像，它要求是一个Tensor，具有[batch, in_height, in_width, in_channels]这样的shape，
具体含义是[训练时一个batch的图片数量, 图片高度, 图片宽度, 图像通道数]，注意这是一个4维的Tensor，要求类型为float32和float64其中之一

第二个参数filter：相当于CNN中的卷积核，它要求是一个Tensor，具有[filter_height, filter_width, in_channels, out_channels]这样的shape，
具体含义是[卷积核的高度，卷积核的宽度，图像通道数，卷积核个数]，要求类型与参数input相同，有一个地方需要注意，第三维in_channels，就是参数input的第四维

第三个参数strides：卷积时在图像每一维的步长，这是一个一维的向量，长度4
对于图片，因为只有两维，通常strides取[1，stride，stride，1]

第四个参数padding：string类型的量，只能是"SAME","VALID"其中之一，当其为‘SAME’时，表示卷积核可以停留在图像边缘，边缘外自动补0

第五个参数：use_cudnn_on_gpu:bool类型，是否使用cudnn加速，默认为true

结果返回一个Tensor，这个输出，就是我们常说的feature map，shape仍然是[batch, height, width, channels]这种形式。
'''

with tf.name_scope('input_layer'): #输入层。将这两个变量放到input_layer作用域下，tensorboard会把他们放在一个图形里面
# 声明一个占位符，None表示输入图片的数量不定，66*200图片分辨率，3通道颜色？
  x = tf.placeholder(tf.float32, shape=[None, 66, 200, 3])
# 类别是0-1总共2个类别，对应输出分类结果
  y_ = tf.placeholder(tf.float32, shape=[None, 1])
'''
dtype：数据类型。常用的是tf.float32,tf.float64等数值类型
shape：数据形状。默认是None，就是一维值，也可以是多维（比如[2,3], [None, 3]表示列是3，行不定）
name：名称
'''

x_image = x

#first convolutional layer
# 第一二参数值得卷积核尺寸大小，即patch，第三个参数是图像通道数，第四个参数是卷积核的数目，代表会出现多少个卷积特征图像;
W_conv1 = weight_variable([5, 5, 3, 24])
# 对于每一个卷积核都有一个对应的偏置量
b_conv1 = bias_variable([24])
# 图片乘以卷积核，步长为2，并加上偏执量
h_conv1 = tf.nn.relu(conv2d(x_image, W_conv1, 2) + b_conv1)
#这个函数的作用是计算激活函数 relu，即 max(features, 0)。即将矩阵中每行的非最大值置0。

#second convolutional layer
W_conv2 = weight_variable([5, 5, 24, 36])
b_conv2 = bias_variable([36])

h_conv2 = tf.nn.relu(conv2d(h_conv1, W_conv2, 2) + b_conv2)

#third convolutional layer
W_conv3 = weight_variable([5, 5, 36, 48])
b_conv3 = bias_variable([48])

h_conv3 = tf.nn.relu(conv2d(h_conv2, W_conv3, 2) + b_conv3)

#fourth convolutional layer
W_conv4 = weight_variable([3, 3, 48, 64])
b_conv4 = bias_variable([64])

h_conv4 = tf.nn.relu(conv2d(h_conv3, W_conv4, 1) + b_conv4)

#fifth convolutional layer
W_conv5 = weight_variable([3, 3, 64, 64])
b_conv5 = bias_variable([64])

h_conv5 = tf.nn.relu(conv2d(h_conv4, W_conv5, 1) + b_conv5)

#FCL 1
# 二维张量，第一个参数 可以认为是只有一行 卷积输出图像像素*图片数量 的卷积，第二个参数代表卷积个数共1164个
W_fc1 = weight_variable([1152, 1164])
b_fc1 = bias_variable([1164])

# 将卷积结果reshape成1152列数据，转置
h_conv5_flat = tf.reshape(h_conv5, [-1, 1152])
# 卷积操作，matmul实现最基本的矩阵相乘，不同于tf.nn.conv2d的遍历相乘，自动认为是前行向量后列向量
h_fc1 = tf.nn.relu(tf.matmul(h_conv5_flat, W_fc1) + b_fc1)

# 使用占位符，由dropout自动确定scale，也可以自定义，比如0.5
keep_prob = tf.placeholder(tf.float32)
# dropout操作，减少过拟合，其实就是降低上一层某些输入的权重scale，甚至置为0，升高某些输入的权值，甚至置为2，防止评测曲线出现震荡，个人觉得样本较少时很必要
h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

#FCL 2
W_fc2 = weight_variable([1164, 100])
b_fc2 = bias_variable([100])

h_fc2 = tf.nn.relu(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

h_fc2_drop = tf.nn.dropout(h_fc2, keep_prob)

#FCL 3
W_fc3 = weight_variable([100, 50])
b_fc3 = bias_variable([50])

h_fc3 = tf.nn.relu(tf.matmul(h_fc2_drop, W_fc3) + b_fc3)

h_fc3_drop = tf.nn.dropout(h_fc3, keep_prob)

#FCL 4
W_fc4 = weight_variable([50, 10])
b_fc4 = bias_variable([10])

h_fc4 = tf.nn.relu(tf.matmul(h_fc3_drop, W_fc4) + b_fc4)

h_fc4_drop = tf.nn.dropout(h_fc4, keep_prob)

#Output
W_fc5 = weight_variable([10, 1])
b_fc5 = bias_variable([1])

y = tf.multiply(tf.atan(tf.matmul(h_fc4_drop, W_fc5) + b_fc5), 2) #scale the atan output
#tf.atan计算arctanx的值，tf.mul相乘
