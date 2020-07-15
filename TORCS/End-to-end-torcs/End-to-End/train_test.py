import os
import tensorflow as tf
from tensorflow.core.protobuf import saver_pb2
import driving_data_test
import model

LOGDIR = 'C:\\Users\\Chen\\Desktop\\New\\save'

sess = tf.InteractiveSession()

L2NormConst = 0.001

train_vars = tf.trainable_variables() #返回所有可训练的变量

#损失函数，均方误差？添加了一些 regularization term(正则项) 避免参数过拟合。
loss = tf.reduce_mean(tf.square(tf.subtract(model.y_, model.y))) + tf.add_n([tf.nn.l2_loss(v) for v in train_vars]) * L2NormConst
#自适应学习率，反向传播算法
train_step = tf.train.AdamOptimizer(1e-4).minimize(loss)
#计算
sess.run(tf.initialize_all_variables())

# create a summary to monitor cost tensor，用tf.scalar_summary来收集想要显示的变量,命名为loss
tf.summary.scalar("loss", loss)
# merge all summaries into a single op，定义一个summury op, 用来汇总由scalar_summary记录的所有变量
merged_summary_op = tf.summary.merge_all()

# 保存成v1版本
saver = tf.train.Saver(write_version = saver_pb2.SaverDef.V1)

# op to write logs to Tensorboard，生成一个summary writer对象，需要指定写入路径
logs_path = 'C:\\Users\\Chen\\Desktop\\New\\logs'
summary_writer = tf.summary.FileWriter(logs_path, graph=tf.get_default_graph())

epochs = 30 #训练次数
batch_size = 100
output_file = open('results.txt', 'w')

# train over the dataset about 30 times
for epoch in range(epochs):
  count=driving_data_test.num_images/batch_size
  for i in range(int(count)): #有多少批
    xs, ys = driving_data_test.LoadTrainBatch(batch_size)
    #run带的参数表明的意义似乎是把占位符对应的原本值替换再跑一遍
    train_step.run(feed_dict={model.x: xs, model.y_: ys, model.keep_prob: 0.8})
    #keep_prob留下的神经元的概率
    if i % 10 == 0: #每训练10批测试一次loss
      xs, ys = driving_data_test.LoadValBatch(batch_size)
      loss_value = loss.eval(feed_dict={model.x:xs, model.y_: ys, model.keep_prob: 1.0})
      #eval函数其实就是一个把代码变成语句的函数，也就是run的意思
      print("Epoch: %d, Step: %d, Loss: %g" % (epoch, epoch * batch_size + i, loss_value))
      output_file.write("Epoch: %d, Step: %d, Loss: %g" % (epoch, epoch * batch_size + i, loss_value))

    # write logs at every iteration
    summary = merged_summary_op.eval(feed_dict={model.x:xs, model.y_: ys, model.keep_prob: 1.0})
    summary_writer.add_summary(summary, epoch * driving_data_test.num_images/batch_size + i)
    #add_summary仅仅是向FileWriter对象的缓存中存放event data

    if i % batch_size == 0:
      if not os.path.exists(LOGDIR):
        os.makedirs(LOGDIR)
      checkpoint_path = os.path.join(LOGDIR, "model.ckpt")
      filename = saver.save(sess, checkpoint_path)
  print("Model saved in file: %s" % filename)

print("Run the command line:\n" \
          "--> tensorboard --logdir=./logs " \
          "\nThen open http://0.0.0.0:6006/ into your web browser")
