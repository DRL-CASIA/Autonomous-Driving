import numpy as np
import tensorflow as tf
from ddpg_networks import LSRActor, LSRCritic

def OU_noise(x, mu, epsilon):
    # Ornstein-Uhlenbeck process
    # dx_t = \theta*(\mu - x_t)dt + \sigma*dW_t
    # Here set \theta = 0.15, \sigma = 0.3
    return epsilon*(0.15*(mu - x) + 0.2*np.random.randn(1))

class Agent(object):
    def __init__(self, sess, state_dim, action_dim,
                 writer=None, lr_a=1e-3, lr_c=1e-4, discount = 0.99):
        self.sess = sess
        self.discount = discount
        self.writer = writer
        self.lr_a = lr_a
        self.lr_c = lr_c
        self.train_count = 0

        self.actor = LSRActor(sess, state_dim, action_dim)
        self.critic = LSRCritic(sess, state_dim, action_dim)
        self.frame = tf.placeholder('float', [1, 210, 280, 3])

        self.pre_ML = -4.0
        self.pre_MR = 0.0
        a_vars = self.actor.get_total_vars()
        c_vars = self.critic.get_total_vars()
        self.saver = tf.train.Saver(var_list=a_vars + c_vars)

    def predict(self, s, epsilon):
        a = self.actor.predict_action(s)
        a += OU_noise(a, 0.0, epsilon)
        q = self.critic.predict_value(s, a)
        return a[0], q[0]

    def cnn_predict_laser(self, image):
        lsr = self.sess.run(self.torcs_net_out, feed_dict={self.frame: image})[0]
        angle = (lsr[0] - 0.5)*1.1
        toMarking_L = (lsr[1] - 1.34445) * 5.6249
        toMarking_M = (lsr[2] - 0.39091) * 6.8752
        toMarking_R = (lsr[3] + 0.34445) * 5.6249
        toMarking_ML = (lsr[4] - 0.98) * 6.25
        toMarking_MR = (lsr[5] - 0.02) * 6.25

        if -toMarking_ML + toMarking_MR < 5.5:
            to_middle = (toMarking_ML + toMarking_MR)/2.0
            self.pre_ML = toMarking_ML
            self.pre_MR = toMarking_MR
        else:
            if -self.pre_ML > self.pre_MR:
                to_middle = (toMarking_L + toMarking_M)/2.0
            else:
                to_middle = (toMarking_R + toMarking_M)/2.0
        to_middle /= 4.0  # half width = 4.0
        angle /= 3.1416
        return [angle, to_middle]

    def cnn_predict_action(self, cnn_lsr):
        a = self.actor.predict_action(cnn_lsr)
        return a[0]

    def train(self, batch, step):
        self.train_count += 1
        if self.train_count == 20000:
            self.train_count = 0
            self.lr_a *= 0.1
            self.lr_c *= 0.1
        s, a, r, s2, term = batch[0], batch[1], batch[2], batch[3], batch[4]
        q2 = self.critic.predict_target_value(s2, self.actor.predict_target_action(s2))
        y = np.zeros(a.shape[0])
        for i in range(a.shape[0]):
            if term[i]:
                y[i] = r[i]
            else:
                y[i] = r[i] + self.discount * q2[i]
        y = np.reshape(y, (-1, 1))

        # update critic
        train_summ = self.critic.train(s, a, y, self.lr_c)
        self._record(train_summ, step)

        # update actor
        a_ = self.actor.predict_action(s)
        cg, cg_summ = self.critic.get_critic_grads(s, a_)
        self._record(cg_summ, step)
        train_summ = self.actor.train(s, cg, self.lr_a)
        self._record(train_summ, step)

    def train_target(self):
        self.actor.train_target()
        self.critic.train_target()

    def _record(self, summary, step):
        self.writer.add_summary(summary, global_step=step)
        self.writer.flush()

    def save(self, path):
        self.saver.save(self.sess, path)

    def restore(self, path):
        self.saver.restore(self.sess, path)
