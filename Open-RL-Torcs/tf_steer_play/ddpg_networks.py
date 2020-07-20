import numpy as np
import tensorflow as tf

def _fc_variables(shape, small_init=False):
    input_shape = shape[0]
    output_shape = shape[1]
    d = 1.0/np.sqrt(input_shape)
    bias_shape = [output_shape]
    if small_init:
        weight = tf.Variable(tf.random_uniform(shape, minval=-3e-4, maxval=3e-4))
        bias = tf.Variable(tf.random_uniform(bias_shape, minval=-3e-4, maxval=3e-4))
    else:
        weight = tf.Variable(tf.random_uniform(shape, minval=-d, maxval=d))
        bias = tf.Variable(tf.random_uniform(bias_shape, minval=-d, maxval=d))
    return weight, bias

class LSRActor(object):
    def __init__(self, sess, state_dim, action_dim,
                 tau = 0.001, name='actor'):
        self.sess = sess
        self.tau = tau
        self.name = name
        self.lr = tf.placeholder('float')

        with tf.variable_scope(self.name + '_online'):
            self.s, self.a = self.create_network(state_dim, action_dim)

        with tf.variable_scope(self.name + '_target'):
            self.target_s, self.target_a = self.create_network(state_dim, action_dim)

        self.vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=self.name + '_online')
        self.target_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES,
                                             scope=self.name + '_target')

        self.train_target_ops = [self.target_vars[i].assign(tf.multiply(tau, self.vars[i]) +
                                                            tf.multiply(1 - tau, self.target_vars[i]))
                                                            for i in range(len(self.vars))]

        self.optimize_op, self.v_norm, self.g_norm = self.prepare_loss(action_dim)
        vn_summ = tf.summary.scalar('actor/vars_norm', self.v_norm)
        gn_summ = tf.summary.scalar('actor/grads_norm', self.g_norm)
        self.train_summ = tf.summary.merge([vn_summ, gn_summ])

    def create_network(self, state_dim, action_dim):
        W_fc1, b_fc1 = _fc_variables([state_dim, 150])
        W_fc2, b_fc2 = _fc_variables([150, 100])
        W_fc3, b_fc3 = _fc_variables([100, action_dim], small_init=True)

        s = tf.placeholder('float', [None, state_dim])
        x = tf.nn.relu(tf.matmul(s, W_fc1) + b_fc1)
        x = tf.nn.relu(tf.matmul(x, W_fc2) + b_fc2)
        a = tf.nn.tanh(tf.matmul(x, W_fc3) + b_fc3)
        return s, a

    def prepare_loss(self, action_dim):
        self.critic_grads = tf.placeholder('float', [None, action_dim])
        grads = tf.gradients(self.a, self.vars, -self.critic_grads)
        # grads, grads_norm = tf.clip_by_global_norm(grads, 5.0)
        gv = zip(grads, self.vars)
        optimize_op = tf.train.AdamOptimizer(self.lr).apply_gradients(gv)
        vars_norm = tf.global_norm(self.vars)
        grads_norm = tf.global_norm(grads)
        return optimize_op, vars_norm, grads_norm

    def predict_action(self, s):
        a = self.sess.run(self.a, feed_dict={self.s: s})
        return a

    def predict_target_action(self, s):
        a = self.sess.run(self.target_a, feed_dict={self.target_s: s})
        return a

    def train(self, s, cg, lr):
        summ, _ = self.sess.run([self.train_summ, self.optimize_op], feed_dict={
            self.s: s,
            self.critic_grads: cg,
            self.lr: lr
        })
        return summ

    def train_target(self):
        self.sess.run(self.train_target_ops)

    def get_total_vars(self):
        return self.vars + self.target_vars


class LSRCritic(object):
    def __init__(self, sess, state_dim, action_dim,
                 tau=0.001, l2_decay=0.01, name='critic'):
        self.sess = sess
        self.tau = tau
        self.name = name
        self.lr = tf.placeholder('float')

        with tf.variable_scope(self.name + '_online'):
            self.s, self.a, self.q = self.create_network(state_dim, action_dim)

        with tf.variable_scope(self.name + '_target'):
            self.target_s, self.target_a, self.target_q = self.create_network(state_dim, action_dim)

        self.vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=self.name + '_online')
        self.target_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES,
                                             scope=self.name + '_target')
        self.train_target_ops = [self.target_vars[i].assign(tf.multiply(tau, self.vars[i]) +
                                                            tf.multiply(1 - tau, self.target_vars[i]))
                                                            for i in range(len(self.vars))]

        self.optimize_op, self.loss, self.v_norm, self.g_norm = self.prepare_loss(l2_decay)
        self.critic_grads = tf.gradients(self.q, self.a)
        cg_norm = tf.global_norm(self.critic_grads)
        self.cg_summ = tf.summary.scalar('critic/cg_norm', cg_norm)
        vn_summ = tf.summary.scalar('critic/vars_norm', self.v_norm)
        gn_summ = tf.summary.scalar('critic/grads_norm', self.g_norm)
        loss_summ = tf.summary.scalar('critic/loss', self.loss)
        self.train_summ = tf.summary.merge([vn_summ, gn_summ, loss_summ])

    def create_network(self, state_dim, action_dim):
        W_fc1, b_fc1 = _fc_variables([state_dim, 150])
        W_fc2, b_fc2 = _fc_variables([150, 100])
        W_fc3, b_fc3 = _fc_variables([action_dim, 100])
        W_fc4, b_fc4 = _fc_variables([200, 100])
        W_fc5, b_fc5 = _fc_variables([100, 1], small_init=True)

        s = tf.placeholder('float', [None, state_dim])
        a = tf.placeholder('float', [None, action_dim])
        x = tf.nn.relu(tf.matmul(s, W_fc1) + b_fc1)
        x1 = tf.nn.relu(tf.matmul(x, W_fc2) + b_fc2)
        x2 = tf.nn.relu(tf.matmul(a, W_fc3) + b_fc3)
        x = tf.concat([x1, x2], axis=1)
        x = tf.nn.relu(tf.matmul(x, W_fc4) + b_fc4)
        q = tf.matmul(x, W_fc5) + b_fc5
        return s, a, q

    def prepare_loss(self, l2_decay):
        self.y = tf.placeholder('float', [None, 1])
        loss = tf.reduce_mean(tf.square(self.q - self.y))
        reg_loss = l2_decay * tf.reduce_sum([tf.reduce_sum(tf.square(x)) for x in self.vars])
        loss += reg_loss
        grads = tf.gradients(loss, self.vars)
        gv = zip(grads, self.vars)
        optimize_op = tf.train.AdamOptimizer(self.lr).apply_gradients(gv)
        vars_norm = tf.global_norm(self.vars)
        grads_norm = tf.global_norm(grads)
        return optimize_op, loss, vars_norm, grads_norm

    def predict_value(self, s, a):
        q = self.sess.run(self.q, feed_dict={self.s: s,
                                             self.a: a})
        return q

    def predict_target_value(self, s, a):
        q = self.sess.run(self.target_q, feed_dict={self.target_s: s,
                                                    self.target_a: a})
        return q

    def train(self, s, a, y, lr):
        summ, _ = self.sess.run([self.train_summ, self.optimize_op], feed_dict={
            self.s: s,
            self.a: a,
            self.y: y,
            self.lr: lr
        })
        return summ

    def get_critic_grads(self, s, a):
        cg, cg_summ = self.sess.run([self.critic_grads, self.cg_summ], feed_dict={self.s: s,
                                                                                  self.a: a})
        return cg[0], cg_summ

    def train_target(self):
        self.sess.run(self.train_target_ops)

    def get_total_vars(self):
        return self.vars + self.target_vars
