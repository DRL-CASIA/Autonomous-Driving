import numpy as np
from keras.layers import Input, Conv2D, Flatten, Activation
from keras.layers import MaxPool2D, Dense, Lambda, Dropout
from keras.models import Model
import tensorflow as tf
import keras.backend as K
from keras.engine.topology import Layer


def lrn(x):
    return tf.nn.local_response_normalization(x, depth_radius=2, alpha=2e-5, beta=0.75, bias=1.0)

class LRN(Layer):
    def __init__(self, alpha=0.0001,k=1,beta=0.75,n=5, **kwargs):
        self.alpha = alpha
        self.k = k
        self.beta = beta
        self.n = n
        super(LRN, self).__init__(**kwargs)

    def call(self, inputs, **kwargs):
        N, H ,W, C = K.int_shape(inputs)
        half_n = self.n // 2  # half the local region
        input_sqr = K.square(inputs)
        extra_channels = K.zeros((N, H, W, C+2*half_n,))
        input_sqrt = K.concatenate([extra_channels[:,:,:,:half_n],
                                    extra_channels[:,:,:,half_n+C,]], axis=3)
        scale = self.k
        norm_alpha = self.alpha / self.n
        for i in range(self.n):
            scale += norm_alpha * input_sqrt[:,:,:,i:i+C]
        scale = scale ** self.beta
        inputs = inputs / scale
        return inputs

    def get_config(self):
        config = {"alpha": self.alpha,
                  "k": self.k,
                  "beta": self.beta,
                  "n": self.n}
        base_config = super(LRN, self).get_config()
        return dict(list(base_config.items()) + list(config.items()))


def conv_output_length(input_length, filter_size,
                       padding, stride, dilation=1):
    """Determines output length of a convolution given input length.

    # Arguments
        input_length: integer.
        filter_size: integer.
        padding: one of "same", "valid", "full".
        stride: integer.
        dilation: dilation rate, integer.

    # Returns
        The output length (integer).
    """
    if input_length is None:
        return None
    assert padding in {'same', 'valid', 'full', 'causal'}
    dilated_filter_size = filter_size + (filter_size - 1) * (dilation - 1)
    if padding == 'same':
        output_length = input_length
    elif padding == 'valid':
        output_length = input_length - dilated_filter_size + 1
    elif padding == 'causal':
        output_length = input_length
    elif padding == 'full':
        output_length = input_length + dilated_filter_size - 1
    return (output_length + stride - 1) // stride


class Conv2D_groups(Layer):
    # rewrite by reference to keras Conv2D implementation.
    def __init__(self, num_kernels, kernel_size, strides=(1, 1),
                 padding='valid', activation='relu', groups=1, **kwargs):
        self.num_kernels = num_kernels
        self.kernel_size = kernel_size
        self.strides = strides
        self.padding = padding
        self.activation = activation
        self.groups = groups
        self.dilation_rate = (1, 1)
        super(Conv2D_groups, self).__init__(**kwargs)

    def build(self, input_shape):
        input_dim = input_shape[3]
        kernel_shape = self.kernel_size + (input_dim / self.groups, self.num_kernels)
        print 'input_dim: {}, kernel_shape: {}'.format(input_dim, kernel_shape)

        self.kernel = self.add_weight(shape=kernel_shape,
                                      initializer='glorot_uniform',
                                      name='kernel')
        self.bias = self.add_weight(shape=(self.num_kernels,),
                                    initializer='uniform',
                                    name='bias')
        self.built = True

    def call(self, inputs):
        if self.groups == 1:
            outputs = K.conv2d(inputs, self.kernel, strides=self.strides,
                               padding=self.padding, data_format='channels_last',
                               dilation_rate=(1, 1))
            outputs = K.bias_add(outputs, self.bias, data_format='channels_last')
            if self.activation is not None:
                return Activation(self.activation)(outputs)
            return outputs
        else:
            input_groups = tf.split(3, self.groups, inputs)
            kernel_groups = tf.split(3, self.groups, self.kernel)
            outputs = [K.conv2d(input, kernel, strides=self.strides,
                                padding=self.padding, data_format='channels_last',
                                dilation_rate=(1, 1)) for input, kernel in zip(input_groups, kernel_groups)]
            outputs = tf.concat(3, outputs)
            outputs = K.bias_add(outputs, self.bias, data_format='channels_last')
            if self.activation is not None:
                return Activation(self.activation)(outputs)
            return outputs

    def compute_output_shape(self, input_shape):
        space = input_shape[1:-1]
        new_space = []
        for i in range(len(space)):
            new_dim = conv_output_length(space[i], self.kernel_size[i],
                                         padding=self.padding, stride=self.strides[i],
                                         dilation=self.dilation_rate[i])
            new_space.append(new_dim)
        return (input_shape[0],) + tuple(new_space) + (self.num_kernels,)



def torcsnet(input_shape):
    input = Input(shape=input_shape)
    x = Conv2D(96, (11, 11), strides=(4, 4), padding='valid', activation='relu', name='conv1')(input)
    x = MaxPool2D((3, 3), strides=(2, 2), padding='same', name='pool1')(x)
    x = Lambda(lrn, name='norm1')(x)
    # x = LRN()(x)
    x = Conv2D_groups(256, (5, 5), strides=(1, 1), padding='same', activation='relu', groups=2, name='conv2')(x)
    x = MaxPool2D((3, 3), strides=(2, 2), padding='same', name='pool2')(x)
    x = Lambda(lrn, name='norm2')(x)
    # x = LRN()(x)
    x = Conv2D(384, (3, 3), activation='relu', padding='same', name='conv3')(x)
    x = Conv2D_groups(384, (3, 3), activation='relu', padding='same', groups=2, name='conv4')(x)
    x = Conv2D_groups(256, (3, 3), activation='relu', padding='same', groups=2, name='conv5')(x)
    x = MaxPool2D((3, 3), (2, 2), name='pool5')(x)
    x = Flatten()(x)
    x = Dense(4096, activation='relu', name='fc6')(x)
    x = Dropout(0.5)(x)
    x = Dense(4096, activation='relu', name='fc7')(x)
    x = Dropout(0.5)(x)
    x = Dense(256, activation='relu', name='fc8')(x)
    x = Dropout(0.5)(x)
    out = Dense(8, activation='relu', name='fc9')(x)
    model = Model(input, out)
    return model


net = torcsnet((210, 280, 3))

