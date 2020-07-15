from keras.layers import Input, Conv2D, Flatten, Activation
from keras.layers import MaxPool2D, Dense, Lambda, Dropout
from keras.models import Model
import tensorflow as tf
import keras.backend as K
from keras import optimizers
from keras import regularizers, initializers
from keras.engine.topology import Layer

def lrn(x):
    return tf.nn.local_response_normalization(x, depth_radius=2, alpha=2e-5, beta=0.75, bias=1.0)

def conv_output_length(input_length, filter_size,
                       padding, stride, dilation=1):
    # from keras source code
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
    def __init__(self,
                 num_kernels,
                 kernel_size,
                 strides=(1, 1),
                 padding='valid',
                 activation='relu',
                 kernel_initializer='glorot_uniform',
                 bias_initializer='zeros',
                 kernel_regularizer=None,
                 groups=1,
                 **kwargs):
        self.num_kernels = num_kernels
        self.kernel_size = kernel_size
        self.strides = strides
        self.padding = padding
        self.activation = activation
        self.kernel_initializer = initializers.get(kernel_initializer)
        self.bias_initializer = initializers.get(bias_initializer)
        self.kernel_regularizer = regularizers.get(kernel_regularizer)
        self.groups = groups
        self.dilation_rate = (1, 1)
        super(Conv2D_groups, self).__init__(**kwargs)

    def build(self, input_shape):
        input_dim = input_shape[3]
        kernel_shape = self.kernel_size + (input_dim / self.groups, self.num_kernels)
        # print 'input_dim: {}, kernel_shape: {}'.format(input_dim, kernel_shape)

        self.kernel = self.add_weight(shape=kernel_shape,
                                      initializer=self.kernel_initializer,
                                      name='kernel',
                                      regularizer=self.kernel_regularizer)
        self.bias = self.add_weight(shape=(self.num_kernels,),
                                    initializer=self.bias_initializer,
                                    name='bias') # no regularizer for bias
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
            # note for tf1.3, should be
            input_groups = tf.split(inputs, self.groups, 3)
            kernel_groups = tf.split(self.kernel, self.groups, 3)
            # input_groups = tf.split(3, self.groups, inputs)
            # kernel_groups = tf.split(3, self.groups, self.kernel)
            outputs = [K.conv2d(input, kernel, strides=self.strides,
                                padding=self.padding, data_format='channels_last',
                                dilation_rate=(1, 1)) for input, kernel in zip(input_groups, kernel_groups)]
            # note for tf1.3, should be
            outputs = tf.concat(outputs, 3)
            # outputs = tf.concat(3, outputs)
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

weight_decay = 0.0001
def torcsnet(input_shape, task, verbose=False):
    input = Input(shape=input_shape)
    # Conv1
    x = Conv2D(96, (11, 11), strides=(4, 4),
               kernel_regularizer=regularizers.l2(weight_decay),
               padding='valid', activation='relu', name='conv1')(input)
    x = MaxPool2D((3, 3), strides=(2, 2), padding='same', name='pool1')(x)
    x = Lambda(lrn, name='norm1')(x)

    # Conv2
    x = Conv2D_groups(256, (5, 5), strides=(1, 1), padding='same',
                      kernel_regularizer=regularizers.l2(weight_decay),
                      activation='relu', groups=2, name='conv2')(x)
    x = MaxPool2D((3, 3), strides=(2, 2), padding='same', name='pool2')(x)
    x = Lambda(lrn, name='norm2')(x)

    # Conv3
    x = Conv2D(384, (3, 3), activation='relu', padding='same',
               kernel_regularizer=regularizers.l2(weight_decay),
               name='conv3')(x)

    # Conv4
    x = Conv2D_groups(384, (3, 3), activation='relu', padding='same',
                      kernel_regularizer=regularizers.l2(weight_decay),
                      groups=2, name='conv4')(x)

    # Conv5
    x = Conv2D_groups(256, (3, 3), activation='relu', padding='same',
                      kernel_regularizer=regularizers.l2(weight_decay),
                      groups=2, name='conv5')(x)
    x = MaxPool2D((3, 3), (2, 2), name='pool5')(x)
    x = Flatten()(x)

    # FC6
    x = Dense(4096, activation='relu',
              kernel_regularizer=regularizers.l2(weight_decay), name='fc6')(x)
    x = Dropout(0.5)(x)

    # FC7
    x = Dense(4096, activation='relu',
              kernel_regularizer=regularizers.l2(weight_decay), name='fc7')(x)
    x = Dropout(0.5)(x)

    # FC8
    loss_weights = None
    if task == 'type':
        x = Dense(256, activation='relu',
                    kernel_regularizer=regularizers.l2(weight_decay), name='fc9_type')(x)
        out = Dense(3, activation='softmax',
                    kernel_regularizer=regularizers.l2(weight_decay), name='type')(x)
        loss = 'categorical_crossentropy'
    elif task == 'angle':
        x = Dense(256, activation='relu', 
                    kernel_regularizer=regularizers.l2(weight_decay), name='fc9_angle')(x)
        out = Dense(1, activation='sigmoid',
                    kernel_regularizer=regularizers.l2(weight_decay), name='angle')(x)
        loss = 'mean_absolute_error'
    elif task == 'distance':
        x = Dense(256, activation='relu', 
                    kernel_regularizer=regularizers.l2(weight_decay), name='fc9_distance')(x)
        out = Dense(5, activation='sigmoid',
                    kernel_regularizer=regularizers.l2(weight_decay), name='distance')(x)
        loss = 'mean_absolute_error'
    elif task == 'mtl':
        x1_ = Dense(256, activation='relu',
                    kernel_regularizer=regularizers.l2(weight_decay), name='fc9_type')(x)
        x1 = Dense(3, activation='softmax',
                    kernel_regularizer=regularizers.l2(weight_decay), name='type')(x1_)
        x2_ = Dense(256, activation='relu', 
                    kernel_regularizer=regularizers.l2(weight_decay), name='fc9_angle')(x)
        x2 = Dense(1, activation='sigmoid',
                    kernel_regularizer=regularizers.l2(weight_decay), name='angle')(x2_)
        x3_ = Dense(256, activation='relu', 
                    kernel_regularizer=regularizers.l2(weight_decay), name='fc9_distance')(x)
        x3 = Dense(5, activation='sigmoid',
                    kernel_regularizer=regularizers.l2(weight_decay), name='distance')(x3_)
        out = [x1, x2, x3]

    if task == 'mtl':
        loss = dict(type='categorical_crossentropy',
                    angle='mean_absolute_error',
                    distance='mean_absolute_error')
        loss_weights = dict(type=1./3,
                            angle=1./3,
                            distance=1./3,)

    model = Model(input, out, name='torcsnet')
    sgd = optimizers.SGD(lr=0.01, momentum=0.9, nesterov=True)
    metrics = {'type': 'accuracy'}
    model.compile(optimizer=sgd, loss=loss,
                  metrics=metrics, loss_weights=loss_weights)
    if verbose:
        model.summary()
    return model