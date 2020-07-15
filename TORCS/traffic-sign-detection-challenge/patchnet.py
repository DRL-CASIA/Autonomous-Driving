import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Input
from keras.layers import Conv2D, MaxPooling2D, Activation

def PatchNet(input_shape,
             num_filters=[32, 64],
             filter_size=(3, 3),
             num_fc_units = [128],
             drop_keep_prob = [0.5],
             num_classes=26,
             optimizer = 'rmsprop'):
    # input
    model = Sequential()
    model.add(Conv2D(num_filters[0], filter_size, input_shape=input_shape))
    model.add(Activation('relu'))

    # build conv parts
    for n in num_filters[1:]:
        model.add(Conv2D(n, filter_size))
        model.add(Activation('relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        if len(drop_keep_prob) > 1:
            model.add(Dropout(drop_keep_prob[0]))

    # build fc parts
    model.add(Flatten())
    for n in num_fc_units:
        model.add(Dense(n))
        model.add(Activation('relu'))
        model.add(Dropout(drop_keep_prob[-1]))

    # output
    model.add(Dense(num_classes))
    model.add(Activation('softmax'))

    if optimizer == 'rmsprop':
        opt = keras.optimizers.rmsprop(lr=0.0001, decay=1e-6)
    elif optimizer == 'adam':
        opt = keras.optimizers.adam(lr=0.0001)
    elif optimizer == 'sgd':
        opt = keras.optimizers.sgd(lr=0.0001, momentum=0.9, nesterov=True)
    else:
        raise ValueError('Need to specify the optimizer.')

    model.compile(loss='categorical_crossentropy',
                  optimizer=opt,
                  metrics=['accuracy'])
    model.summary()
    return model
