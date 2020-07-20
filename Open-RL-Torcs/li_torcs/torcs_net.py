from kaffe.tensorflow import Network

class TORCSNet(Network):
    def setup(self):
        (self.feed('data')
             .conv(11, 11, 96, 4, 4, padding='VALID', name='conv1')
             .max_pool(3, 3, 2, 2, name='pool1')
             .lrn(2, 2e-05, 0.75, name='norm1')
             .conv(5, 5, 256, 1, 1, group=2, name='conv2')
             .max_pool(3, 3, 2, 2, padding='SAME', name='pool2')
             .lrn(2, 2e-05, 0.75, name='norm2')
             .conv(3, 3, 384, 1, 1, name='conv3')
             .conv(3, 3, 384, 1, 1, group=2, name='conv4')
             .conv(3, 3, 256, 1, 1, group=2, name='conv5')
             .max_pool(3, 3, 2, 2, padding='VALID', name='pool5')
             .fc(4096, name='fc6')
             .dropout(0.5, name='drop6')
             .fc(4096, name='fc7')
             .dropout(0.5, name='drop7')
             .fc(256, name='fc8')
             .dropout(0.5, name='drop8')
             .fc(8, relu=False, name='fc9iR')
             .sigmoid(name='prob'))