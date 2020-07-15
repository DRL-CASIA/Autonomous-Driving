from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import argparse
import json
import random
import numpy as np
import PIL.Image as Image
import tensorflow as tf
import contextlib2

import utils

def read_additional_dataset(image_dir, label_dir):
    label_path = os.path.join(label_dir, 'labels.json')
    with open(label_path, 'r') as f:
        labels = json.load(f)
    train_samples_lst = []
    for image_path, label in labels.items():
        bbox = label['bbox']
        category = label['category']
        target_num = len(category)
        train_samples_lst.append({'path': image_path,
                                  'target_num': target_num,
                                  'bboxes': bbox,
                                  'category': category})
    return train_samples_lst

def create_tf_record(output_filebase, samples_lst, num_shards=5):
    with contextlib2.ExitStack() as tf_record_close_stack:
        output_tfrecords = utils.open_sharded_output_tfrecords(tf_record_close_stack,
                                                               output_filebase, num_shards)
        for idx, sample in enumerate(samples_lst):
            if idx % 500 == 0:
                print('On image %d of %d' % (idx, len(samples_lst)))
            tf_sample_record = create_tf_sample_record(sample)
            output_shard_idx = idx % num_shards
            output_tfrecords[output_shard_idx].write(tf_sample_record.SerializeToString())


def create_tf_sample_record(sample):
    with tf.gfile.GFile(sample['path'], 'rb') as fid:
        encoding_img = fid.read()
    image = Image.open(sample['path'])
    width, height = image.size

    xmin = []
    ymin = []
    xmax = []
    ymax = []
    categories = []

    for i in range(sample['target_num']):
        xmin.append(sample['bboxes'][i]['xmin'] / width)
        ymin.append(sample['bboxes'][i]['ymin'] / height)
        xmax.append(sample['bboxes'][i]['xmax'] / width)
        ymax.append(sample['bboxes'][i]['ymax'] / height)
        if sample['category'][i] in [1, 2, 3, 4, 5, 6]:
            categories.append(1)
        elif sample['category'][i] in [7, 8, 9, 13]:
            categories.append(2)
        elif sample['category'][i] in [10, 11, 12, 14]:
            categories.append(3)
        elif sample['category'][i] in [15, 16, 17]:
            categories.append(4)
        else:
            categories.append(5)

    example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': utils.int64_feature(height),
        'image/width': utils.int64_feature(width),
        'image/encoded': utils.bytes_feature(encoding_img),
        'image/object/bbox/xmin': utils.float_list_feature(xmin),
        'image/object/bbox/xmax': utils.float_list_feature(xmax),
        'image/object/bbox/ymin': utils.float_list_feature(ymin),
        'image/object/bbox/ymax': utils.float_list_feature(ymax),
        'image/object/class/label': utils.int64_list_feature(categories)
    }))
    return example


def main():
    label_map_dict = utils.get_label_map_dict('./data/tsd_label_map_detection.pbtxt')
    image_dir = '/home/lab/traffic-sign-detection-challenge/data/full_tsd_official_aug_dataset/images'
    label_dir = '/home/lab/traffic-sign-detection-challenge/data/full_tsd_official_aug_dataset/labels'

    train_samples = read_additional_dataset(image_dir, label_dir)
    random.shuffle(train_samples)
    create_tf_record('./data/full_tsd_official_aug_tfrecord/train.record', train_samples)


if __name__ == '__main__':
    main()