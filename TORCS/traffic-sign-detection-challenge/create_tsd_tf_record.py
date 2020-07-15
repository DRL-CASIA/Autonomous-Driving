'''Code for generating the TFRecord dataset for TSD task.

Adopted from tensorflow/models repo.
'''

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import json
import numpy as np
import PIL.Image as Image
import tensorflow as tf
import contextlib2

import utils

tf.app.flags.DEFINE_string('data_dir', '', 'Location of root directory for the '
                           'data. Folder structure is assumed to be:'
                           '<data_dir>/training/label_2 (annotations) and'
                           '<data_dir>/data_object_image_2/training/image_2'
                           '(images).')
tf.app.flags.DEFINE_string('output_path', '', 'Path to which TFRecord files'
                           'will be written. The TFRecord with the training set'
                           'will be located at: <output_path>_train.tfrecord.'
                           'And the TFRecord with the validation set will be'
                           'located at: <output_path>_val.tfrecord')
tf.app.flags.DEFINE_string('classes_to_use', 'car,pedestrian,dontcare',
                           'Comma separated list of class names that will be'
                           'used. Adding the dontcare class will remove all'
                           'bboxs in the dontcare regions.')
tf.app.flags.DEFINE_string('label_map_path', 'data/kitti_label_map.pbtxt',
                           'Path to label map proto.')
tf.app.flags.DEFINE_integer('validation_set_size', '500', 'Number of images to'
                            'be used as a validation set.')
FLAGS = tf.app.flags.FLAGS

def read_tsinghua_dataset(image_dir, label_path):
  tsinghua_train_images_dir = os.path.join(image_dir, 'train_')
  tsinghua_val_images_dir = os.path.join(image_dir, 'test')
  tsinghua_train_images_path = utils.read_and_parse_dir(tsinghua_train_images_dir)
  tsinghua_val_images_path = utils.read_and_parse_dir(tsinghua_val_images_dir)
  print('-'*25)
  print('Tsinghua dataset train_ samples: ', len(tsinghua_train_images_path))
  print('Tsinghua dataset val_ samples: ', len(tsinghua_val_images_path))
  print('-'*10 + 'End' + '-'*10)
  with open(label_path, 'r') as f:
    annotations = json.load(f)
  labels = annotations['imgs']
  train_samples_lst = []
  val_samples_lst = []
  for image_path in tsinghua_train_images_path:
    name = image_path.split('/')[-1] # e.g. 1000.jpg
    idx = name.split('.')[0] # e.g. 1000
    if idx in ['ids', 'Thumbs']:
      continue
    label = labels[idx]
    targets = label['objects']
    target_num = len(targets)
    bboxes = []
    categories = []
    for target in targets:
      bboxes.append(target['bbox'])
      categories.append(target['category'])
    train_samples_lst.append({'path': image_path,
                               'target_num': target_num,
                               'bboxes': bboxes,
                               'categories_text': categories})
  for image_path in tsinghua_val_images_path:
    name = image_path.split('/')[-1] # e.g. 1000.jpg
    idx = name.split('.')[0] # e.g. 1000
    if idx in ['ids', 'Thumbs']:
      continue
    label = labels[str(idx)]
    targets = label['objects']
    target_num = len(targets)
    bboxes = []
    categories = []
    for target in targets:
      bboxes.append(target['bbox'])
      categories.append(target['category'])
    val_samples_lst.append({'path': image_path,
                       'target_num': target_num,
                       'bboxes': bboxes,
                       'categories_text': categories})
  return train_samples_lst, val_samples_lst

def create_tf_sample_record(sample, label_map_dict):
  with tf.gfile.GFile(sample['path'], 'rb') as fid:
    encoding_img = fid.read()
  image = Image.open(sample['path'])
  width, height = image.size
  
  xmin = []
  ymin = []
  xmax = []
  ymax = []
  categories = []
  categories_text = []

  for i in range(sample['target_num']):
    xmin.append(sample['bboxes'][i]['xmin'] / width)
    ymin.append(sample['bboxes'][i]['ymin'] / height)
    xmax.append(sample['bboxes'][i]['xmax'] / width)
    ymax.append(sample['bboxes'][i]['ymax'] / height)
    if 'w' in sample['categories_text'][i] and 'p' not in sample['categories_text'][i]:
      categories.append(label_map_dict['yellow'])
      categories_text.append('Warning')
    elif 'p' in sample['categories_text'][i] and 'ip' not in sample['categories_text'][i]:
      categories.append(label_map_dict['red'])
      categories_text.append('Prohibition')
    elif 'i' in sample['categories_text'][i]:
      categories.append(label_map_dict['blue'])
      categories_text.append('Illustration')
    else:
      raise('Unknow label. File: %s' % sample['path'])
  categories_text = [e.encode('utf-8') for e in categories_text]

  example = tf.train.Example(features=tf.train.Features(feature={
    'image/height': utils.int64_feature(height),
    'image/width': utils.int64_feature(width),
    'image/encoded': utils.bytes_feature(encoding_img),
    'image/object/bbox/xmin': utils.float_list_feature(xmin),
    'image/object/bbox/xmax': utils.float_list_feature(xmax),
    'image/object/bbox/ymin': utils.float_list_feature(ymin),
    'image/object/bbox/ymax': utils.float_list_feature(ymax),
    'image/object/class/text': utils.bytes_list_feature(categories_text),
    'image/object/class/label': utils.int64_list_feature(categories)
  }))
  return example

def create_tf_record(output_filebase, samples_lst, label_map_dict, num_shards=5):
  with contextlib2.ExitStack() as tf_record_close_stack:
    output_tfrecords = utils.open_sharded_output_tfrecords(tf_record_close_stack,
                                                           output_filebase, num_shards)
    for idx, sample in enumerate(samples_lst):
      if idx % 500 == 0:
        print('On image %d of %d' % (idx, len(samples_lst)))
      tf_sample_record = create_tf_sample_record(sample, label_map_dict)
      output_shard_idx = idx % num_shards
      output_tfrecords[output_shard_idx].write(tf_sample_record.SerializeToString())

def main(_):
  '''Pipeline
  1. Read all image path as in tsinghua dataset and tsd dataset
  2. Divide and select the train_ and val_ dataset image path
  3. Create tf record for single sample
  4. Merge all tf records
  '''
  # label_map_dict = get_label_map_dict(FLAGS.label_map_path)
  label_map_dict = utils.get_label_map_dict('./tsd_label_map_detection.pbtxt')
  tsinghua_data_dir = FLAGS.data_dir.join('tsinghua_traffic_sign')
  tsinghua_data_dir = '/home/lab/datasets/TSD/tsinghua_traffic_sign'
  tsinghua_images_dir = os.path.join(tsinghua_data_dir, 'images')
  tsinghua_label_path = '/home/lab/datasets/TSD/tsinghua_traffic_sign/labels/annotations.json'
  tsinghua_train_samples, tsinghua_val_samples = read_tsinghua_dataset(tsinghua_images_dir, 
                                                                       tsinghua_label_path)
  create_tf_record('./data/train_tfrecord/tsinghua_train.record', tsinghua_train_samples, label_map_dict)
  create_tf_record('./data/val_tfrecord/tsinghua_val.record', tsinghua_val_samples, label_map_dict, 2)
  print('Done')

if __name__ == '__main__':
  tf.app.run()