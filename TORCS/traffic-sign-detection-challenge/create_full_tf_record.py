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

parser = argparse.ArgumentParser()
parser.add_argument('--mode', default='five_categories', type=str, help='TFRecord creation mode, [five_categories|one_category]')
args = parser.parse_args()

def read_tsinghua_dataset(image_dir, label_path):
    tsinghua_train_images_dir = os.path.join(image_dir, 'train')
    tsinghua_val_images_dir = os.path.join(image_dir, 'test')
    tsinghua_train_images_path = utils.read_and_parse_dir(tsinghua_train_images_dir)
    tsinghua_val_images_path = utils.read_and_parse_dir(tsinghua_val_images_dir)
    print('-' * 20)
    print('Tsinghua dataset train samples: ', len(tsinghua_train_images_path)-2)
    print('Tsinghua dataset val samples: ', len(tsinghua_val_images_path)-2)
    with open(label_path, 'r') as f:
        annotations = json.load(f)
    labels = annotations['imgs']
    train_samples_lst = []
    val_samples_lst = []
    for image_path in tsinghua_train_images_path:
        name = image_path.split('/')[-1]  # e.g. 1000.jpg
        idx = name.split('.')[0]  # e.g. 1000
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
        name = image_path.split('/')[-1]  # e.g. 1000.jpg
        idx = name.split('.')[0]  # e.g. 1000
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

def read_offline_challenge_dataset(image_dir, label_dir):
    train_images_dir = os.path.join(image_dir, 'train')
    val_images_dir = os.path.join(image_dir, 'val')
    train_images_paths = utils.read_and_parse_dir(train_images_dir)
    val_images_paths = utils.read_and_parse_dir(val_images_dir)
    label_paths = utils.read_and_parse_dir(label_dir)
    print('-'*25)
    print('Offline Challenge train samples:', len(train_images_paths))
    print('Offline Challenge val samples:', len(val_images_paths))
    print('-' * 12 + 'End' + '-' * 12)

    train_samples_lst = []
    val_samples_lst = []
    for image_path in train_images_paths:
        video_name = image_path.split('/')[-2]
        frame_name = image_path.split('/')[-1]
        frame_name = frame_name.split('.')[0]
        idx = frame_name.split('-')[-1] # 00001, 00002...
        label_path = os.path.join(label_dir, video_name+'-GT.json')
        with open(label_path, 'r') as f:
            annotations = json.load(f)
        target_num = annotations['Frame%sTargetNumber'%idx]
        bboxes = []
        categories = []
        for i in range(target_num):
            target = annotations['Frame%sTarget%05d'%(idx, i)]
            position = target['Position'].strip().split(' ')
            position = [float(e) for e in position]
            bboxes.append({'xmin': position[0], 'ymin': position[1],
                           'xmax': position[0] + position[2],
                           'ymax': position[1] + position[3]})
            categories.append(target['Type'])
        train_samples_lst.append({'path': image_path,
                               'target_num': target_num,
                               'bboxes': bboxes,
                               'categories_text': categories})

    for image_path in val_images_paths:
        video_name = image_path.split('/')[-2]
        frame_name = image_path.split('/')[-1]
        frame_name = frame_name.split('.')[0]
        idx = frame_name.split('-')[-1] # 00001, 00002...
        label_path = os.path.join(label_dir, video_name+'-GT.json')
        with open(label_path, 'r') as f:
            annotations = json.load(f)
        target_num = annotations['Frame%sTargetNumber'%idx]
        bboxes = []
        categories = []
        for i in range(target_num):
            target = annotations['Frame%sTarget%05d'%(idx, i)]
            position = target['Position'].strip().split(' ')
            position = [float(e) for e in position]
            bboxes.append({'xmin': position[0], 'ymin': position[1],
                           'xmax': position[0] + position[2],
                           'ymax': position[1] + position[3]})
            categories.append(target['Type'])
            val_samples_lst.append({'path': image_path,
                               'target_num': target_num,
                               'bboxes': bboxes,
                               'categories_text': categories})
    return train_samples_lst, val_samples_lst


def merge_dataset(dataset_list):
    all_samples = []
    for dataset in dataset_list:
        all_samples += dataset
    random.shuffle(all_samples)
    return all_samples

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
        # Only check the tsinghua dataset category text,
        # since offline dataset is already formatted.
        if args.mode == 'five_categories':
            if 'offline' in sample['path']:
                categories.append(label_map_dict[sample['categories_text'][i]])
                if 'yellow' == sample['categories_text'][i]:
                    categories_text.append('Warning')
                elif 'red' == sample['categories_text'][i]:
                    categories_text.append('Prohibition')
                elif 'blue' == sample['categories_text'][i]:
                    categories_text.append('Illustration')
                elif 'light' == sample['categories_text'][i]:
                    categories_text.append('Light')
                elif 'arrow' == sample['categories_text'][i]:
                    categories_text.append('Arrow')
                else:
                    raise('Unkown cagetories_text: %s' % sample['categories_text'])
            else:
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
                    raise ('Unknow label. File: %s' % sample['path'])
        elif args.mode == 'one_category':
            categories.append(1) # only one category, 1 for traffic sign
            categories_text.append('Sign')
        else:
            raise('Unkown mode, please use --help for details')
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


def main():
    '''Pipeline

    1. Read all tsinghua and offline_challenge dataset.
    2. Split train_ and val_ dataset.
    3. Shuffle and create tf record for single sample
    4. Merge all tf records.
    '''
    # Path config
    if args.mode == 'five_categories':
        label_map_dict = utils.get_label_map_dict('./data/tsd_label_map_detection.pbtxt')
    else:
        label_map_dict = utils.get_label_map_dict('./data/tsd_label_map_detection_one_category.pbtxt')
    tsinghua_image_dir = '/home/lab/datasets/TSD/tsinghua_traffic_sign/images'
    tsinghua_label_path = '/home/lab/datasets/TSD/tsinghua_traffic_sign/labels/annotations.json'
    offline_image_dir = '/home/lab/datasets/TSD/offline_challenge/images'
    offline_label_dir = '/home/lab/datasets/TSD/offline_challenge/labels_json/five_categories'

    tsinghua_train_samples, tsinghua_val_samples = read_tsinghua_dataset(tsinghua_image_dir,
                                                                         tsinghua_label_path)

    offline_train_samples, offline_val_samples = read_offline_challenge_dataset(offline_image_dir,
                                                                                offline_label_dir)

    train_samples = merge_dataset([tsinghua_train_samples, offline_train_samples])
    val_samples = merge_dataset([tsinghua_val_samples, offline_val_samples])

    create_tf_record('./data/full_tfrecord_one_category/train/train.record', train_samples, label_map_dict)
    create_tf_record('./data/full_tfrecord_one_category/val/val.record', val_samples, label_map_dict)

    print('Done')

if __name__ == '__main__':
    main()
