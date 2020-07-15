'''Utility functions.

Some functions are from tensorflow/model repo.
'''
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    print('Not use ros cv2 already.')
import os
import cv2
from natsort import natsorted
import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from google.protobuf import text_format
from protos import string_int_label_map_pb2

def load_label_map(path):
  """Loads label map proto.

  Args:
    path: path to StringIntLabelMap proto text file.
  Returns:
    a StringIntLabelMapProto
  """
  with tf.gfile.GFile(path, 'r') as fid:
    label_map_string = fid.read()
    label_map = string_int_label_map_pb2.StringIntLabelMap()
    try:
      text_format.Merge(label_map_string, label_map)
    except text_format.ParseError:
      label_map.ParseFromString(label_map_string)
  return label_map
        
def get_label_map_dict(label_map_path,
                       use_display_name=False,
                       fill_in_gaps_and_background=False):
  """Reads a label map and returns a dictionary of label names to id.

  Args:
    label_map_path: path to StringIntLabelMap proto text file.
    use_display_name: whether to use the label map items' display names as keys.
    fill_in_gaps_and_background: whether to fill in gaps and background with
    respect to the id field in the proto. The id: 0 is reserved for the
    'background' class and will be added if it is missing. All other missing
    ids in range(1, max(id)) will be added with a dummy class name
    ("class_<id>") if they are missing.

  Returns:
    A dictionary mapping label names to id.

  Raises:
    ValueError: if fill_in_gaps_and_background and label_map has non-integer or
    negative values.
  """
  label_map = load_label_map(label_map_path)
  label_map_dict = {}
  for item in label_map.item:
    if use_display_name:
      label_map_dict[item.display_name] = item.id
    else:
      label_map_dict[item.name] = item.id

  if fill_in_gaps_and_background:
    values = set(label_map_dict.values())

    if 0 not in values:
      label_map_dict['background'] = 0
    if not all(isinstance(value, int) for value in values):
      raise ValueError('The values in label map must be integers in order to'
                       'fill_in_gaps_and_background.')
    if not all(value >= 0 for value in values):
      raise ValueError('The values in the label map must be positive.')

    if len(values) != max(values) + 1:
      # there are gaps in the labels, fill in gaps.
      for value in range(1, max(values)):
        if value not in values:
          label_map_dict['class_' + str(value)] = value
  
  return label_map_dict

def open_sharded_output_tfrecords(exit_stack, base_path, num_shards):
  """Opens all TFRecord shards for writing and adds them to an exit stack.

  Args:
    exit_stack: A context2.ExitStack used to automatically closed the TFRecords
      opened in this function.
    base_path: The base path for all shards
    num_shards: The number of shards

  Returns:
    The list of opened TFRecords. Position k in the list corresponds to shard k.
  """
  tf_record_output_filenames = [
      '{}-{:05d}-of-{:05d}'.format(base_path, idx, num_shards)
      for idx in range(num_shards)
  ]

  tfrecords = [
      exit_stack.enter_context(tf.python_io.TFRecordWriter(file_name))
      for file_name in tf_record_output_filenames
  ]

  return tfrecords

def read_and_parse_dir(src_path):
    '''Recursively read all filenames in dirs and subdirs
    Return full path files list, in which each sublist
    includes file path in that directory.
    '''
    samples = []
    for root, dirs, files in os.walk(src_path):
        for name in natsorted(files):
            samples.append(os.path.join(root, name))
    samples_list = []
    dir_name = None
    dir_samples = []
    for sample in samples:
        name = sample.split('/')[-2]
        if not dir_name == name:
            if len(dir_samples) > 0:
                samples_list.append(dir_samples)
            dir_samples = [sample]
            dir_name = name
        else:
            dir_samples.append(sample)
    samples_list.append(dir_samples)
    return samples_list

def swamp_kv(dict):
    keys = []
    vals = []
    for k, v in dict.items():
        keys.append(k)
        vals.append(v)
    tmp_dict = {}
    for k, v in zip(vals, keys):
        tmp_dict[k] = v
    return tmp_dict

def wash_with_threshold(raw, thresh):
    raw = [raw]  # dir raw to all raw
    clean_results = []
    for dir_results in raw:
        # directory level
        clean_dir_results = []
        for frame_results_dict in dir_results:
            # frame level
            bboxes = frame_results_dict['bboxes']
            scores = frame_results_dict['scores']
            bboxes = np.asarray(bboxes)
            scores = np.asarray(scores)
            w, h = frame_results_dict['metadata']
            cutpoint = np.argmax(scores < thresh)
            th_bboxes = bboxes[:cutpoint]
            for i, bbox in enumerate(th_bboxes):
                xmin = bbox[1] * w
                ymin = bbox[0] * h
                width = (bbox[3] - bbox[1]) * w
                height = (bbox[2] - bbox[0]) * h
                th_bboxes[i] = [xmin, ymin, width, height]
            th_bboxes = np.asarray(th_bboxes, dtype=int)
            clean_dir_results.append(th_bboxes.tolist())
        clean_results.append(clean_dir_results)
    return clean_results[0]

def show_bbox_on_image(img, bbox, path):
    # print(score)
    name = path.split('/')[-1]
    save_base = 'tsd-2018/posnet_show_aug'
    save_path = os.path.join(save_base, 'det_'+name)
    fig, ax = plt.subplots()
    plt.axis('off')
    ax.imshow(img)
    for bb in bbox:
        rect = patches.Rectangle((bb[0], bb[1]), bb[2], bb[3], edgecolor='r', facecolor='none')
        ax.add_patch(rect)
    plt.savefig(save_path)

def recursive_parse_xml_to_dict(xml):
  """Recursively parses XML contents to python dict.

  We assume that `object` tags are the only ones that can appear
  multiple times at the same level of a tree.

  Args:
    xml: xml tree obtained by parsing XML file contents using lxml.etree

  Returns:
    Python dictionary holding XML contents.
  """
  if not xml:
    return {xml.tag: xml.text}
  result = {}
  for child in xml:
    child_result = recursive_parse_xml_to_dict(child)
    if child.tag != 'object':
      result[child.tag] = child_result[child.tag]
    else:
      if child.tag not in result:
        result[child.tag] = []
      result[child.tag].append(child_result[child.tag])
  return {xml.tag: result}

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])

def to_clahe(arr):
    # Clahe (Contrast limit adaptive histogram equalizationg) for
    # gray images.
    arr_ = np.asarray(arr, dtype=np.uint8)
    clahe = cv2.createCLAHE(tileGridSize=(5, 5), clipLimit=10.0)
    clahe_arr = np.asarray(list(map(lambda img: clahe.apply(img), arr_)))
    return clahe_arr

def int64_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))

def int64_list_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=value))

def bytes_feature(value):
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))

def float_list_feature(value):
  return tf.train.Feature(float_list=tf.train.FloatList(value=value))

def bytes_list_feature(value):
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=value))

