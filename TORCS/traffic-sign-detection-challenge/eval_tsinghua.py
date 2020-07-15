'''Evaluate the model trained ONLY on tsinghua dataset.
'''
import os
import json
import numpy as np
from tqdm import tqdm
from PIL import Image as Image
from PIL import ImageDraw
from PIL import ImageFont
import tensorflow as tf

def pil2arr(x):
    return np.asarray(x)

def arr2pil(x):
    return Image.fromarray(x)

# Read validation samples path.
base_dir = '/home/lab/datasets/TSD/tsinghua_traffic_sign/images/test'
label_path = 'tsinghua_annotations_clean.json'
with open(label_path, 'r') as f:
    labels = json.load(f)
test_dict = []
for k, v in labels.items():
    mode = v['mode']
    if mode == 'test':
        test_dict.append(k)
test_dict = [os.path.join(base_dir, e+'.jpg') for e in test_dict]

ckpt_path = '/home/lab/traffic-sign-detection-challenge/pretrained_models/faster_rcnn_resnet101_kitti_2018_01_28/frozen_inference_graph.pb'
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(ckpt_path, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

raw_results = []
with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        for path in tqdm(test_dict, ncols=64):
            image = Image.open(path)
            w, h = image.size
            image_np = pil2arr(image)
            image_np_expanded = np.expand_dims(image_np, axis=0)
            (bboxes, scores, cls, num) = sess.run([detection_boxes, detection_scores,
                                                    detection_classes, num_detections],
                                                    feed_dict={image_tensor: image_np_expanded})
            bboxes = bboxes.squeeze()
            scores = scores.squeeze()
            cls = cls.squeeze()
            cutpoint = np.argmax(scores < 0.5)
            bboxes = bboxes[:cutpoint]
            scores = scores[:cutpoint]
            cls = cls[:cutpoint]
            raw_results.append({'path': path,
                                'bbox': bboxes.tolist(),
                                'score': scores.tolist(),
                                'category': cls.tolist(),
                                'metadata': (w, h)})
# Save the detection results.
with open('tsinghua_detection_result/raw_detection_result.json', 'w') as f:
    json.dump({'test_result': raw_results}, f)
print('Raw detection results saved.')

# thresh = 0.5
with open('tsinghua_detection_result/raw_detection_result.json', 'r') as f:
    raw_results = json.load(f)
raw_results = raw_results['test_result']
save_dir = './tsinghua_detection_result'
print('Drawing bbox on image...')
category_name = {1: 'Proh.', 2: 'Illu.', 3: 'Warn.'}
colors = {1: 'red', 2: 'blue', 3: 'yellow'}
for det in tqdm(raw_results, ncols=64):
    image = Image.open(det['path'])
    bbox = det['bbox']
    score = det['score']
    category = det['category']
    w, h = det['metadata']
    name = det['path'].split('/')[-1]  # 1234.jpg
    save_path = os.path.join(save_dir, name)
    if len(bbox) < 1:
        image.save(save_path)
        continue
    # cutpoint = np.argmax(score < thresh)
    # th_bbox = bbox[:cutpoint]
    category_text = [category_name[int(e)] for e in category]
    for i, box in enumerate(bbox):
        xmin = box[1] * w
        ymin = box[0] * h
        width = (box[3] - box[1]) * w
        height = (box[2] - box[0]) * h
        bbox[i] = [xmin, ymin, width, height]
    bbox = np.asarray(bbox, dtype=int)
    # Draw bbox on image and save it.
    font0 = ImageFont.truetype('/usr/share/fonts/MyFonts/msyhbd.ttf', 20)
    for box, cat, val in zip(bbox, category, score):
        draw = ImageDraw.Draw(image)
        color = 'purple'
        width = 4
        # upper
        draw.line([(box[0], box[1]), (box[0] + box[2], box[1])],
                  fill=colors[int(cat)], width=width)
        # right
        draw.line([(box[0] + box[2], box[1]), (box[0] + box[2], box[1] + box[3])],
                  fill=colors[int(cat)], width=width)
        # lower
        draw.line([(box[0], box[1] + box[3]), (box[0] + box[2], box[1] + box[3])],
                  fill=colors[int(cat)], width=width)
        # left
        draw.line([(box[0], box[1]), (box[0], box[1] + box[3])],
                  fill=colors[int(cat)], width=width)
        draw.text((box[0], box[1]-25), '{:s} {:.3f}'.format(category_name[int(cat)], val),
                  fill='red', font=font0)
    image.save(save_path)