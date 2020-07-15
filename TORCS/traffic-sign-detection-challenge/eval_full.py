import os
import json
import numpy as np
from tqdm import tqdm
from PIL import Image as Image
from PIL import ImageDraw
from PIL import ImageFont
import tensorflow as tf

import utils

def pil2arr(x):
    return np.asarray(x)

def arr2pil(x):
    return Image.fromarray(x)


test_dir = '/home/drl/ld/dataset/2018/TSD-Signal'
ckpt_path = '/home/drl/ld/traffic-sign-detection-2018/trained_models/faster-rcnn-resnet-astru-coco-tsdfull/frozen_inference_graph.pb'
save_base = '/home/drl/ld/traffic-sign-detection-2018/results/faster-rcnn-resnet-astru-coco-tsdfull-result'
test_samples = utils.read_and_parse_dir(test_dir)

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(ckpt_path, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

print('-'*30)
print('Evaluating on test dataset...')
raw_results = []
with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        for image_dir in test_samples:
            for path in tqdm(image_dir, ncols=64):
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
with open(os.path.join(save_base, 'raw_detection_result_tsinghua.json'), 'w') as f:
    json.dump({'test_result': raw_results}, f)
print('Raw detection results saved.')

print('Drawing bbox on image...')
category_name = {1: 'Proh.', 2: 'Illu.', 3: 'Warn.', 4: 'Light', 5: 'Arrow'}
colors = {1: 'red', 2: 'blue', 3: 'yellow', 4: 'green', 5: 'purple'}
for det in tqdm(raw_results, ncols=64):
    image = Image.open(det['path'])
    bbox = det['bbox']
    score = det['score']
    category = det['category']
    w, h = det['metadata']
    name = det['path'].split('/')[-1]  # 1234.jpg
    save_path = os.path.join(save_base, 'images/'+name)
    if len(bbox) < 1:
        image.save(save_path)
        continue

    category_text = [category_name[int(e)] for e in category]
    for i, box in enumerate(bbox):
        xmin = box[1] * w
        ymin = box[0] * h
        width = (box[3] - box[1]) * w
        height = (box[2] - box[0]) * h
        bbox[i] = [xmin, ymin, width, height]
    bbox = np.asarray(bbox, dtype=int)
    # Draw bbox on image and save it.
    font0 = ImageFont.truetype('/usr/share/fonts/MyFonts/msyhbd.ttf', 25)
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
        draw.text((box[0], box[1] - 25), '{:s} {:.3f}'.format(category_name[int(cat)], val),
                  fill='red', font=font0)
    image.save(save_path)

