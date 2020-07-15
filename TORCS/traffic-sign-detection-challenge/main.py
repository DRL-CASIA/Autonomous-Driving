import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    print('Not use ros cv2 already.')
import os
import copy
import json
import argparse
from tqdm import tqdm
import numpy as np
import PIL.Image as Image
from PIL import ImageDraw
from PIL import ImageFont
from patchnet import PatchNet
# from patchnet import PatchNet
import tensorflow as tf
import utils
import repairer
import xml_writer

parser = argparse.ArgumentParser()
parser.add_argument('--test_dir', type=str, help='Path to test directory')
args = parser.parse_args()

'''File reader'''
print('Reading and parsing testing directory...')
test_dir = args.test_dir
test_files_list = utils.read_and_parse_dir(test_dir)
print('Done')


'''Loading networks'''
print('Loading detection model...')
ckpt_path = 'trained_models/faster-rcnn-resnet-astru-coco-tsdfull/frozen_inference_graph.pb'
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(ckpt_path, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
print('Done')

print('Loading classification model...')
patchnet_weights = 'trained_models/patchnet_model/clahe_tile5_limit10_2017_2018_final2.h5'
patchnet = PatchNet(input_shape = (50,50,1),
                    num_filters=[32, 32, 64],
                    filter_size=(5, 5),
                    num_fc_units = [128],
                    drop_keep_prob = [0.5], # only for FC layers
                    num_classes=27)
patchnet.load_weights(patchnet_weights)
print('Done')

print('Idx, custom-official transforming...')
with open('label_dict_custom_to_official2018.json', 'r') as f:
    custom_to_official_dict = json.load(f)
with open('label_dict_idx_to_custom.json', 'r') as f:
    idx_to_custom_dict = json.load(f)
print('Done')

print('-'*30)
print('Evaluating on test files...')
raw_test_position_results = []
sess = tf.Session(graph=detection_graph)
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
num_detections = detection_graph.get_tensor_by_name('num_detections:0')
for i, dir_files_list in enumerate(test_files_list):
    print('Processing directory %d/%d' % (i + 1, len(test_files_list)))
    raw_dir_position_results = []
    dir_files_list = sorted(dir_files_list)
    print(dir_files_list[0])
    try:
        for image_path in tqdm(dir_files_list, ncols=64):
            image = Image.open(image_path)
            w, h = image.size
            image_np = np.asarray(image)
            image_np_expanded = np.expand_dims(image_np, axis=0)
            (bboxes, scores) = sess.run([detection_boxes, detection_scores],
                                        feed_dict={image_tensor: image_np_expanded})
            bboxes = bboxes.squeeze()
            scores = scores.squeeze()
            raw_dir_position_results.append({'bboxes': bboxes.tolist(),
                                            'scores': scores.tolist(),
                                            'metadata': (w, h)})
    except:
        print('Exception in posnet, skip the directory')
        continue
    raw_test_position_results.append(raw_dir_position_results)

    dir_position_results = utils.wash_with_threshold(raw_dir_position_results, 0.5)

    '''Crop patches'''
    dir_patches = []
    try:
        print('\tCropping patches...')
        for j, image_path in enumerate(dir_files_list):
            image = Image.open(image_path)
            w, h = image.size
            image_np = np.asarray(image)
            frame_position_results = dir_position_results[j]
            crops_per_image = []
            for bbox in frame_position_results:
                crop = image_np[bbox[1]:bbox[1] + bbox[3],
                       bbox[0]:bbox[0] + bbox[2], :]
                crop = Image.fromarray(crop).resize((50, 50))
                crop = np.asarray(crop)
                crops_per_image.append(np.asarray(crop))
            crops_per_image = np.array(crops_per_image)
            dir_patches.append(crops_per_image)
        print('\tDone')
    except:
        print('Exception on cropping, skip the crop')
        continue
    print('\tEvaluating on patches...')
    dir_class_results = []
    try:
        for patch in dir_patches:
            if patch.shape[0] == 0:
                dir_class_results.append(np.array([]))
                continue
            gray = utils.rgb2gray(patch)
            clahe = utils.to_clahe(gray).reshape(-1, 50, 50, 1)
            y_hat = patchnet.predict(clahe)
            # Filter the over-predicted candidates
            preds = []
            for k in range(y_hat.shape[0]):
                y = y_hat[k, ...]
                # cat = np.argmax(y) + 1
            #     if cat != 27:
            #         preds.append(cat)
            #     else:
            #         continue
                if np.max(y) < 0:
                    continue
                else:
                    cat = np.argmax(y) + 1
                    preds.append(cat)
            # preds = np.argmax(y_hat) + 1
            preds = np.asarray(preds)
            dir_class_results.append(preds)
    except:
        print('Exception in patchnet, skip the classify')
        continue
    dir_class_results_ = copy.deepcopy(dir_class_results)
    print('\tDone')
    # try:
    #     dir_class_results = repairer.wash_one_tar_detection(dir_class_results)
    # except:
    #     print('\tWARNING: repairer with exception')
    #     dir_class_results = dir_class_results_

    dir_dets = []
    try:
        for frame_pos, frame_cls in zip(dir_position_results, dir_class_results):
            frame_dets = []
            assert len(dir_position_results) == len(dir_class_results)
            for j, cls in enumerate(frame_cls):
                pos = frame_pos[j]
                try:
                    custom = idx_to_custom_dict[str(cls)]
                    label = custom_to_official_dict[custom]
                except:
                    print('\tCustom to official index except, replaced with idx 1')
                    custom = idx_to_custom_dict['1']
                    label = custom_to_official_dict[custom]
                if isinstance(pos, np.ndarray):
                    position = pos.tolist()
                frame_dets.append({'Position': pos,
                                   'Type': label})
            dir_dets.append(frame_dets)
    except:
        print('Exception in idx convertion')
        continue

    print('\tWriting to xml...')
    result_dir = 'TSD-Signal-Result-中科车智'
    if not os.path.exists(result_dir):
        os.mkdir(result_dir)
    video_name = dir_files_list[0].split('/')[-2]
    xml_path = os.path.join(result_dir, video_name + '-Result.xml')
    xml_writer.write_xml(dir_dets, xml_path)
    print('\tDone, results are saved in %s' % xml_path)

    if 0:
        print('\tDrawing detection bbox and category...')
        font0 = ImageFont.truetype('/usr/share/fonts/MyFonts/msyhbd.ttf', 25)
        save_base = 'TSD-Signal-Result'
        for j, image_path in enumerate(dir_files_list):
            image = Image.open(image_path)
            name = image_path.split('/')[-1]
            tmp_dir = os.path.join(save_base, 'images')
            if not os.path.join(tmp_dir):
                os.mkdir(tmp_dir)
            save_path = os.path.join(save_base, 'images/'+name)
            frame_dets = dir_dets[j]
            for det in frame_dets:
                box = det['Position']
                cat = det['Type']
                if cat == 'others':
                    continue
                draw = ImageDraw.Draw(image)
                color = 'red'
                width = 3
                # upper
                draw.line([(box[0], box[1]), (box[0] + box[2], box[1])],
                          fill=color, width=width)
                # right
                draw.line([(box[0] + box[2], box[1]), (box[0] + box[2], box[1] + box[3])],
                          fill=color, width=width)
                # lower
                draw.line([(box[0], box[1] + box[3]), (box[0] + box[2], box[1] + box[3])],
                          fill=color, width=width)
                # left
                draw.line([(box[0], box[1]), (box[0], box[1] + box[3])],
                          fill=color, width=width)
                draw.text((box[0], box[1] - 25), '%s'%cat, fill=color, font=font0)
            image.save(save_path)
        print('\tDone')