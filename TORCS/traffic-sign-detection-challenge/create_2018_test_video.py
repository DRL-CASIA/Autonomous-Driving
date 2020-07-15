import os
import utils
import tensorflow as tf
import PIL.Image as Image
from PIL import ImageDraw
from PIL import ImageFont
from lxml import etree
import cv2

image_dir_base = '2018_testset/TSD-Signal'
xml_dir = '2018_testset/TSD-Signal-Result'
font0 = ImageFont.truetype('/usr/share/fonts/MyFonts/msyhbd.ttf', 25)
save_base = '2018_testset/show'
if not os.path.exists(save_base):
    os.makedirs(save_base)

xml_files_lst = utils.read_and_parse_dir(xml_dir)
xml_files_lst = xml_files_lst[0]

# for xml_file in xml_files_lst:
#     name = xml_file.split('/')[-1]
#     name = name.split('.')[0]
#     name = name[:-7]
#     image_dir = os.path.join(image_dir_base, name)
#     images = utils.read_and_parse_dir(image_dir)[0]
#     with tf.gfile.GFile(xml_file, 'r') as fid:
#         xml_str = fid.read()
#     xml_str = xml_str.encode('utf-8')
#     xml = etree.fromstring(xml_str)
#     data = utils.recursive_parse_xml_to_dict(xml)['opencv_storage']
#     num_images = len(images)
#     for i in range(num_images):
#         image_name = images[i]
#         image = Image.open(image_name)
#         image_name = image_name.split('/')[-1]
#         save_path = os.path.join(save_base, image_name)
#         target_num = int(data['Frame%05dTargetNumber'%i])
#         draw = ImageDraw.Draw(image)
#         for j in range(target_num):
#             target = data['Frame%05dTarget%05d'%(i, j)]
#             box = target['Position']
#             cat = target['Type']
#             box = box.split(' ')
#             box = [int(e) for e in box]
#             color = 'red'
#             width = 3
#             # upper
#             draw.line([(box[0], box[1]), (box[0] + box[2], box[1])],
#                       fill=color, width=width)
#             # right
#             draw.line([(box[0] + box[2], box[1]), (box[0] + box[2], box[1] + box[3])],
#                       fill=color, width=width)
#             # lower
#             draw.line([(box[0], box[1] + box[3]), (box[0] + box[2], box[1] + box[3])],
#                       fill=color, width=width)
#             # left
#             draw.line([(box[0], box[1]), (box[0], box[1] + box[3])],
#                       fill=color, width=width)
#             draw.text((box[0], box[1] - 25), '%s' % cat, fill=color, font=font0)
#         image.save(save_path)
#     print('Done')

det_image_files_list = utils.read_and_parse_dir(save_base)[0]
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
fps = 5
video_writer = cv2.VideoWriter('2018_testset/traffic_signal_detection_test_2018.mp4', fourcc, fps, (1280, 1024))
for image_path in det_image_files_list:
    img = cv2.imread(image_path)
    video_writer.write(img)
video_writer.release()
print('Done')