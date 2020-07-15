import utils
import os
import json
import tensorflow as tf
from lxml import etree
import PIL.Image as Image

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


xml_dir = 'data/offline/2017_test/labels'
files = utils.read_and_parse_dir(xml_dir)
# files = files[0]
save_base = 'data/offline/2017_test/json'
image_base = 'data/offline/2017_test/TSD-Signal'
global_cnter = 50000
for dir_files in files:
    for path in dir_files:
        with tf.gfile.GFile(path, 'r') as fid:
            xml_str = fid.read()
        # xml_str = xml_str.encode('utf-8')
        xml = etree.fromstring(xml_str)
        data = recursive_parse_xml_to_dict(xml)['annotation']
        video_name = path.split('/')[-2]
        frame_name = path.split('/')[-1]
        frame_name = frame_name.split('.')[0]
        image_path = os.path.join(image_base, video_name+'/'+frame_name+'.png')
        image = Image.open(image_path)
        for obj in data['object']:
            cat_name = obj['name']
            bbox = obj['bndbox']
            xmin = int(bbox['xmin'])
            ymin = int(bbox['ymin'])
            xmax = int(bbox['xmax'])
            ymax = int(bbox['ymax'])
            box = (xmin, ymin, xmax, ymax)
            patch = image.crop(box)
            patch = patch.resize((50, 50))
            patch_save_base = os.path.join(save_base, cat_name)
            if not os.path.exists(patch_save_base):
                os.mkdir(patch_save_base)
            patch_save_path = os.path.join(patch_save_base, str(global_cnter)+'.png')
            patch.save(patch_save_path)
            global_cnter += 1
print('Done')