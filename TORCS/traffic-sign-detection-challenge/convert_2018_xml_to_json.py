import utils
import os
import json
import tensorflow as tf
from lxml import etree

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

xml_dir = 'data/offline/TSD-Signal-GT'
files = utils.read_and_parse_dir(xml_dir)
files = files[0]
save_base = 'data/offline/TSD-Signal-GT/json'
for path in files:
    with tf.gfile.GFile(path, 'r') as fid:
        xml_str = fid.read()
    xml_str = xml_str.encode('utf-8')
    xml = etree.fromstring(xml_str)
    data = recursive_parse_xml_to_dict(xml)['opencv_storage']

    target_dict = {}
    for k, v in data.items():
        if isinstance(v, str):
            v = int(v)
        target_dict[k] = v

    name = path.split('/')[-1]
    name = name.split('.')[0]
    save_path = os.path.join(save_base, name+'.json')

    with open(save_path, 'w') as f:
        json.dump(target_dict, f)

print('Done')