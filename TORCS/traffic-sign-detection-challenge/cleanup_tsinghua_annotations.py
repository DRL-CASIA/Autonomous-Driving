'''Clenup tsinghua annotations used for classification phase.
'''
import json

path = '/home/lab/datasets/TSD/tsinghua_traffic_sign/labels/annotations.json'
convert_dict = {'pne': 'r1',
                'p10': 'r2',
                'p5': 'r3',
                'p1': 'r4',
                'pn': 'r5',
                'pl': 'r6', # Note pl stands for pl*
                'i3': 'b1',
                'i4': 'b2',
                'w32': 'y1'}


with open(path, 'r') as f:
    annotations = json.load(f)

labels = annotations['imgs']
clean_annot_dict = {}
cnter = 0
img_cnter = 0
for k, v in labels.items():
    targets = v['objects']
    path = v['path']
    mode = path.split('/')[0]
    target_num = len(targets)
    clean_targets = []
    for target in targets:
        category = target['category']
        if category in convert_dict.keys():
            clean_targets.append({'category': convert_dict.get(category),
                                  'bbox': target['bbox']})
            cnter += 1
        if 'pl' in category:
            clean_targets.append({'category': convert_dict.get('pl'),
                                  'bbox': target['bbox']})
            cnter += 1
    if len(clean_targets) > 0:
        clean_annot_dict[k] = {'mode': mode, 'targets': clean_targets}
        img_cnter += 1
with open('./tsinghua_annotations_clean.json', 'w') as f:
    json.dump(clean_annot_dict, f)

print('Valid image #: %d, sign #: %d' % (img_cnter, cnter))