'''Analyze the dataset, give related statistics.
'''
import os
import json
import numpy as np
import PIL.Image as Image
from collections import Counter

import utils

class PatchnetDatasetAnalyzer(object):
    def __init__(self, train_dir, val_dir, label_path, label_map_path):
        self.train_dir = train_dir
        self.val_dir = val_dir
        self.train_samples_lst = utils.read_and_parse_dir(train_dir)
        self.val_samples_lst = utils.read_and_parse_dir(val_dir)
        with open(label_path, 'r') as f:
            self.labels = json.load(f)
        dict = utils.get_label_map_dict(label_map_path)
        self.label_map_dict = utils.swamp_kv(dict)

    def report_count_info(self):
        print('Train samples #: %d, val samples #: %d' % (len(self.train_samples_lst),
                                                          len(self.val_samples_lst)))
        print('\tTsinghua train #: %d, tsinghua val #: %d' % (len(self.labels['tsinghua_train']),
                                                               len(self.labels['tsinghua_val'])))
        print('\tOffline val #: %d, offline val #: %d' % (len(self.labels['offline_train']),
                                                           len(self.labels['offline_val'])))

    def report_category_info(self):
        categories = []
        for name, dataset in self.labels.items():
            categories.extend([target['category'] for target in dataset])
        counter = Counter(categories)
        num = 0
        for k, v in counter.items():
            print(self.label_map_dict[k] + ': %d' % v, end=' | ')
            if num % 5 == 0 and num != 0:
                print('')
            num += 1



if __name__ == '__main__':
    analyzer = PatchnetDatasetAnalyzer('data/full_patches/train',
                                       'data/full_patches/val',
                                       '/home/lab/traffic-sign-detection-challenge/data/full_patches/patchnet_annotations.json',
                                       '/home/lab/traffic-sign-detection-challenge/tsd_label_map_classification.pbtxt')
    analyzer.report_count_info()
    analyzer.report_category_info()
