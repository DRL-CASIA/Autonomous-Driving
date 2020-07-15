'''Create small image patches for PatchNet training.
'''

import os
import json
from tqdm import tqdm
from PIL import Image as Image
import utils

def read_tsinghua_dataset(image_dir, label_path,
                          label_map_dict, save_dir,
                          cnter_train, cnter_val):
    tsinghua_train_images_dir = os.path.join(image_dir, 'train')
    tsinghua_val_images_dir = os.path.join(image_dir, 'test')
    tsinghua_train_images_path = utils.read_and_parse_dir(tsinghua_train_images_dir)
    tsinghua_val_images_path = utils.read_and_parse_dir(tsinghua_val_images_dir)
    print('-' * 20)
    print('Tsinghua dataset train samples: ', len(tsinghua_train_images_path)-2)
    print('Tsinghua dataset val samples: ', len(tsinghua_val_images_path)-2)
    with open(label_path, 'r') as f:
        labels = json.load(f)

    train_samples_lst = []
    val_samples_lst = []
    for image_path in tsinghua_train_images_path:
        image = Image.open(image_path)
        name = image_path.split('/')[-1]  # e.g. 1000.jpg
        idx = name.split('.')[0]  # e.g. 1000
        if idx in ['ids', 'Thumbs']:
            continue
        if idx not in labels.keys():
            print('image %s not in annotations' % image_path)
            continue
        label = labels[idx]
        targets = label['targets']
        for target in targets:
            bbox = target['bbox']
            box = (int(bbox['xmin']), int(bbox['ymin']),
                   int(bbox['xmax']), int(bbox['ymax']))
            patch = image.crop(box)
            save_path = os.path.join(save_dir, 'train/'+str(cnter_train)+'.jpg')
            patch.save(save_path)
            train_samples_lst.append({'path': save_path,
                                      'category': label_map_dict[target['category']]})
            cnter_train += 1

    for image_path in tsinghua_val_images_path:
        image = Image.open(image_path)
        name = image_path.split('/')[-1]  # e.g. 1000.jpg
        idx = name.split('.')[0]  # e.g. 1000
        if idx in ['ids', 'Thumbs']:
            continue
        if idx not in labels.keys():
            print('image %s not in annotations' % image_path)
            continue
        label = labels[idx]
        targets = label['targets']
        for target in targets:
            bbox = target['bbox']
            box = (int(bbox['xmin']), int(bbox['ymin']),
                   int(bbox['xmax']), int(bbox['ymax']))
            patch = image.crop(box)
            save_path = os.path.join(save_dir, 'val/'+str(cnter_val)+'.jpg')
            patch.save(save_path)
            val_samples_lst.append({'path': save_path,
                                    'category': label_map_dict[target['category']]})
            cnter_val += 1

    return train_samples_lst, val_samples_lst, cnter_train, cnter_val

def read_offline_challenge_dataset(image_dir, label_dir,
                                   label_map_dict, save_dir,
                                   cnter_train, cnter_val):
    train_images_dir = os.path.join(image_dir, 'train')
    val_images_dir = os.path.join(image_dir, 'val')
    train_images_paths = utils.read_and_parse_dir(train_images_dir)
    val_images_paths = utils.read_and_parse_dir(val_images_dir)
    print('-'*25)
    print('Offline Challenge train samples:', len(train_images_paths))
    print('Offline Challenge val samples:', len(val_images_paths))
    print('-' * 12 + 'End' + '-' * 12)

    train_samples_lst = []
    val_samples_lst = []
    for image_path in tqdm(train_images_paths, ncols=64):
        image = Image.open(image_path)
        width, height = image.size
        video_name = image_path.split('/')[-2]
        frame_name = image_path.split('/')[-1]
        frame_name = frame_name.split('.')[0]
        idx = frame_name.split('-')[-1] # 00001, 00002...
        label_path = os.path.join(label_dir, video_name+'-GT.json')
        with open(label_path, 'r') as f:
            annotations = json.load(f)
        try:
            target_num = annotations['Frame%sTargetNumber'%idx]
        except:
            print('\nimage: %s not in annotations' % image_path)
            continue
        for i in range(target_num):
            target = annotations['Frame%sTarget%05d'%(idx, i)]
            position = target['Position'].strip().split(' ')
            position = [int(e) for e in position]
            xmin = position[0] if position[0] >= 0 else 0
            ymin = position[1] if position[1] >= 0 else 0
            xmax = position[0] + position[2] if position[0] + position[2] <= width else width
            ymax = position[1] + position[3] if position[1] + position[3] <= height else height
            box = (xmin, ymin, xmax, ymax)
            patch = image.crop(box)
            save_path = os.path.join(save_dir, 'train/'+str(cnter_train)+'.jpg')
            try:
                patch.save(save_path)
            except SystemError:
                print('\nCheck file: %s' % image_path)
                print('\tImage width = %d, height = %d' % (width, height))
                print('\tBox information:', box)
                continue
            try:
                train_samples_lst.append({'path': save_path,
                                      'category': label_map_dict[target['Type']]})
            except:
                print('\nCheck annotation of file: %s' % image_path)
                raise
            cnter_train += 1

    for image_path in tqdm(val_images_paths, ncols=64):
        image = Image.open(image_path)
        video_name = image_path.split('/')[-2]
        frame_name = image_path.split('/')[-1]
        frame_name = frame_name.split('.')[0]
        idx = frame_name.split('-')[-1] # 00001, 00002...
        label_path = os.path.join(label_dir, video_name+'-GT.json')
        with open(label_path, 'r') as f:
            annotations = json.load(f)
        try:
            target_num = annotations['Frame%sTargetNumber'%idx]
        except:
            print('image: %s not in annotations' % image_path)
            continue
        for i in range(target_num):
            target = annotations['Frame%sTarget%05d'%(idx, i)]
            position = target['Position'].strip().split(' ')
            position = [int(e) for e in position]
            box = (position[0], position[1],
                   position[0] + position[2],
                   position[1] + position[3])
            patch = image.crop(box)
            save_path = os.path.join(save_dir, 'val/'+str(cnter_val)+'.jpg')
            patch.save(save_path)
            val_samples_lst.append({'path': save_path,
                                      'category': label_map_dict[target['Type']]})
            cnter_val += 1

    return train_samples_lst, val_samples_lst, cnter_train, cnter_val

def main():
    '''Pipeline.
    1. Read all samples path in tsinghua and offline dataset.
    2. Read annotations, select related categories (26 categories in Challenge 2018)
    3. Crop and save bbox area with label
    '''
    tsinghua_image_dir = '/home/lab/datasets/TSD/tsinghua_traffic_sign/images'
    tsinghua_label_path = 'tsinghua_annotations_clean.json'
    offline_image_dir = '/home/lab/datasets/TSD/offline_challenge/images'
    offline_label_dir = '/home/lab/datasets/TSD/offline_challenge/labels_json/full_categories'
    label_map_dict = utils.get_label_map_dict('data/tsd_label_map_patchnet.pbtxt')
    save_dir = '/home/lab/traffic-sign-detection-challenge/data/full_patches'
    cnter_train = 0
    cnter_val = 0

    offline_train_samples, offline_val_samples, \
    cnter_train, cnter_val = read_offline_challenge_dataset(offline_image_dir,
                                                            offline_label_dir,
                                                            label_map_dict,
                                                            save_dir,
                                                            cnter_train,
                                                            cnter_val)
    offline_patchnet_annotations = {'offline_train': offline_train_samples,
                                    'offline_val': offline_val_samples}
    with open('data/full_patches/offline_patchnet_annotations.json', 'w') as f:
        json.dump(offline_patchnet_annotations, f)

    tsinghua_train_samples, tsinghua_val_samples,\
    cnter_train, cnter_val = read_tsinghua_dataset(tsinghua_image_dir,
                                                   tsinghua_label_path,
                                                   label_map_dict,
                                                   save_dir,
                                                   cnter_train,
                                                   cnter_val)
    tsinghua_patchnet_annotations = {'tsinghua_train': tsinghua_train_samples,
                                     'tsinghua_val': tsinghua_val_samples,}
    with open('data/full_patches/tsinghua_patchnet_annotations.json', 'w') as f:
        json.dump(tsinghua_patchnet_annotations, f)


    patchnet_annotations = {'tsinghua_train': tsinghua_train_samples,
                            'tsinghua_val': tsinghua_val_samples,
                            'offline_train': offline_train_samples,
                            'offline_val': offline_val_samples}
    with open('data/full_patches/patchnet_annotations.json', 'w') as f:
        json.dump(patchnet_annotations, f)

    print('-'*20)
    print('Raw Patches Information')
    print('tsinghua train #: %d, val #: %d' % (len(tsinghua_train_samples), len(tsinghua_val_samples)))
    print('offline train #: %d, val #: %d' % (len(offline_train_samples), len(offline_val_samples)))
    print('-'*20)
    print('Done')

if __name__ == '__main__':
    main()