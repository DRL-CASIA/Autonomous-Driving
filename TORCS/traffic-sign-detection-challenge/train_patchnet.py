'''Train the Patchnet
'''

import os
import json
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    print('Not use ros cv2 already.')
import cv2
import json
import keras
import random
import numpy as np
import PIL.Image as Image
from keras.preprocessing.image import ImageDataGenerator

import utils
from patchnet import PatchNet

def pil2arr(x):
    return np.asarray(x)

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])

def prepare_dataset(image_dir, label_path, re_size, mode='train'):
    with open(label_path, 'r') as f:
        labels = json.load(f)

    filtered_labels = []
    for k, v in labels.items():
        if mode in k:
            filtered_labels.extend(v)
    random.shuffle(filtered_labels)
    gray_samples = []
    categories = []
    for label in filtered_labels:
        categories.append(label['category'])
        image = Image.open(label['path'])
        image = image.resize((re_size, re_size))
        arr = pil2arr(image)
        gray = rgb2gray(arr)
        gray = gray.reshape(re_size, re_size, 1).astype(np.uint8)
        gray_samples.append(gray)
    gray_samples = np.asarray(gray_samples, dtype=np.uint8).reshape(-1, re_size, re_size, 1)
    clahe = cv2.createCLAHE(tileGridSize=(5,5), clipLimit=10.0)
    clahe_samples = np.asarray(list(map(lambda img: clahe.apply(img), gray_samples)))
    clahe_samples = clahe_samples.reshape(-1, re_size, re_size, 1)
    categories = [e - 1 for e in categories]
    return clahe_samples, categories

def prepare_dataset_2017(train_2017_dir, label_map_dict_path):
    samples_list = utils.read_and_parse_dir(train_2017_dir)
    samples = []
    for l in samples_list:
        samples.extend(l)
    samples_list = samples
    random.shuffle(samples_list)
    with open(label_map_dict_path, 'r') as f:
        label_map_dict = json.load(f)

    label_map_dict = utils.swamp_kv(label_map_dict)

    gray_samples = []
    categories = []
    pw = None
    ph = None
    for path in samples_list:
        category_text = path.split('/')[-2]
        categories.append(int(label_map_dict[category_text]))
        image = Image.open(path)
        pw, ph = image.size
        arr = pil2arr(image)
        gray = rgb2gray(arr)
        gray = gray.astype(np.uint8)
        gray_samples.append(gray)
    gray_samples = np.asarray(gray_samples, dtype=np.uint8).reshape(-1, pw, ph, 1)
    clahe = cv2.createCLAHE(tileGridSize=(5, 5), clipLimit=10.0)
    clahe_samples = np.asarray(list(map(lambda img: clahe.apply(img), gray_samples)))
    clahe_samples = clahe_samples.reshape(-1, pw, ph, 1)
    categories = [e - 1 for e in categories]
    return clahe_samples, categories

def train(train_samples, train_labels, save_path, batch_size=32):
    model = PatchNet(input_shape=(50, 50, 1),
                     num_filters=[32, 32, 64],
                     filter_size=(5, 5),
                     num_fc_units=[128],
                     drop_keep_prob=[0.5],  # only for FC layers
                     num_classes=27)
    datagen = ImageDataGenerator()
    datagen.fit(train_samples)
    model.fit_generator(datagen.flow(train_samples, train_labels, batch_size=batch_size),
                        steps_per_epoch=int(np.ceil(train_samples.shape[0] / float(batch_size))),
                        epochs=50,
                        workers=1)
    model.save(save_path)
    return model

def evaluate(model, val_samples, val_labels):
    y_hat = model.predict(val_samples)
    preds = np.argmax(y_hat, axis=1)
    acc = (val_labels == preds).mean()
    print('Accuracy: %.4f' % acc)

def main():
    # train_full_dir = '/home/lab/traffic-sign-detection-challenge/data/full_patches/train'
    # val_full_dir = '/home/lab/traffic-sign-detection-challenge/data/full_patches/val'
    # label_path = '/home/lab/traffic-sign-detection-challenge/data/full_patches/patchnet_annotations.json'
    save_path = 'trained_models/patchnet_model/clahe_tile5_limit10_2017_2018_final2.h5'
    # train_samples, train_labels = prepare_dataset(train_full_dir, label_path, re_size, mode='train')

    # 2017 patchnet dataset
    train_1718_dir = 'data/train_2017_2018'
    label_map_dict_path = 'label_dict_idx_to_custom.json'
    train_samples_1718, train_labels_1718 = prepare_dataset_2017(train_1718_dir, label_map_dict_path)
    # Only test on offline patches.
    # val_samples, val_labels = prepare_dataset(val_full_dir, label_path, re_size, mode='offline_val')
    print('Train samples #: %d' % train_samples_1718.shape[0])
    train_labels_1718 = keras.utils.to_categorical(train_labels_1718, 27)
    patchnet = train(train_samples_1718, train_labels_1718, save_path, batch_size=32)
    # evaluate(patchnet, val_samples, val_labels)

if __name__ == '__main__':
    main()
