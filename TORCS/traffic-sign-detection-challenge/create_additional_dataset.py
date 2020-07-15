'''Create additional dataset by inserting the traffic sign ground-truth'''
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import os
import json
import numpy as np
import PIL.Image as Image
import imgaug as ia
from imgaug import augmenters as iaa
ia.seed(123)
import utils

seq = iaa.Sequential([
        # Small gaussian blur with random sigma between 0 and 0.5.
        # But we only blur about 50% of all images.
        #     iaa.Sometimes(0.5, iaa.GaussianBlur(sigma=(0, 0.5))),
        iaa.OneOf([iaa.GaussianBlur(sigma=(0, 8.0)),
                   iaa.AverageBlur(k=(9, 17)),
                   iaa.MedianBlur(k=(9, 17))]),
        # Strengthen or weaken the contrast in each image.
        iaa.ContrastNormalization((0.75, 1.5)),
        # Add gaussian noise.
        iaa.AdditiveGaussianNoise(loc=0, scale=(0.0, 0.3 * 255), per_channel=0.5),
        # Make some images brighter and some darker.
        iaa.Multiply((0.8, 1.2), per_channel=0.2),
        # Apply affine transformations to each image.
        # rotate them, we scale the patch when running due to the limitation
        # of keep origin image size fixed.
        iaa.Affine(rotate=(-20, 20))
    ], random_order=True)



def pil2arr(x):
    return np.asarray(x)

def arr2pil(x):
    return Image.fromarray(x)

def augment_per_video_seq(image_dir,
                          patch_list,
                          image_save_dir,
                          label_dic,
                          max_num_sign_per_image=5,
                          select_prob=0.2):
    label_map_dict = utils.get_label_map_dict('data/tsd_label_map_patchnet.pbtxt')
    scale_range = (0.1, 0.3)
    path = image_dir[0]
    path = path.split('/')[-2]
    video_save_dir = os.path.join(image_save_dir, path)
    for image_path in image_dir:
        if np.random.random() > select_prob:
            continue
        name = image_path.split('/')[-1]
        final_save_path = os.path.join(video_save_dir, name)

        try:
            image = Image.open(image_path)
        except:
            print('Cannot read file %s, skipped' % image_path)
            continue
        width, height = image.size
        bbox = []
        category = []
        num_sign = np.random.randint(max_num_sign_per_image)+1
        for i in range(num_sign):
            idx = np.random.randint(len(patch_list))
            patch_path = patch_list[idx]
            category_text = patch_path.split('/')[-1].split('.')[0]
            patch = Image.open(patch_path)
            arr_patch = pil2arr(patch)
            arr_patch_aug = seq.augment_image(arr_patch)
            patch_aug = arr2pil(arr_patch_aug)
            pw, ph = patch_aug.size
            # scale the patch_aug manually
            x_scale = (scale_range[1]-scale_range[0])*np.random.random() + scale_range[0]
            y_scale = (scale_range[1]-scale_range[0])*np.random.random() + scale_range[0]
            pw_new = int(x_scale*pw)
            ph_new = int(y_scale*ph)
            patch_aug = patch_aug.resize((pw_new, ph_new))
            x_min = np.random.randint(width - pw_new - 50)
            y_min = np.random.randint(height - ph_new - 50)
            if patch_aug.mode == 'RGB':
                patch_aug = patch_aug.convert('RGBA')
            image.paste(patch_aug, (x_min, y_min), patch_aug)
            bbox.append({'xmin': x_min, 'ymin': y_min,
                         'xmax': x_min + pw_new, 'ymax': y_min + ph_new})
            category.append(label_map_dict[category_text])
        if not os.path.exists(video_save_dir):
            os.makedirs(video_save_dir)
        image.save(final_save_path)
        label_dic[image_path] = {'bbox': bbox,
                                 'category': category}
    return label_dic




if __name__ == '__main__':
    image_dir = '/home/lab/datasets/TSD/additional_tsd'
    patch_dir = 'data/tsd_patches_augmentation'
    image_dir_list = utils.read_and_parse_dir(image_dir)
    patch_list = utils.read_and_parse_dir(patch_dir)
    patch_list = patch_list[0]
    patch_list = [e for e in patch_list if '.png' in e]

    max_num_sign_per_image = 3
    select_prob = 0.2  # totally ~16,000 images, we only select ~3200 randomly
    image_save_dir = '/home/lab/traffic-sign-detection-challenge/data/full_tsd_official_aug_dataset/images'
    label_save_path = 'data/full_tsd_official_aug_dataset/labels/labels.json'
    label_dic = {}
    for i, image_seq in enumerate(image_dir_list):
        print('On %d of %d' % (i+1, len(image_dir_list)))
        label_dic = augment_per_video_seq(image_seq, patch_list,
                                          image_save_dir, label_dic,
                                          max_num_sign_per_image,
                                          select_prob)
    with open(label_save_path, 'w') as f:
        json.dump(label_dic, f)
    print('Done')





