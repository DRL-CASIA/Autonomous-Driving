import os
import json
import utils
import PIL.Image as Image

image_dir = '/home/drl/ld/dataset/2018/TSD-Signal'
label_dir = 'data/offline/TSD-Signal-GT/json'
save_base = 'data/patches_2018'
label_map_dict = 'label_dict_custom_to_official2018.json'
with open(label_map_dict, 'r') as f:
    label_map_dict = json.load(f)
label_map_dict = utils.swamp_kv(label_map_dict)

images_list = utils.read_and_parse_dir(image_dir)
global_cnt = 500
for image_dir_list in images_list:
    name = image_dir_list[0]
    name = name.split('/')[-2]  # video name
    label_path = os.path.join(label_dir, name+'-GT.json')
    with open(label_path, 'r') as f:
        annotations = json.load(f)

    for image_path in image_dir_list:
        image = Image.open(image_path)
        idx = image_path.split('-')[-1]
        idx = idx.split('.')[0]
        cnter = annotations['Frame%sTargetNumber' % idx]
        for i in range(cnter):
            target = annotations['Frame%sTarget%05d' % (idx, i)]
            position = target['Position'].strip().split(' ')
            position = [int(e) for e in position]
            box = (position[0], position[1],
                    position[0] + position[2], position[1] + position[3])
            patch = image.crop(box)
            patch = patch.resize((50, 50))
            cat = target['Type']
            save_dir = os.path.join(save_base, label_map_dict[cat])
            if not os.path.exists(save_dir):
                os.mkdir(save_dir)
            save_path = os.path.join(save_dir, '%05d'%global_cnt + '.jpg')
            global_cnt += 1
            patch.save(save_path)
print('Done')