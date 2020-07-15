import torch
from road_seg import ERFNet
from torch.autograd import Variable
from PIL import Image as PImage
import torch
from torchvision.transforms import ToTensor, ToPILImage, Compose, Resize
from transform import Relabel

import cv2
import glob
import os
import numpy as np
import matplotlib.pyplot as plt
import time
input_transform_bdd = Compose([

    Resize(512),

    ToTensor(),

])

bdd_trainIds2labelIds = Compose([

    Relabel(2, 2),

    Relabel(1, 1),

    Relabel(0, 0),

    Relabel(255, 0),

    ToPILImage(),

    Resize(512, PImage.NEAREST),

])

campus_images_dir = "./image_raw"
results_dir = "./save_results/campus/"
if not os.path.exists(results_dir):
    os.makedirs(results_dir)

images_files = glob.glob(os.path.join(campus_images_dir, "*.jpg"))
device = 'cuda' if torch.cuda.is_available() else 'cpu'
use_cpu = False
num_classes = 3
model = ERFNet(num_classes)
weightspath = "./model_best.pth"

model = torch.nn.DataParallel(model)
if (not use_cpu):
    model = model.cuda()
model.load_state_dict(torch.load(weightspath))
print("Load model path from ", weightspath)
model.eval()
for f in images_files:
    image = PImage.open(f)

    image = image.resize((896, 512))
    rgb_image = np.array(image)
    images_th = input_transform_bdd(image).float()
    images_th = images_th.unsqueeze_(0)
    if (not use_cpu):
        images_th = images_th.cuda()
    inputs = Variable(images_th)
    time_start = time.time()
    with torch.no_grad():
        outputs = model(inputs)
    time_use = time.time() - time_start

    label = outputs[0].max(0)[1].byte().cpu().data
    label_bdd = bdd_trainIds2labelIds(label.unsqueeze(0))
    road_map = np.array(label_bdd)

    print(time_use)
    print(np.unique(road_map))
    print(road_map.shape)

    rgb_image[:, :, 1] += road_map * 50
    rgb_image = np.clip(rgb_image, 0, 255)
    rgb_image = np.array(rgb_image, dtype=np.uint8)
    fname = os.path.join(results_dir, os.path.basename(f))
    cv2.imwrite(fname, rgb_image)
