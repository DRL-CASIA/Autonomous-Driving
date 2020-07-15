#!/usr/bin/env bash
cd /home/user/ld/tf-models_old/research
python object_detection/train.py \
    --pipeline_config_path=/home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_inception_resnet_v2_atrous_coco_tsdaug/faster_rcnn_inception_resnet_v2_atrous_coco.config \
    --train_dir=/home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_inception_resnet_v2_atrous_coco_tsdaug/train \
    --cuda_id=2,3
