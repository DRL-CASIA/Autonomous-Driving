#!/usr/bin/env bash
echo 'Exporting full dataset coco model...'
cd /home/user/ld/tf-models_old/research
python object_detection/export_inference_graph.py \
    --input_type image_tensor \
    --pipeline_config_path /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_inception_resnet_v2_atrous_coco_tsdaug/faster_rcnn_inception_resnet_v2_atrous_coco.config \
    --trained_checkpoint_prefix /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_inception_resnet_v2_atrous_coco_tsdaug/train/model.ckpt-50715 \
    --output_directory /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_inception_resnet_v2_atrous_coco_tsdaug/eval