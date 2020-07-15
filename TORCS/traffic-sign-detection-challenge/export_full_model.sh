cd /home/user/ld/tf-models_old/research
echo 'Exporting full dataset coco model...'
python object_detection/export_inference_graph.py \
    --input_type image_tensor \
    --pipeline_config_path /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_resnet101_coco_full/faster_rcnn_resnet101_coco.config \
    --trained_checkpoint_prefix /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_resnet101_coco_full/train/model.ckpt-200000 \
    --output_directory /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_resnet101_coco_full/eval
