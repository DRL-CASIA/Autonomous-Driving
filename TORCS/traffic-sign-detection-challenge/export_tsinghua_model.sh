cd /home/user/ld/tf-models_old/research
echo 'Exporting tsinghua kitti model...'
python object_detection/export_inference_graph.py \
    --input_type image_tensor \
    --pipeline_config_path /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_resnet101_kitti_tsinghua/faster_rcnn_resnet101_kitti.config \
    --trained_checkpoint_prefix /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_resnet101_kitti_tsinghua/train/model.ckpt-200000 \
    --output_directory /home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_resnet101_kitti_tsinghua/eval