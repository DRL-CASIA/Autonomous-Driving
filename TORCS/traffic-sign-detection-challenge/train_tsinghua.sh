cd /home/user/ld/tf-models_old/research
python object_detection/train.py \
    --pipeline_config_path=/home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_resnet101_kitti_tsinghua/faster_rcnn_resnet101_kitti.config \
    --train_dir=/home/user/ld/traffic-sign-detection-challenge/models/faster_rcnn_resnet101_kitti_tsinghua/train \
    --cuda_id=0
