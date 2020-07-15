# traffic-sign-detection-challenge

本仓库存放2018年比赛的代码和日志文件，2017年比赛的程序参见`challenge_2017`分支。

#### TODO
- 尝试FCN直接检测+预测是否含有标志+[分类]。
- 分类网络（PatchNet）的精细化调整，包括数据增广，参考[github](https://github.com/aleju/imgaug)。
- 检测网络的训练超参数调整，检测网络训练时，可考虑先采用Offline，再用tt100k微调，防止tt100k的数据量过大导致数据偏离。
- 按照Challenge2018实现xml文件自动写入，可参考Challenge2017的xml写入文件。
- All in One Inference，可参考Challenge2017的`main.py`

### Pipeline
- 检测（PosNet）数据集的准备：`create_full_tf_record.py`或`create_tsd_tf_record.py`(only for tt100k)
- 检测网络训练：配置`models`文件夹下的网络（`.config`文件），上运行`train_full.sh`或`train_tsinghua.sh`（可能需要修改shell脚本中的路径）
- 分类网络（PatchNet）数据集的准备：`create_patchnet_dataset.py`
- 分类网络训练：`train_patchnet.py`

### Useful Reference
- 数据增广：https://github.com/aleju/imgaug
- TT100K paper: https://cg.cs.tsinghua.edu.cn/traffic-sign/
- Tensorflow Object Detection API: https://github.com/tensorflow/models/tree/master/research/object_detection
