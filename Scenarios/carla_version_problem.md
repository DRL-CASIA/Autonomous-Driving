CARLA版本问题梳理

# 1. 多版本carla使用

## 1.1 import carla  

针对直接使用终端和vscode等工具的用户,需要在终端中export carla路径

```
export CARLA_ROOT=/path/to/your/carla/installation
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-<VERSION>.egg:${CARLA_ROOT}/PythonAPI/carla/agents:${CARLA_ROOT}/PythonAPI/carla
```

针对使用anaconda建立conda环境,搭配pycharm开发的用户,可以在脚本最前端加入如下代码来引入carla  
注意替换相应的目录,使用这种方法要保证anaconda环境的python=3.5 

```
from __future__ import print_function
import glob
import os
import sys

# using carla 098 as an example
sys.path.append("/{your carla unzip folder}/CARLA_098/PythonAPI/carla")
sys.path.append("/{your carla unzip folder}/CARLA_098/PythonAPI/carla/agents")
carla_path = '/{your carla unzip folder}/CARLA_098/PythonAPI'

try:
    sys.path.append(glob.glob(carla_path + '/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

```
## 1.2 server与egg版本一致
注意导入的.egg文件与实际打开的carla server版本应一致

# 2. 不同版本API可能出现的问题 
## 2.1 wait_for_tick功能
carla 096版本开始,wait_for_tick函数不再适用于synchronous mode，即

    world.settings.synchronous mode=True  
    
解决办法: 当world.synchronous mode=True时

## 2.2 spawn_actor问题

carla 098中，生成actor时有两种函数,分别是

    carla.World.spawn_actor(blueprint, transform, attach_to=None, attachment=Rigid)
以及  

    carla.World.try_spawn_actor(self, blueprint, transform, attach_to=None, attachment=Rigid)

`world.spawn_actor()`会在生成actor失败时返回错误,而`world.try_spawn_actor`则不会报错.

所以在生成关键actor时要使用`world.spawn_actor()`命令,在生成不重要的环境车辆等actor时,可以使用`world.try_spawn_actor`

在carla 098版本中，由于对于碰撞的检测机制不同,生成车辆时需设置一定的初始高度,通常大于１.0即可
否则会出现由于与地面干涉 无法正确生成车辆的情况

## 2.3 set_autopilot问题

carla 098中，还是可以使用actor.set_autopilot()函数使车辆自动驾驶，需要注意的是，在异步模式下，使用 循环 连续生成环境车辆并set_autopilot，可能存在只有首辆车运动，其余车不运动的问题。在实践中发现，我们使用synchronous mode（同步模式）运行环境，即可解决该问题。


# 3. 删除actor时可能遇到的问题
## 3.1 有效方法

carla文档中介绍了三种删除actor的方法，以carla 0.9.8版本为例，经过测试和对比，比较直接有效的方法是使用命令
`carla.client.apply_batch_sync(）`  
该命令的具体格式详见于[carla文档](https://carla.readthedocs.io/en/latest/python_api/#carlaclient)，命令会返回一个`carla.command.Response`类型的数据，主要用于检查错误，见[文档](https://carla.readthedocs.io/en/latest/python_api/#commandresponse)。  

使用方法举例：  
```
# 假设所有要删除的车辆（actor）都保存在delete_list当中
response_list=self.client.apply_batch_sync([carla.command.DestroyActor(x) for x in delete_list], True)

# 检查是否成功删除了全部车辆
method_error = response_list[0].has_error()
if not method_error:
    print('npc vehicle', delete_list[0].id, "is destroyed")

```
## 3.2 另外两种方法和问题
另外补充说明一下其他两种方法，以及可能出现的问题。
第二种是使用`carla.command.DestroyActor()`命令，例如  
```
for x in delete_list:
    carla.command.DestroyActor(x)
```  
包括单独使用命令或者是结合`carla.client.apply_batch()`命令使用
```
carla.client.apply_batch([carla.command.DestroyActor(x) for x in delete_list])
```  
这种方法在一般情况下能够顺利删除actor，但是在运行中会出现bug，导致carla server的直接崩溃，原因不明。

第二种是使用`carla.actor.destroy()`命令
```
for actor in delete_list:
    actor.destroy()
```
这种方法在实验中经常不能正常删除车辆，但并不报错，原因不明。

**相关问题可能和carla底层的C++封装有关，carla的github仓库中有相关的C++代码可以查阅，如果有同学有新的理解欢迎补充！**

