## 超车换道相关调研

### 1、已有工作基础

1)    带有规则约束的强化学习横向换道决策（IJCNN 2019）
2)    结合车道保持的换道决策（李栋师兄工作）
3)    自己构建的高速车道保持场景，并复现基于物理真值的车道保持
4)    自己构建的低速转弯、红绿灯启停场景，强化学习负责高层决策部分


  目前，已有的工作基础在超车换道决策方面都只考虑了横向控制，纵向控制往往基于规则。在算法层面，可以结合赵老师和之前师兄们的建议，从两个方面对之前的工作做一个拓展：1) 横纵向控制同时考虑，具体纵向控制器如何拿进来需要进一步研究探讨；2) 强化学习的动作输出可以是更高层的“语义”指令，类似有限状态机的状态。


### 2、 难点

1)    算法外部

- 场景的定义，主要是周围车辆的行为定义，需要给出合理的解释，如果是从实际数据中建模而来相对来说比较直接，但也有一定不足（如实现难度，数据分布等）
- 算法的评估，即智能体行为的评价方式，需要结合场景定义综合考虑

2)    算法内部

- 状态的表征，之前网格形式的表征可以作为备选之一，但目前文献中以这种网格形式作为状态表征的不多，大多是以具有特定物理含义的向量表示
- 考虑交互的决策


### 3、相关资料

1)   interaction decision

- Interaction-aware Decision Making with Adaptive Strategies under Merging Scenarios [paper](https://arxiv.org/pdf/1904.06025.pdf) 汇车场景，利用多智能体的方法将交互考虑进来
- https://github.com/jiachenli94/Awesome-Interaction-aware-Trajectory-Prediction [github] 非常丰富的关于interaction-aware的资料

2)   self-play

- Towards Learning Multi-agent Negotiations via Self-Play [paper](https://arxiv.org/pdf/2001.10208.pdf)

3)   场景与算法评估

- Accelerated Evaluation of Automated Vehicles Safety in Lane-Change Scenarios Based on Importance Sampling Techniques [paer](https://arxiv.org/ftp/arxiv/papers/1605/1605.04965.pdf)
- Accelerated Evaluation of Autonomous Vehicles in the Lane Change Scenario Based on Subset Simulation Technique [paper](https://ieeexplore.ieee.org/document/8569800)
- Development of a Scenario Simulation Platform to Support Autonomous Driving Verification [paper](https://ieeexplore.ieee.org/document/8964914)

4)   换道决策算法

- Reinforcement Learning based Lane Change Decision-Making with Imaginary Sampling [paper](https://ieeexplore.ieee.org/document/9003029)
- Autonomous Driving using Safe Reinforcement Learning by Incorporating a Regret-based Human Lane-Changing Decision Model [paper](https://arxiv.org/pdf/1910.04803.pdf)
- Multiple criteria decision-making for lane-change model [paper](https://arxiv.org/pdf/1910.10142.pdf)
- Automated Lane Change Strategy using Proximal Policy Optimization-based Deep Reinforcement Learning [paper](https://arxiv.org/pdf/2002.02667.pdf)


### 4、换道决策算法几篇文献总结

- Reinforcement Learning based Lane Change Decision-Making with Imaginary Sampling（李栋师兄的文章）

  模拟器：TORCS
  算法：Dueling DQN + Gaussian Process + DDPG
  分别负责高层换道决策、预测状态与奖赏、低层车道保持
  状态：${s_t} = \left[ {{l_{host}},{v_{host}},{d}_{i,j},{v}_{i,j}} \right],\;\;\;i \in \left\{ {1,2,3} \right\}, j \in \left\{ {{\rm{front}},{\rm{ back}}} \right\}$
  只考虑当前以及左右车道[-10, 60]米范围内的最近车辆，只有越过车道线2d范围才算换道成功
  动作：向左换道，保持车道，向右换道三个离散动作
  奖赏：碰撞-10，终止；无效动作-1；动作切换-0.1；正常情况每步0.3
  优势：可以在换道中途取消换道，回到原车道；缺点：换道用切换车道保持的目标车道完成，换道过程比较“生猛”

- Autonomous Driving using Safe Reinforcement Learning by Incorporating a Regret-based Human Lane-Changing Decision Model [Michigan State University] [2019.10.10]

  
  与我们之前带规则限制的换道决策类似，在学习方法输出驾驶动作之后有一个对这个动作安全性的判定。特色之处在于建立了一个人类驾驶员换道的模型，基于此模型对学习的动作进行修正。

- Multiple criteria decision-making for lane-change model [UC, Berkeley] [2019.10.22]

  非基于学习的方法，将换道决策中要考虑的诸多因素作为激励，并将它们结合
  文中未对测试场景做充分的解释，只提到仿真器是自己搭建的，具体环境没有明确说明
  文中综合考虑了路线变更激励、速度激励、礼貌性激励、舒适性激励、安全性激励等，并将它们线性结合，通过调整每部分的权重转换驾驶风格，这部分可以为强化学习的奖赏函数设计、智能体评价、驾驶风格转换提供参考。

- Automated Lane Change Strategy using Proximal Policy Optimization-based Deep Reinforcement Learning [UC, Berkeley] [2020.02.07]

  在SUMO中依据实际地图建立仿真，其他车辆的控制器及本车的低层纵向控制器均为intelligent driver model (IDM)，环境车辆数目未知
  评价指标：成功率（在出口前换到指定车道），撞车率（最终4%左右）


### 5、场景搭建途径

1)   CARLA自带：

- scenario_runner中的换道：最新0.9.7版本，两辆环境车
- 车辆set_autopilot()：无法设置指定车速，无法在任意位置设置自动驾驶
- traffic_manager：官方还在持续更新中

2)   MACAD-Gym：基于CARLA，两到三辆车 [link](https://github.com/praveen-palanisamy/macad-gym)

```bash
Environment-ID: Short description
{'HeteNcomIndePOIntrxMATLS1B2C1PTWN3-v0': 'Heterogeneous, Non-communicating, '
                                          'Independent,Partially-Observable '
                                          'Intersection Multi-Agent scenario '
                                          'with Traffic-Light Signal, 1-Bike, '
                                          '2-Car,1-Pedestrian in Town3, '
                                          'version 0',
 'HomoNcomIndePOIntrxMASS3CTWN3-v0': 'Homogenous, Non-communicating, '
                                     'Independed, Partially-Observable '
                                     'Intersection Multi-Agent scenario with '
                                     'Stop-Sign, 3 Cars in Town3, version 0'}
```

3)   SUMMIT：基于CARLA的扩展，暂未开源

4)   CarlaScenarioLoader：[link](https://github.com/MrMushroom/CarlaScenarioLoader)

A scenario loader for the automotive simulator Carla 0.9.5. Loads scenarios based on OpenScenario 0.9.1.

5)   RareSim：Scalable End-to-End Autonomous Vehicle Testing via Rare-event Simulation [NIPS 2018]

[link](https://github.com/travelbureau/RareSim)  Coming soon，上次更新6个月前

基于目前的调研情况，在CARLA上做换道决策，需要考虑地图与周围车辆的控制问题。


 **解决CARLA_0.9.6版本仿真器“吃鼠标”的问题：**

  编辑`/CARLA_0.9.6/CarlaUE4/Config/DefaultInput.ini`文件，将其中四句替换成：

```
bCaptureMouseOnLaunch=False
DefaultViewportMouseCaptureMode=CaptureDuringMouseDown
bDefaultViewportMouseLock=False
DefaultViewportMouseLockMode=DoNotLock
```
