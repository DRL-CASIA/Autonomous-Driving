# 这里汇总DRL和自动驾驶相关的论文脉络，追踪前沿

一些论文forked from https://github.com/jiachenli94/Awesome-Decision-Making-Reinforcement-Learning

by 张启超  2020.3.2 

# Survey
Self-Driving Cars: A Survey, 2019, [paper] (https://arxiv.org/pdf/1901.04407.pdf)

A Survey of Motion Planning and Control Techniques for Self-Driving Urban Vehicles, 2016, [paper](https://ieeexplore.ieee.org/abstract/document/7490340)

Planning and Decision-Making for Autonomous Vehicles, MIT Reviewer, 2018, [paper](https://www.annualreviews.org/doi/abs/10.1146/annurev-control-060117-105157)

Deep Reinforcement Learning for Autonomous Driving: A Survey, 2020, [paper](https://arxiv.org/pdf/2002.00444.pdf)

A Survey of Deep Learning Applications to Autonomous Vehicle Control, IEEE Transaction on ITS 2019, [paper](https://arxiv.org/pdf/1912.10773v1.pdf)

# IL
Learning a Driving Simulator, 2016, Comma.ai  [paper](https://arxiv.org/abs/1608.01230), [code](https://github.com/commaai/research)

End to End Learning for Self-Driving Cars, 2016, NVIDIA [paper](https://arxiv.org/abs/1604.07316), [code](https://github.com/navoshta/behavioral-cloning)

Generative Adversarial Imitation Learning, NIPS 2016, [paper](https://arxiv.org/abs/1606.03476)

Robust End-to-End Learning for Autonomous Vehicles, MIT, 2018 [Master Thesis](https://dspace.mit.edu/bitstream/handle/1721.1/118031/1051458698-MIT.pdf?sequence=1&isAllowed=y)

End-to-end Driving via Conditional Imitation Learning, 2018, ICRA [paper](http://export.arxiv.org/abs/1710.02410), [code](https://github.com/carla-simulator/imitation-learning)

ChauffeurNet: Learning to Drive by Imitating the Best and Synthesizing the Worst, 2018, Waymo  [paper](https://arxiv.org/abs/1812.03079v1), [code](https://github.com/Iftimie/ChauffeurNet)

Self-Imitation Learning, ICML 2018, [paper](https://arxiv.org/abs/1806.05635), [code](https://github.com/wudongming97/self-imitation-learning)

Multi-Agent Generative Adversarial Imitation Learning, ICLR 2018, [paper](https://arxiv.org/abs/1807.09936)

Conditional affordance learning for driving in urban environments, CoRL, 2018, [paper](https://arxiv.org/abs/1806.06498), [code](https://github.com/xl-sr/CAL)

Learning by cheating， 2019， [paper](https://arxiv.org/pdf/1912.12294.pdf), [code](https://github.com/dianchen96/LearningByCheating)

Imitation Learning from Imperfect Demonstration, ICML, 2019, [paper](https://arxiv.org/pdf/1901.09387), [code](https://github.com/kristery/Imitation-Learning-from-Imperfect-Demonstration)

Exploring the Limitations of Behavior Cloning for Autonomous Driving, ICCV, 2019, [paper](https://arxiv.org/pdf/1904.08980.pdf)

Multimodal End-to-End Autonomous Driving， 2019， [paper](https://arxiv.org/pdf/1906.03199.pdf)

DEEP IMITATIVE MODELS FOR FLEXIBLE INFERENCE, PLANNING, AND CONTROL， 2020，ICLR，[paper](https://openreview.net/pdf?id=Skl4mRNYDr), [model](https://github.com/nrhine1/deep_imitative_models)

Deep Imitation Learning for Autonomous Driving in Generic Urban Scenarios with Enhanced Safety, 2020 [paper](https://arxiv.org/abs/1903.00640)

# IRL
Maximum Entropy Deep Inverse Reinforcement Learning, 2015, [paper](https://arxiv.org/abs/1507.04888)

Generative Adversarial Imitation Learning, NIPS, 2016, [paper](https://arxiv.org/abs/1606.03476)

Guided Cost Learning: Deep Inverse Optimal Control via Policy Optimization, ICML 2016, [paper](https://arxiv.org/abs/1603.00448)

A Connection between Generative Adversarial Networks, Inverse Reinforcement Learning, and Energy-Based Models, NIPS 2016, [paper](https://arxiv.org/abs/1611.03852)

InfoGAIL: Interpretable Imitation Learning from Visual Demonstrations, NIPS 2017, [paper](https://arxiv.org/pdf/1703.08840.pdf), [code](https://github.com/YunzhuLi/InfoGAIL)

Courteous Autonomous Cars, IROS, 2018, [paper](https://arxiv.org/pdf/1808.02633.pdf), [blog](https://msc.berkeley.edu/research/social_interaction_IRL.html)

 Probabilistic Prediction of Interactive Driving Behavior via Hierarchical Inverse Reinforcement Learning, ITS, 2018, [paper](https://arxiv.org/pdf/1809.02926.pdf), [blog](https://msc.berkeley.edu/research/HIRL_prediction.html)

Learning Robust Rewards with Adversarial Inverse Reinforcement Learning, ICLR 2018, [paper](https://arxiv.org/abs/1710.11248)

Multi-Agent Adversarial Inverse Reinforcement Learning, ICML 2019, [paper](https://arxiv.org/abs/1907.13220v1)

Learning Driving decisions by imitating drivers' control behaviors, 2019, [paper] (https://arxiv.org/pdf/1912.00191.pdf)

# DRL

Towards Learning Multi-agent Negotiations via Self-Play, ICCV 2019, [paper](http://openaccess.thecvf.com/content_ICCVW_2019/papers/ADW/Tang_Towards_Learning_Multi-Agent_Negotiations_via_Self-Play_ICCVW_2019_paper.pdf)

CIRL: Controllable Imitative Reinforcement Learning for Vision-based Self-driving, ECCV 2018, [paper](http://openaccess.thecvf.com/content_ECCV_2018/papers/Xiaodan_Liang_CIRL_Controllable_Imitative_ECCV_2018_paper.pdf), [code](https://github.com/HubFire/Muti-branch-DDPG-CARLA)

Imitating Driver Behavior with Generative Adversarial Networks, IV 2017, [paper] [code]

Multi-Agent Imitation Learning for Driving Simulation, IROS 2018, [paper] [code]

Simulating Emergent Properties of Human Driving Behavior Using Multi-Agent Reward Augmented Imitation Learning, ICRA 2019, [paper] [code]

Learning from Demonstration in the Wild, ICRA 2018, [paper]

Multi-Agent Connected Autonomous Driving using Deep Reinforcement Learning, NeurIPS 2019, [paper] [code]

Model-free Deep Reinforcement Learning for Urban Autonomous Driving, ITSC 2019, [paper]

A reinforcement learning based approach for automated lane change maneuvers, IV 2018, [paper]

Adversarial Inverse Reinforcement Learning for Decision Making in Autonomous Driving, ICRA 2020, [paper]

Deep hierarchical reinforcement learning for autonomous driving with distinct behaviors, IV 2018, [paper]

A Hierarchical Architecture for Sequential Decision-Making in Autonomous Driving using Deep Reinforcement Learning, ICML 2019, [paper]

End-to-end Interpretable Neural Motion Planner, CVPR 2019, [paper]

Jointly Learnable Behavior and Trajectory Planning for Self-Driving Vehicles, IROS 2019, [paper]

Dynamic Input for Deep Reinforcement Learning in Autonomous Driving, IROS 2019, [paper]

Learning to Navigate in Cities Without a Map, NIPS 2018, [paper]

Scalable End-to-End Autonomous Vehicle Testing via Rare-event Simulation, NIPS 2018, [paper]

Data-Efficient Hierarchical Reinforcement Learning, NIPS 2018, [paper]

vae+PPO in carla, Accelerating Training of Deep Reinforcement Learning-based Autonomous Driving Agents Through Comparative Study of Agent and Environment Designs, 2019, [paper](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2625841),[code](https://github.com/bitsauce/Carla-ppo)

# POMDP+ planning

Autonomous driving at intersections: a critical-turning-point approach for left turn, 2020, [paper](https://arxiv.org/pdf/2003.02409.pdf)

# prediction + planning

TPNet: Trajectory Proposal Network for Motion Prediction, 2020, CVPR, [paper](https://arxiv.org/pdf/2004.12255.pdf)



# DRL+X

DRL+PGM, Interpretable End-to-end Urban Autonomous Driving with Latent Deep Reinforcement Learning, 2020, [paper](https://arxiv.org/pdf/2001.08726.pdf) 


#Urban Autonomous Driving

Dreaming about Driving, 2018, Wayve, [blog](https://wayve.ai/blog/dreaming-about-driving-imagination-rl)

Recurrent World Models Facilitate Policy Evolution, 2018, NIPS, [paper](https://papers.nips.cc/paper/7512-recurrent-world-models-facilitate-policy-evolution.pdf), [code](https://github.com/hardmaru/WorldModelsExperiments), [blog](https://worldmodels.github.io/)

Model-Predictive Policy Learning with Uncertainty Regularization for Driving in Dense Traffic， ICLR, 2019, [paper](https://arxiv.org/abs/1901.02705), [code](https://github.com/Atcold/pytorch-PPUU), [video](https://www.bilibili.com/video/av69229685)

Variational End-to-End Navigation and Localization, ICRA, 2019, [paper](https://arxiv.org/pdf/1811.10119.pdf)

Urban Driving with Conditional Imitation Learning, Wayve, 2019, [paper](https://arxiv.org/pdf/1912.00177.pdf)

Learning to Drive from Simulation without Real World Labels, Wayve, 2018, [paper](https://arxiv.org/pdf/1812.03823.pdf) 

End-to-End Model-Free Reinforcement Learning for Urban Driving using Implicit Affordances, valeo, 2019, [paper](https://arxiv.org/pdf/1911.10868.pdf)

OUR TOP TIPS FOR CONDUCTING ROBOTICS FIELD RESEARCH, 2019, [blog](https://clearpathrobotics.com/blog/2019/06/our-top-tips-for-conducting-robotics-field-research/)

Urban Driving with Multi-Objective Deep Reinforcement Learning, AMMAS, [paper](https://dl.acm.org/doi/pdf/10.5555/3306127.3331714)



# Autonomous Driving test

DeepTest: Automated Testing of Deep-Neural-Network-driven Autonomous Cars, 2018, [paper](https://arxiv.org/pdf/1708.08559.pdf)

DeepXplore: Automated Whitebox Testing of Deep Learning Systems, 2017, [paper](https://arxiv.org/pdf/1705.06640.pdf)

# Policy transfer

Driving Policy Transfer via Modularity and Abstraction, 2018, CoRL [paper](https://arxiv.org/pdf/1804.09364.pdf)


# Focused aera

1. combined prediction with planning under uncertainty / Model-free & model-based
    
2. combined NN with rules to guarantee safey

3. desicion model testing to evaluate the performance

-----------------------------------------------------------------------------------------------------------------

# Reading paper 

## 1 DRL下的规控相关

1.1. Learning by cheating， 2019， [paper](https://arxiv.org/pdf/1912.12294.pdf), [code](https://github.com/dianchen96/LearningByCheating)
    
   该论文直接输出轨迹，动作空间可以选用，输入为带有command的local map
   
1.2. Learning Driving decisions by imitating drivers' control behaviors, 2019, [paper] (https://arxiv.org/pdf/1912.00191.pdf)

   该论文的动作空间选用了带速度规划的goal作为离散动作空间，可以参考，输入为驾驶员视野的图像，利用的是GAIL的思路，解决了控制部分独立情况下的梯度反向问题，重参数化技巧。

1.3. Probabilistic Future Prediction for Video Scene Understanding, wayve, 2020 [paper](https://arxiv.org/abs/2003.06409)

   基于预测下的决策： 场景理解与预测，预测未来的分割，深度，光流信息，数据来源CityScapes[13], Mapillary Vistas [46], ApolloScape [28] and Berkeley Deep Drive [65]. 语义分割的未来预测，在cityscape上测试


1.4. 无保护左转的论文阅读与梳理，从规控的角度看预测（高精度地图下的local map 和 感知范围内的local map可以不同），基于预测做如何做决策

1.6. world model系列，在PPUU，Interpretable E2E， world-model，planet，Dreamer， wayve情境预测 的基础上理解world model的运作机理，为planning中的快慢线中的慢线提供支撑


## 2 可解释性相关：

2.1. DRL+PGM, Interpretable End-to-end Urban Autonomous Driving with Latent Deep Reinforcement Learning, 2020, [paper](https://arxiv.org/pdf/2001.08726.pdf), [code](https://github.com/cjy1992/interp-e2e-driving)

  该论文的状态空间可以直接选用， 将PGM和world model与 RL相结合，结合了PPUU的隐空间架构，属于Model-based 与 model-free的结合
  
2.2. DeepXplore: Automated Whitebox Testing of Deep Learning Systems, 2017, [paper](https://arxiv.org/pdf/1705.06640.pdf)， [code](https://github.com/peikexin9/deepxplore), [deeptest code](https://github.com/ARiSE-Lab/deepTest)
   
  该论文为可解释性中的loss设计提供了好的思路，考虑来结合对抗测试的方式来提升算法的性能，同时为planning中的快慢线中的快线提供支撑
  
2.3. Self-Supervised Discovering of Causal Features: Towards Interpretable Reinforcement Learning, 清华大学， 2020, [paper](https://arxiv.org/pdf/2003.07069.pdf)

our framework learns to predict an attention mask to highlight the features that may be task-relevant in the state.
为了引入可解释性，如何选择可解释性的loss参考

2.4. Towards Interpretable Reinforcement Learning Using Attention Augmented Agents， DeepMind，Nips，2019, [paper](http://papers.nips.cc/paper/9400-towards-interpretable-reinforcement-learning-using-attention-augmented-agents.pdf)

简介： soft-attention model for RL： 关注任务相关的信息， attention的输出是 可视化信息， 关注space和context两个方面,论文工作很好，但是目前缺少结合的思路，值得后续继续再深入理解

how decisions are taken, what information is used and why mistakes are made. tracking the region ahead of the player, focusing on enemies and important moving objects

2.5 Toward Driving Scene Understanding: A Dataset for Learning Driver Behavior and Causal Reasoning, CVPR, 2018 [paper](http://openaccess.thecvf.com/content_cvpr_2018/papers/Ramanishka_Toward_Driving_Scene_CVPR_2018_paper.pdf)
 
    简介：提供了140小时驾驶员数据集，we need to understand the interactions between human driver behaviors and
the corresponding traffic scene situations，视频数据+位置车速等信息，定义了4类标签：goal-directed驾驶意图(左转/右转等)，应激性行为(停车/偏离车道)，应激性行为的原因cause，attention目标

2.6. graph attention 机制的融合，可解释性论文的持续阅读，特别是基于数据的IL下的可解释性


2.7 Transparency and Explanation in Deep Reinforcement Learning Neural Networks, 2018, AIES, [paper](https://dl.acm.org/doi/abs/10.1145/3278721.3278776)



2.8 Generation of Policy-Level Explanations for Reinforcement Learning, 2018, AAAI, [paper](https://www.aaai.org/ojs/index.php/AAAI/article/view/4097)




2.9 Explaining Decisions of a Deep Reinforcement Learner with a Cognitive Architecture, 2018, [paper](https://digitalcommons.usmalibrary.org/cgi/viewcontent.cgi?article=1128&context=aci_ja)



## 3. Attention相关
Social Attention for Autonomous Decision-Making in Dense Traffic， 2019， [paper](https://arxiv.org/abs/1911.12250) [code](https://github.com/eleurent/rl-agents) [blog](https://eleurent.github.io/social-attention/)


# 如何实现？分步走：

1. gym-carla环境的安装与掌握  [code](https://github.com/cjy1992/gym-carla) 解决环境配置的问题

2. local-map生成的掌握  [code](https://github.com/cjy1992/detect-loc-map) 解决输入状态表征的问题

3. baseline的掌握      [PPO/SAC code] 解决实验对比的问题

4. Learning by cheating [code](https://github.com/dianchen96/LearningByCheating)解决动作空间选择的问题

5. Interpretable E2E  [code](https://github.com/cjy1992/interp-e2e-driving) 解决world-model的问题

# 融合的idea

1. graph attention的引入DRL

2. prediction-dependent decision的引入

3. world model 的引入

4. deep test的引入

# 待阅读的papers
End-to-end Interpretable Neural Motion Planner, CVPR 2019, [paper]

Motion Prediction of Traffic Actors for Autonomous Driving using Deep Convolutional Networks， 2018， [paper](https://arxiv.org/pdf/1808.05819v1.pdf)

Jointly Learnable Behavior and Trajectory Planning for Self-Driving Vehicles, IROS 2019, [paper]

Dynamic Input for Deep Reinforcement Learning in Autonomous Driving, IROS 2019, [paper]

Learning to Navigate in Cities Without a Map, NIPS 2018, [paper]

Scalable End-to-End Autonomous Vehicle Testing via Rare-event Simulation, NIPS 2018, [paper]

Deep Imitative Models for Flexible Inference, Planning, and Control, ICLR, 2020 [paper](https://openreview.net/forum?id=Skl4mRNYDr), [code](https://github.com/nrhine1/deep_imitative_models)

MultiPath: Multiple Probabilistic Anchor Trajectory Hypotheses for Behavior Prediction, 2019, CORL [paper](https://arxiv.org/abs/1910.05449)

   AGIL: Learning Attention from Human for Visuomotor Tasks， 2018， [paper](https://arxiv.org/pdf/1806.03960.pdf)
   
   Skill Transfer in Deep Reinforcement Learning under Morphological Heterogeneity, TNNLS, 2020 
   
   Model-free Deep Reinforcement Learning for Urban Autonomous Driving, ITSC 2019, [paper]
   
   End-to-End Model-Free Reinforcement Learning for Urban Driving using Implicit Affordances, valeo, 2019, [paper](https://arxiv.org/pdf/1911.10868.pdf)
   
   Learning Robust Control Policies for End-to-End Autonomous Driving From Data-Driven Simulation, 2020, [paper](https://ieeexplore.ieee.org/abstract/document/8957584)
   
   待阅读， 可解释性与skill transfer以及潜空间之间有着非常紧密的联系
   
   
   
   # 决策规划的测试方法 ： 采集数据--> 地图构建 --> 场景筛选 -->场景重建 -->场景孪生 --> 测试库 -->评测指标 --> 
   
从形式上讲： 1. 基于挑战性数据集的回放，确定性的场景，这条路只能与企业来合作，建立什么样的合作机制呢？
            2.  基于大量生成的交通流测试，随机性场景，这条路通过与企业合作，后续有机会独立发展
            
            无论如何，都要有1个仿真器，可以是highway轻量级的纯决策  也可以是带感知的重量级carla
            
            1. 状态表征统一
            2. 场景确定
            3. 
            

从评测指标上讲：
            
  安全性指标：参照carla和车脑挑战赛 
            
            
  功能性指标：单个场景下有相应的评分细则, 参照车脑挑战赛和panosim挑战赛
  
            园区场景：
            
            高速路场景：
            
            城市路场景： 汇车
                        路口交互
                        窄道通行
                        环岛
                        超车换道
                   
   目前完成的工作： 1. 统一了状态空间？  yes  以local map为主要输入状态表征
                   2. 统一动作空间？    还差一些  首先一定明确绝大多数场景只涉及到决策规划层 goal or  trajectory
                                                 AEB等急停场景会直接涉及到加速度控制， 这块还要再梳理清楚
                   3. reward空间： 不同场景的不同规则意味着不同的reward设计，这也是目前大多数RL文献只处理1种场景的问题，这个将会牵扯到真正的科学问题， 检索urban autonomous driving， 首先要明确这是个什么科学问题 多任务学习？ 连续学习？ 元学习？
                   
                     是通过场景划分与架构方式来解决？ 还是通过算法设计与改进来解决？ 目前大家都是怎么做的？
                     
                     另外有一个问题没搞清楚：究竟是做到行为决策层 还是goal  or做到轨迹层  还是都可以？
                     
                     
整个测试的工作需要：   
                  1. 测试场景的搭建    基于carla挑战赛 和  corner case的场景
                  
                                      地图和场景是后续迁移必须要考虑的内容，为什么？ 因为local map中包含了道路属性和场景属性
                  
                  2. 基线才算的论证
                  
                  3. paper算法的创新
