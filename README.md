## Hybrid A* ROS源码中文注释

本仓库是Hybrid A* 的ROS版代码注释，原始代码地址为:
[https://github.com/karlkurzer/path_planner](https://github.com/karlkurzer/path_planner)

整体注释依据的主要参考文献有：

- DOLGOV D, THRUN S, MONTEMERLO M, *et al.* **Practical search techniques in path planning for autonomous driving**[J]. Ann Arbor, 2008, 1001(48105):18-80.
- KURZERK. __Path Planning in Unstructured Environments: A Real-time Hybrid A* Implementation for Fast and Deterministic Path Generation for the KTH Research Concept Vehicle__[Z].2016.
- OK K, ANSARI S, GALLAGHER B, *et al.* **Path planning with uncertainty: Voronoi uncertainty fields**[C]//2013 IEEE International Conference on Robotics and Automation. 2013:4596-4601.
- DOLGOV D, THRUN S. **Autonomous driving in semi-structured environments: Mapping and planning**[C]//2009 IEEE International Conference on Robotics and Automation. 2009:3407-3414.
- DOLGOVD, THRUNS, MONTEMERLOM, *et al*. **Path planning for autonomous vehiclesin unknown semi-structured environments**[J]. The International Journal of Robotics Research, 2010, 29(5):485-501.
- 齐尧, 徐友春, 李华等. __一种基于改进混合 A* 的智能车路径规划算法__[J]. 军事交通学院学报, 2018(2018 年 08):85-90.

Dubins路径生成参考文献（[源码来源](https://github.com/AndrewWalker/Dubins-Curves)）：
- Shkel A M, Lumelsky V. **Classification of the Dubins set**[J]. Robotics and Autonomous Systems, 2001, 34(4): 179-202.
- Giese A. **A Comprehensive, Step-by-Step Tutorial on Computing Dubins’s Curves**[J]. 2014.
- Eriksson-Bique S, Kirkpatrick D, Polishchuk V. **Discrete dubins paths**[J]. arXiv preprint arXiv:1211.2365, 2012.

Reeds-Shepp曲线参考文献：
- Reeds J, Shepp L. **Optimal paths for a car that goes both forwards and backwards**[J]. Pacific journal of mathematics, 1990, 145(2): 367-393.

**附加代码**：

- 文件夹[`EwingKang-Dubins-Curve-For-MATLAB`](./EwingKang-Dubins-Curve-For-MATLAB)提供了一份入门Hybrid A*算法的Matlab程序
- 文件夹[dynamicvoronoi](dynamicvoronoi)为原始作者实现Dynamic Voronoi的C源代码，作者为**Boris Lau**，主页为[http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/](http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/)


**注**： 

- 注释主要分布在`include`和`src`文件夹的文件内
- 若有错误请在[Issues](https://github.com/teddyluo/hybrid-a-star-annotation/issues)里提出，会尽快更正

---

**注**：关于运动规划与控制，翻译了一篇综述文献，由于刚入门，错误在所难免，请各位扔个砖头。

翻译的文献为：
Paden B, Čáp M, Yong S Z, et al. A survey of motion planning and control techniques for self-driving urban vehicles[J]. IEEE Transactions on intelligent vehicles, 2016, 1(1): 33-55.

**翻译项目主页**： 

[https://github.com/teddyluo/motion-planning-chs](https://github.com/teddyluo/motion-planning-chs)

---
*以下为原始Hybrid A Star算法的README内容*

### Hybrid A* Path Planner for the KTH Research Concept Vehicle [![Build Status](https://travis-ci.org/karlkurzer/path_planner.svg?branch=master)](https://travis-ci.org/karlkurzer/path_planner)

* [Characteristics](#characteristics)
* [Videos](#videos)
* [Images](#images)
* [Dependencies](#dependencies)
* [Setup](#setup)
* [Visualization](#visualization)
* [Citation](#citation)

The code in this repository is the result of my master's thesis which I have written at the Integrated Research Lab (ITRL) at KTH Royal Institute of Technology (2016).
The code is documented [here](http://karlkurzer.github.io/path_planner) and the associated thesis can be found [here](http://urn.kb.se/resolve?urn=urn:nbn:se:kth:diva-198534).


The goal of the thesis and hence this code is to create a real-time path planning algorithm for the nonholonomic Research Concept Vehicle (RCV). The algorithm uses a binary obstacle map as an input, generated using LIDAR mounted on top of the vehicle. The algorithm is being developed using C++ due to real-time requirements in combination with ROS to ensure modularity and portability as well as using RViz as a visualization/simulation environment.

##### <a name="characteristics"></a>Key Characteristics
* Sampling in continuous space with 72 different headings per cell (5° discretization)
* Constrained Heuristic - _nonholonomic without obstacles_
* Unconstrained Heuristic - _holonomic with obstacles_
* Dubin's Shot
* C++ real-time implementation (~10 Hz)

Large parts of the implementation are closely related to the hybrid A* algorithm developed by Dmitri Dolgov and Sebastian Thrun (_Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments_ DOI: 10.1177/0278364909359210)

##### <a name="videos"></a>Videos
* [Path Planning with Search Visualization](https://www.youtube.com/watch?v=1WZEQtg8ZZ4)
* [Dubin's Path - Constrained Heuristic](https://www.youtube.com/watch?v=VNo9fU6XEGE)
* [2D A* Search - Unconstrained Heuristic](https://www.youtube.com/watch?v=Ip2iUrVoFXc)
* [Open Loop Path Planning using Sensor Fusion](https://www.youtube.com/watch?v=GwIU00jukO4)

##### <a name="images"></a>Images
<img src="http://i.imgur.com/OICPCTB.png" alt="Reversing in a Maze" width="600"/>
<img src="http://i.imgur.com/ZiV9GDW.png" alt="Parking" width="600"/>
<img src="http://i.imgur.com/z7aT6lt.png" alt="Mitigating a U-shape Obstacle" width="600"/>

#### <a name="dependencies"></a>Dependencies
* [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/)
* [ros_map_server](http://wiki.ros.org/map_server)

#### <a name="setup"></a>Setup

Run the following command to clone, build, and launch the package (requires a sources ROS environment):

```
sudo apt install libompl-dev \
&& mkdir -p ~/catkin_ws/src \
&& cd ~/catkin_ws/src \
&& git clone https://github.com/karlkurzer/path_planner.git  \
&& cd .. \
&& catkin_make \
&& source devel/setup.bash \
&& rospack profile \
&& roslaunch hybrid_astar manual.launch
```
#### <a name="visualization"></a>Visualization (Rviz)
1. Add -> By Topic -> /map, /path, /pathVehicle, (/visualizeNode2DPoses)
2. Click 2D Pose Estimate to set a start point on the map (`p`)
3. Click 2D Nav Goal to set a goal point on the map (`g`)
4. Wait for the path being searched! (this process can be visualized [optional])

#### <a name="citation"></a>Citation
In case you are using my code for your project, I would appreciate if you include a respective citation.

```
@mastersthesis{kurzer2016,
  author       = {Karl Kurzer}, 
  title        = {Path Planning in Unstructured Environments : A Real-time Hybrid A* Implementation for Fast and Deterministic Path Generation for the KTH Research Concept Vehicle},
  school       = {KTH Royal Institute of Technology},
  year         = 2016,
  month        = 12,
}
```
