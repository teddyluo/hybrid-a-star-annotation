/**
 * @file planner.cpp
 * @brief 规范器的实现过程
 * 
 * 
 */

#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH：定义了一个发布器
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);//从"/map" topic里接收静态地图
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);//从"/occ_map" topic里接收动态地图
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);//接收目标的topic
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);//接收始点的topic
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
//初始化 查找表，主要有两个：Dubins Looup Table及Collision Looup Table
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;//更新地图指针
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;//二维数组，
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }//这里可简化为一次申请

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }//转化为二值地图

  voronoiDiagram.initializeMap(width, height, binMap);//注意这里传入到DynamicVoronoi里并进行保存，所以没有delete
  voronoiDiagram.update();
  voronoiDiagram.visualize();//将Voronoi Diagram初始化、更新并显示
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  //动态地图情况下的TF变换
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);
    
    //检查起点是否在有效范围内
    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();//启动规划函数
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
//这是回调函数，当接收到始点时自动调用
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}//若为静态地图时

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
// !!! 核心函数，规划过程函数
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0, 2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();

    //核心步骤：
    // 1) 调用hybridAStar()函数获取一条路径
    // 2) 获取路径点(3D Node) -> 原始路径
    // 3) 对路径依据Voronoi图进行平滑->平滑路径
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, 
    configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    //将结果在相应的topic进行发布
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    delete [] nodes3D;
    delete [] nodes2D;
    //注：这里的nSolution是new出来的，应该在用完后删除
    //(下面增加的两句待验证)
    if(nSolution)
        delete [] nSolution;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
