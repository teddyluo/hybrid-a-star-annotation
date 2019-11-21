#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.
   为hybrid A*算法创建接口的类，从`ros::nav_core::BaseGlobalPlanner`派生，很容易集成到ROS Navigation Stack
    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();//默认构造器

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
  void initializeLookups();//初始化碰撞检测器和启发式函数用到的搜索查找表

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);//通过订阅者监听的回调函数设置地图

  /*!
     \brief setStart
     \param start the start pose setStart()函数主要用于设置初始Pose，即设置始点
  */
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

  /*!
     \brief setGoal
     \param goal the goal pose 设置目标pose，即设置终点
  */
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
     核心规划函数
  */
  void plan();

 private:
  /// The node handle
  ros::NodeHandle n; //节点句柄
  /// A publisher publishing the start position for RViz
  ros::Publisher pubStart; //为RVIZ发布始点的发布器
  /// A subscriber for receiving map updates
  ros::Subscriber subMap;//接收地图信息的订阅器
  /// A subscriber for receiving goal updates
  ros::Subscriber subGoal;//接收目标更新的订阅器
  /// A subscriber for receiving start updates
  ros::Subscriber subStart;//接收起点更新的订阅器
  /// A listener that awaits transforms
  tf::TransformListener listener; //监听TF变换的变换树
  /// A transform for moving start positions
  tf::StampedTransform transform;//用于改变始点的变换
  /// The path produced by the hybrid A* algorithm
  Path path;//生成混合A*路径的实体对象
  /// The smoother used for optimizing the path
  Smoother smoother;//路径平滑实体
  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);//用于发布给控制器的平滑路径
  /// The visualization used for search visualization
  Visualize visualization;//可视化对象，与RVIZ交互
  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;//碰撞检测类实体，用以检测某个配置是否会发生碰撞
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram; //Voroni Diagram
  
  //用来存储RVIZ的结果
  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  geometry_msgs::PoseWithCovarianceStamped start;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;

  //查找表：用于碰撞的查找表及Dubin PATH的查找表
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions]; 
  /// A lookup of analytical solutions (Dubin's paths)
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};
}
#endif // PLANNER_H
