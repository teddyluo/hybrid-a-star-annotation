/**
 * @file algorithm.h
 * @brief Hybrid A* 算法的核心过程函数，只有一个函数hybridAStar()
 * 输入：
 *      始点、
 *      目标点、
 *      配置空间的3维和2维表示（2D用来A*，3D用于hybrid A*）、
 *      搜索网格的宽度及高度、
 *      配置空间的查找表、
 *      Dubins查找表（程序实际上没有使用该表，而是直接调用OMPL库计算）、
 *      RVIZ可视化类(用于显示结果)
 * 返回：
 *      满足约束条件的节点（数据结构用指针表示）
 * 
 * @date 2019-11-20
 */

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"

namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;

/*!
 * \brief A class that encompasses the functions central to the search.
 */
class Algorithm {
 public:
  /// The deault constructor
  Algorithm() {}

  // HYBRID A* ALGORITHM
  /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
     \param width the width of the grid in number of cells 网格的宽度（以单元格为单位）
     \param height the height of the grid in number of cells 网格的高度（以单元格为单位）
     \param configurationSpace the lookup of configurations and their spatial occupancy enumeration配置的查找表及空间占据计数
     \param dubinsLookup the lookup of analytical solutions (Dubin's paths) Dubin查找表
     \param visualization the visualization object publishing the search to RViz 
     \return the pointer to the node satisfying the goal condition 返回满足目标条件的节点指针
  */
  static Node3D* hybridAStar(Node3D& start,
                             const Node3D& goal,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                             Visualize& visualization);

};
}
#endif // ALGORITHM_H
