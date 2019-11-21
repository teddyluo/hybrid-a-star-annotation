/**
 * @file smoother.h
 * @brief 用于将路径进行平滑处理的类
 *  
 */

#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "dynamicvoronoi.h"
#include "node3d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.
   
   It also uses the Voronoi diagram as well as the configuration space.
*/
class Smoother {
 public:
  Smoother() {}

  /*!
     \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.
     核心函数，将由节点组成的路径采用梯度下降方法进行平滑
     During the different interations the following cost are being calculated
     不同的迭代阶段采用计算下面的代价值作为指标
     obstacleCost：障碍物代价
     curvatureCost：曲率代价
     smoothnessCost：平滑代价
     voronoiCost: Voronoi代价 (实际计算时没有这一项)
  */
  void smoothPath(DynamicVoronoi& voronoi);

  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
     给定一个点的指针，递归地跟踪该点到根节点的路径，其中：
     参数node：为一个3D节点类，通常是一个目标节点
     参数i：对节点进行计数的参数
  */
  void tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());

  /// returns the path of the smoother object：返回平滑后的路径
  std::vector<Node3D> getPath() {return path;}

  /// obstacleCost - pushes the path away from obstacles
  Vector2D obstacleTerm(Vector2D xi);//障碍物项，用于约束路径远离障碍物

  /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
  Vector2D curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);//曲率项，用于保证可转弯性及通行性

  /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
  //平滑项，用于将节点等距分布并尽量保持同一个方向
  Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

  /// voronoiCost - trade off between path length and closeness to obstaclesg
  //   Vector2D voronoiTerm(Vector2D xi);

  /// a boolean test, whether vector is on the grid or not
  bool isOnGrid(Vector2D vec) {
    if (vec.getX() >= 0 && vec.getX() < width &&
        vec.getY() >= 0 && vec.getY() < height) {
      return true;
    }
    return false;
  }

 private:
  /// maximum possible curvature of the non-holonomic vehicle
  float kappaMax = 1.f / (Constants::r * 1.1);
  /// maximum distance to obstacles that is penalized
  float obsDMax = Constants::minRoadWidth;
  /// maximum distance for obstacles to influence the voronoi field
  float vorObsDMax = Constants::minRoadWidth;

  //权重系数
  /// falloff rate for the voronoi field
  float alpha = 0.1;
  /// weight for the obstacle term
  float wObstacle = 0.2;
  /// weight for the voronoi term
  float wVoronoi = 0;
  /// weight for the curvature term
  float wCurvature = 0;
  /// weight for the smoothness term
  float wSmoothness = 0.2;

  // 描述地图中拓扑结构的voronoi diagram
  /// voronoi diagram describing the topology of the map
  DynamicVoronoi voronoi;

  //地图宽度与高度
  /// width of the map
  int width;
  /// height of the map
  int height;

  //待平滑的路径
  /// path to be smoothed
  std::vector<Node3D> path;
};
}
#endif // SMOOTHER_H
