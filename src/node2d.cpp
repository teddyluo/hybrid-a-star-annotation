/**
 * @file node2d.cpp
 * @brief 
 * 
 */
#include "node2d.h"

using namespace HybridAStar;

// possible directions
const int Node2D::dir = 8;
// possible movements：8个方向相对中心(0, 0)的移动
const int Node2D::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int Node2D::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

//###################################################
//                                         IS ON GRID
//###################################################
//判断点是否在网络内（就一句话，感觉没必要写成一个函数）
bool Node2D::isOnGrid(const int width, const int height) const {
  return  x >= 0 && x < width && y >= 0 && y < height;
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
//创建successor（感觉没必要写成一个函数）
Node2D* Node2D::createSuccessor(const int i) {
  int xSucc = x + Node2D::dx[i];
  int ySucc = y + Node2D::dy[i];
  return new Node2D(xSucc, ySucc, g, 0, this);
}

//###################################################
//                                 2D NODE COMPARISON
//###################################################
//比较两个node是否相等(就一句话，感觉没必要写成一个函数)
bool Node2D::operator == (const Node2D& rhs) const {
  return x == rhs.x && y == rhs.y;
}
