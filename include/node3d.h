#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.
   Each node has a unique configuration (x, y, theta) in the configuration space C.
   这是算法的核心类，每个节点的配置为(x, y, theta)
*/
class Node3D {
 public:

  /// The default constructor for 3D array initialization
  Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  //通过给定参数构造节点
  Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
  }

  // GETTER METHODS：获取方法，获取对应的参数值
  /// get the x position
  float getX() const { return x; }
  /// get the y position
  float getY() const { return y; }
  /// get the heading theta
  float getT() const { return t; }
  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node3D* getPred() const { return pred; }

  // SETTER METHODS：设置方法，用新值替代旧值
  /// set the x position
  void setX(const float& x) { this->x = x; }
  /// set the y position
  void setY(const float& y) { this->y = y; }
  /// set the heading theta
  void setT(const float& t) { this->t = t; }
  /// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }
  /// set and get the index of the node in the 3D grid
  int setIdx(int width, int height) { this->idx = (int)(t / Constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); return idx;}
  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }
  /// set a pointer to the predecessor of the node
  void setPred(const Node3D* pred) { this->pred = pred; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG();//更新从predecessor到该节点的 cost-so-far，同时标记该节点为已探查

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator == (const Node3D& rhs) const;//位置相同且朝向相似则认为是同一节点

  // RANGE CHECKING
  /// Determines whether it is appropriate to find a analytical solution.
  bool isInRange(const Node3D& goal) const;//检测是否可以分析方法找到解

  // GRID CHECKING
  /// Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height) const;//检查该点是否在3D array范围内

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  Node3D* createSuccessor(const int i);//在连续空间中创建successor

  // CONSTANT VALUES
  /// Number of possible directions
  static const int dir;//方向数量
  /// Possible movements in the x direction
  static const float dx[];//在X方向的可能移动步长
  /// Possible movements in the y direction
  static const float dy[];//在Y方向的可能移动步长
  /// Possible movements regarding heading theta
  static const float dt[];//在theta方向的可能移动步长

 private:
  /// the x position
  float x;
  /// the y position
  float y;
  /// the heading theta
  float t;
  /// the cost-so-far
  float g;//目前代价值
  /// the cost-to-go
  float h;//启发式值
  /// the index of the node in the 3D array
  int idx;//节点的索引位置
  /// the open value
  bool o;//是否属于open set
  /// the closed value
  bool c;//是否属于close set
  /// the motion primitive of the node
  int prim;
  /// the predecessor pointer
  const Node3D* pred;//祖先节点
};
}
#endif // NODE3D_H
