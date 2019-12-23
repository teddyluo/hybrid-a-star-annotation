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

#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

//原始A*算法，用来搜索计算 holonomic-with-obstacles heuristic
float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
//计算start到目标点goal的启发式代价(即：cost-to-go)
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
//Dubins shot的路径
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
/**
 * 重载运算符，用来生成节点的比较 逻辑，該函數在“boost::heap::compare<CompareNodes>”获得使用
 */
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                       3D A*
//  Hybrid A* 的主调用函数，输入参数的意义如该文件开始说明
//###################################################
Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? 6 : 3;//如果是后退，那么可能的前进方向为6；否则为3
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;//迭代计数

  // VISUALIZATION DELAY
  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;//open集

  // update h value: 计算到目标的启发式值
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);
  // mark start as open
  start.open();//将start加入open 集合: 1)将点标记为open
  // push on priority queue aka open list
  O.push(&start);// 2) 加入集合
  iPred = start.setIdx(width, height); //计算索引位置
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  // float max = 0.f;

  // continue until O empty
  while (!O.empty()) {

    //    // DEBUG
    //    Node3D* pre = nullptr;
    //    Node3D* succ = nullptr;

    //    std::cout << "\t--->>>" << std::endl;

    //    for (priorityQueue::ordered_iterator it = O.ordered_begin(); it != O.ordered_end(); ++it) {
    //      succ = (*it);
    //      std::cout << "VAL"
    //                << " | C:" << succ->getC()
    //                << " | x:" << succ->getX()
    //                << " | y:" << succ->getY()
    //                << " | t:" << helper::toDeg(succ->getT())
    //                << " | i:" << succ->getIdx()
    //                << " | O:" << succ->isOpen()
    //                << " | pred:" << succ->getPred()
    //                << std::endl;

    //      if (pre != nullptr) {

    //        if (pre->getC() > succ->getC()) {
    //          std::cout << "PRE"
    //                    << " | C:" << pre->getC()
    //                    << " | x:" << pre->getX()
    //                    << " | y:" << pre->getY()
    //                    << " | t:" << helper::toDeg(pre->getT())
    //                    << " | i:" << pre->getIdx()
    //                    << " | O:" << pre->isOpen()
    //                    << " | pred:" << pre->getPred()
    //                    << std::endl;
    //          std::cout << "SCC"
    //                    << " | C:" << succ->getC()
    //                    << " | x:" << succ->getX()
    //                    << " | y:" << succ->getY()
    //                    << " | t:" << helper::toDeg(succ->getT())
    //                    << " | i:" << succ->getIdx()
    //                    << " | O:" << succ->isOpen()
    //                    << " | pred:" << succ->getPred()
    //                    << std::endl;

    //          if (pre->getC() - succ->getC() > max) {
    //            max = pre->getC() - succ->getC();
    //          }
    //        }
    //      }

    //      pre = succ;
    //    }

    // pop node with lowest cost from priority queue
    //循环部分：从集合中取出一个最低代价的点
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width, height);//获取该点在nodes3D的索引 (前缀i表示index, n表示node)
    iterations++;//记录迭代次数

    // RViz visualization
    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {//检查该点是否closed状态
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;//如果为closed，说明该点已经处理过，忽略(将它从open set中移除)
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {//如果该点是在open状态，即正在扩张的点
      // add node to closed list
      nodes3D[iPred].close();//将它的状态标记为closed
      // remove node from open list
      O.pop();//并从open set中移除 (先取出点来，再干活)

      // _________
      // GOAL TEST检测当前节点是否是终点或者是否超出解算的最大时间(最大迭代次数)
      if (*nPred == goal || iterations > Constants::iterations) {
        // DEBUG
        return nPred;
      }//检查该点是否为目标点：是的话直接返回，表示搜索结束；不是的话就继续搜索
      //___________________
      // CONTINUE WITH SEARCH
      else {//不是目标点，那么就找到目标点的点
        // _______________________
        // SEARCH WITH DUBINS SHOT//车子是在前进方向，优先考虑用Dubins去命中目标点
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace);
          
          //如果Dubins方法能直接命中，即不需要进入Hybrid A*搜索了，直接返回结果
          if (nSucc != nullptr && *nSucc == goal) {
            //DEBUG
            // std::cout << "max diff " << max << std::endl;
            return nSucc;//如果下一步是目标点，可以返回了
          }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {//每个方向都搜索
          // create possible successor
          // 创建下一个扩展节点，这里有三种可能的方向，如果可以倒车的话是6种方向
          nSucc = nPred->createSuccessor(i);//找到下一个点
          // set index of the successor
          // 设置节点遍历标识
          iSucc = nSucc->setIdx(width, height);//索引值

          // ensure successor is on grid and traversable：确保是在有效范围内
          // 判断扩展节点是否满足约束，能则继续进行遍历
          // 首先判断产生的节点是否在范围内；其次判断产生的节点会不会产生碰撞；
          // 只有同时满足在可视范围内且不产生碰撞的节点才是合格的节点
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            // 确定新扩展的节点不在close list中，或者没有在之前遍历过
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value：更新cost-so-far
              // 更新合格点的G值
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              // 如果扩展节点不在OPEN LIST中，或者找到了更短G值的路径，或者该节点索引与前一节点在同一网格中（用新点代替旧点）
              // 则进入更新新点的属性：cost-to-go，
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                // calculate H value: 更新cost-to-go
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;//如果下一个点仍在相同的cell、并且cost变大，那继续找
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                // 如果下一节点仍在相同的cell, 但是代价值要小，则用当前successor替代前一个节点（这里仅更改指针，数据留在内存中）
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());//如果下一个点仍在相同的cell、并且cost变小，成功
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";//给出原地踏步的提示
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;//将生成的子节点加入到open list中
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                2D A*
// 原始A*算法，返回start到goal的cost-so-far
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  // 将open list和close list重置
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;//Open list, 注意是一个heap
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();//从Open集合中找出代价最低的元素
    // set index
    iPred = nPred->setIdx(width);//相应的index

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {//检查：如果已扩展，则从open set中移除，处理下一个
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {//没有进行扩展
      // add node to closed list
      nodes2D[iPred].close();//标记为close
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();//返回G值
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {//非目标点，则从可能的方向寻找
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {//A*算法是8个方向：4个正方向和4个45度的方向
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          // 约束性检查：在有效网格范围内、且不是障碍、没有扩展过
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            //更新G值
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            // 如果子节点并在open集中，或者它的G值(cost-so-far)比之前要小，则为可行的方向
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);//计算H值
              // put successor on open list
              nSucc->open();//将该点移到open set中
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
// 计算到目标的启发值(cost)
// 这里的cost由三项组成：《Practical Search Techniques in Path Planning for Autonomous Driving》
// 1) "non-holonomic-without-obstacles" heuristic:（用于指导搜索向目标方向前进）
//    受运动学约束的无障碍启发式值。论文的计算建议为： max(Reed-Shepp距离/Dubins距离, 欧氏距离) 表示
//    至于用Reed-Shepp距离还是Dubins距离取决于车辆是否可倒退
// 2) "holonomic-with-obstacles" heuristic：（用于发现U形转弯(U-shaped obstacles)/死路(dead-ends)）
//    （不受运动学约束的）有障约束启发式值(即：A*)
// 注1： 实际计算时，优先考虑运动学启发式值，A*作为可选项。至于是否启用欧氏距离和A*的启发式值，取决于计算
//      的精度和CPU性能（可作为调优手段之一）
// 注2： 实际计算与论文中的描述存在差异：
//      （1）实际计算的第一步用的启发式值为“Reed-Shepp距离/Dubins距离”，而论文为“max(Reed-Shepp距离/Dubins距离, 欧氏距离)”
//      （2）实际计算的第二步用的启发式值为A*的启发式值 减去 “start与goal各自相对自身所在2D网格的偏移量(二维向量)的欧氏距离”
//          该步计算的意义还需仔细分析，目前我还没想明白代码这样设计的理由。
void  updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height,
 CollisionDetection& configurationSpace, Visualize& visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins) {

    // ONLY FOR dubinsLookup
    //    int uX = std::abs((int)goal.getX() - (int)start.getX());
    //    int uY = std::abs((int)goal.getY() - (int)start.getY());
    //    // if the lookup table flag is set and the vehicle is in the lookup area
    //    if (Constants::dubinsLookup && uX < Constants::dubinsWidth - 1 && uY < Constants::dubinsWidth - 1) {
    //      int X = (int)goal.getX() - (int)start.getX();
    //      int Y = (int)goal.getY() - (int)start.getY();
    //      int h0;
    //      int h1;

    //      // mirror on x axis
    //      if (X >= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);
    //      }
    //      // mirror on y axis
    //      else if (X <= 0 && Y >= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);

    //      }
    //      // mirror on xy axis
    //      else if (X <= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.getT()) / Constants::deltaHeadingRad);

    //      } else {
    //        h0 = (int)(t / Constants::deltaHeadingRad);
    //        h1 = (int)(goal.getT() / Constants::deltaHeadingRad);
    //      }

    //      dubinsCost = dubinsLookup[uX * Constants::dubinsWidth * Constants::headings * Constants::headings
    //                                + uY *  Constants::headings * Constants::headings
    //                                + h0 * Constants::headings
    //                                + h1];
    //    } else {

    /*if (Constants::dubinsShot && std::abs(start.getX() - goal.getX()) >= 10 && std::abs(start.getY() - goal.getY()) >= 10)*/
    //      // start
    //      double q0[] = { start.getX(), start.getY(), start.getT()};
    //      // goal
    //      double q1[] = { goal.getX(), goal.getY(), goal.getT()};
    //      DubinsPath dubinsPath;
    //      dubins_init(q0, q1, Constants::r, &dubinsPath);
    //      dubinsCost = dubins_path_length(&dubinsPath);
    
    //这里改用open motion planning library的算法
    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // if reversing is active use a Reeds-Shepp 
  //假如车子可以后退，则可以启动Reeds-Shepp 算法
  if (Constants::reverse && !Constants::dubins) {
    //    ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(
      //调用A*算法，返回cost-so-far, 并在2D网格中设置相应的代价值
      aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization)
      );
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }

  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt( ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * 
                       ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                       ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * 
                       ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY()))
                      );
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
    //getG()返回A*的启发式代价，twoDoffset为 start与goal各自相对自身所在2D网格的偏移量的欧氏距离

  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));//将两个代价中的最大值作为启发式值
  //注：虽然这里有三个数值，但Dubins Cost的启用和Reeds-Shepp Cost的启用是互斥的，所以实际上是计算两种cost而已
}

//###################################################
//                    DUBINS SHOT
//###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  while (x <  length) {//这是跳出循环的条件之一：生成的路径没有达到所需要的长度
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    //跳出循环的条件之二：生成的路径存在碰撞节点
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    } 
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  //返回末节点，通过getPred()可以找到前一节点。
  return &dubinsNodes[i - 1];
}
