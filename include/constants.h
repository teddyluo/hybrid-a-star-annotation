#ifndef CONSTANTS
#define CONSTANTS
/*!
   \file constants.h
   \brief This is a collection of constants that are used throughout the project.
   \todo All constants need to be checked and documented
*/

////###################################################
////                      INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid
/**
 * 约定: 
 *     HEADING： [0, 359]度，0表示朝向北(指向Y)
 *     X - 表示网格的宽度
 *     Y - 表示网格的高度
 */
#include <cmath>

/*!
    \brief The namespace that wraps the entire project
    \namespace HybridAStar
*/

namespace HybridAStar {
/*!
    \brief The namespace that wraps constants.h
    \namespace Constants
*/
namespace Constants {
// _________________
// CONFIG FLAGS

/// A flag for additional debugging output via `std::cout`
static const bool coutDEBUG = false; //调试开关
/// A flag for the mode (true = manual; false = dynamic). Manual for static map or dynamic for dynamic map.
static const bool manual = true;//true表示静态地图；false表示动态地图
/// A flag for the visualization of 3D nodes (true = on; false = off)
static const bool visualization = false && manual;//可视化开关
/// A flag for the visualization of 2D nodes (true = on; false = off)
static const bool visualization2D = false && manual; //在2D视图上可视化
/// A flag to toggle reversing (true = on; false = off)
static const bool reverse = true; //true表示可以倒退；false表示只能前进不能倒退
/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
static const bool dubinsShot = true; //切换Dubin路径的开关
/// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
static const bool dubins = false;//Dubin路径的切换开关: 若车子可以倒退，值为false
/*!
   \var static const bool dubinsLookup
   \brief A flag to toggle the Dubin's heuristic via lookup, potentially speeding up the search by a lot
   \todo not yet functional
*/
static const bool dubinsLookup = false && dubins;
/// A flag to toggle the 2D heuristic (true = on; false = off)
static const bool twoD = true;

// _________________
// GENERAL CONSTANTS

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
static const int iterations = 30000; //最大迭代次数
/// [m] --- Uniformly adds a padding around the vehicle
static const double bloating = 0; //膨胀范围
/// [m] --- The width of the vehicle
static const double width = 1.75 + 2 * bloating;//车的宽度
/// [m] --- The length of the vehicle
static const double length = 2.65 + 2 * bloating;//车的长度
/// [m] --- The minimum turning radius of the vehicle
static const float r = 6;//最小转弯半径
/// [m] --- The number of discretizations in heading
static const int headings = 72;//车体朝向的离散数量
/// [°] --- The discretization value of the heading (goal condition)
static const float deltaHeadingDeg = 360 / (float)headings; //朝向离散步长(以度表示)
/// [c*M_PI] --- The discretization value of heading (goal condition)
static const float deltaHeadingRad = 2 * M_PI / (float)headings; //朝向离散步长(以弧度表示)
/// [c*M_PI] --- The heading part of the goal condition 
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
/// [m] --- The cell size of the 2D grid of the world
static const float cellSize = 1; //在2D网格中cell的大小
/*!
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell


  As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that 
  the algorithm would prefer the predecessor rather than the successor.
  This would lead to the fact that the successor would never be placed and the the one 
  cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
  to allow the successor being placed in the same cell.
*/
/**
 * 如果cost-so-far的启发式值比cost-to-come的启发式值更大时，算法应选择predecessor而不是successor。
 * 这样会导致successor从不会被选择的情况发生，该单元格永远只能扩展一个节点。tieBreaker可以人为地增加
 * predecessor的代价，允许successor放置在同一个单元中。它的使用见algorithm.cpp, 
 *     if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) 
 */
static const float tieBreaker = 0.01;

// ___________________
// HEURISTIC CONSTANTS

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
static const float factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
static const float penaltyTurning = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
static const float penaltyCOD = 2.0;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
static const float dubinsShotDistance = 100;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
static const float dubinsStepSize = 1;


// ______________________
// DUBINS LOOKUP SPECIFIC

/// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
static const int dubinsWidth = 15;//Dubin搜索区域的宽度
/// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
static const int dubinsArea = dubinsWidth * dubinsWidth; //Dubin搜索区域的面积


// _________________________
// COLLISION LOOKUP SPECIFIC

/// [m] -- The bounding box size length and width to precompute all possible headings
//用于预计算所有可能的转向的框的大小
static const int bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);
/// [#] --- The sqrt of the number of discrete positions per cell
static const int positionResolution = 10;//每个cell里的离散位置数量的平方根
/// [#] --- The number of discrete positions per cell
static const int positions = positionResolution * positionResolution;//位置的数量
/// A structure describing the relative position of the occupied cell based on the center of the vehicle
struct relPos {//相对于中心的位置：即以中心为坐标原点
  /// the x position relative to the center
  int x; 
  /// the y position relative to the center
  int y;
};
/// A structure capturing the lookup for each theta configuration
struct config {//用以获取每个theta的查找表的结构体
  /// the number of cells occupied by this configuration of the vehicle
  int length;//长度，
  /*!
     \var relPos pos[64]
     \brief The maximum number of occupied cells
     \todo needs to be dynamic
  */
  relPos pos[64];//这里为什么是64有待考证
};

// _________________
// SMOOTHER SPECIFIC路径平滑
/// [m] --- The minimum width of a safe road for the vehicle at hand
//当前车的最小行驶宽度
static const float minRoadWidth = 2;

//下面的结构体用于定义颜色以可视化
// ____________________________________________
// COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
/// A structure to express colors in RGB values
struct color {
  /// the red portion of the color
  float red;
  /// the green portion of the color
  float green;
  /// the blue portion of the color
  float blue;
};
// 定义了五种不同的颜色
/// A definition for a color used for visualization
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
/// A definition for a color used for visualization
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
/// A definition for a color used for visualization
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
/// A definition for a color used for visualization
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
/// A definition for a color used for visualization
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
}
}

#endif // CONSTANTS

