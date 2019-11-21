#ifndef COLLISIONLOOKUP
#define COLLISIONLOOKUP

#include "dubins.h"
#include "constants.h"

namespace HybridAStar {
namespace Lookup {

//###################################################
//                                      DUBINS LOOKUP
//###################################################
inline void dubinsLookup(float* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the Dubin's lookup table...";

  DubinsPath path;

  int width = Constants::dubinsWidth / Constants::cellSize;//这里是相对长度

  //  // increase the width by one to make it square
  //  if (width % 2 != 0) {
  //    width++;
  //  }

  const int headings = Constants::headings;//这里是将heading(朝向)进行分段的数量

  // start and goal vector
  double start[3];//分别表示 (x, y, theta)
  double goal[] = {0, 0, 0};

  // iterate over the X index of a grid cell
  for (int X = 0; X < width; ++X) {
    start[0] = X;//索引值：X-方向

    // iterate over the Y index of a grid cell
    for (int Y = 0; Y < width; ++Y) {
      start[1] = Y;//索引值：Y-方向

      // iterate over the start headings
      for (int h0 = 0; h0 < headings; ++h0) {
        start[2] = Constants::deltaHeadingRad * h0;//分辨率乘以相对值为真实的角度
        //这里是角度， start=(X, Y, Theta), X和Y是索引值，Theta是角度(degree)

        // iterate over the goal headings
        for (int h1 = 0; h1 < headings; ++h1) {
          //同上，不过这里是goal的角度: (0, 0, Theta)
          goal[2] = Constants::deltaHeadingRad * h1;

          // calculate the actual cost
          //依据最小转弯半径生成一条从start到goal的path??
          dubins_init(start, goal, Constants::r, &path);
          //存储到查找表中
          lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] = dubins_path_length(&path);

          if (DEBUG && lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] < sqrt(X * X + Y * Y) * 1.000001) {
            std::cout << X << " | " << Y << " | "
                      << Constants::deltaHeadingDeg* h0 << " | "
                      << Constants::deltaHeadingDeg* h1 << " length: "
                      << lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] << "\n";

          }
        }
      }
    }
  }

  std::cout << " done!" << std::endl;
}

//###################################################
//                                   COLLISION LOOKUP
//###################################################

// _____________
// SIGN FUNCTION:符号函数
inline int sign(double x) {
  if (x >= 0) { return 1; }
  else { return -1; }
}

// _________________________
// COLLISION LOOKUP CREATION
inline void collisionLookup(Constants::config* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the collision lookup table...";
  // cell size
  const float cSize = Constants::cellSize;//为1
  // bounding box size length/width
  const int size = Constants::bbSize;// 为 (车的面积+4)/cellsize

  struct point {//定义点的数据结构
    double x;
    double y;
  };

  // ______________________
  // VARIABLES FOR ROTATION
  //center of the rectangle
  point c;//表示方形中心
  point temp;//临时值
  // points of the rectangle
  point p[4];//四个点的坐标
  point nP[4];

  // turning angle
  double theta;//转向角

  // ____________________________
  // VARIABLES FOR GRID TRAVERSAL
  // vector for grid traversal //网格遍历的变量
  point t;//
  point start;//起点
  point end;//终点
  // cell index
  int X;//用于网格索引， X-方向
  int Y;//用于网格索引， Y-方向
  // t value for crossing vertical and horizontal boundary
  double tMaxX;// X-方向的t值
  double tMaxY;// Y-方向的t值
  // t value for width/heigth of cell
  double tDeltaX; //单元格宽度方向的t值
  double tDeltaY; //单元格高度方向的t值
  // positive or negative step direction
  int stepX;//步进方向，可为正亦可为负
  int stepY;
  // grid
  bool cSpace[size * size]; //size为网格数量(车的面积)，
  bool inside = false; //在里面
  int hcross1 = 0;
  int hcross2 = 0;

  // _____________________________
  // VARIABLES FOR LOOKUP CREATION 创建查找表的变量
  int count = 0;
  const int positionResolution = Constants::positionResolution;//每个位置的cell的分辨率，表示该cell下有多少个划分的小网格
  const int positions = Constants::positions;//实际就是 positionResolution * positionResolution，表示数量
  point points[positions];//转化为点表示

  // generate all discrete positions within one cell
  for (int i = 0; i < positionResolution; ++i) {
    for (int j = 0; j < positionResolution; ++j) {
      points[positionResolution * i + j].x = 1.f / positionResolution * j;
      points[positionResolution * i + j].y = 1.f / positionResolution * i;
    }//从左上方开始，给每个点的x、y赋值，实际上是计算出每个点的位置(偏移量)
  }


  for (int q = 0; q < positions; ++q) {
    // set the starting angle to zero;
    theta = 0;

    // set points of rectangle(cell中心为(size/2, size/2)，加上偏移量是在分辨率下的小网格的位置)
    c.x = (double)size / 2 + points[q].x;
    c.y = (double)size / 2 + points[q].y;

    p[0].x = c.x - Constants::length / 2 / cSize; //车的左上角位置
    p[0].y = c.y - Constants::width / 2 / cSize;

    p[1].x = c.x - Constants::length / 2 / cSize;//车的左下角位置
    p[1].y = c.y + Constants::width / 2 / cSize;

    p[2].x = c.x + Constants::length / 2 / cSize;//车的右下角位置
    p[2].y = c.y + Constants::width / 2 / cSize;

    p[3].x = c.x + Constants::length / 2 / cSize;//车的右上角位置
    p[3].y = c.y - Constants::width / 2 / cSize;

    for (int o = 0; o < Constants::headings; ++o) {//将朝向划分为72份(离散化)
      if (DEBUG) { std::cout << "\ndegrees: " << theta * 180.f / M_PI << std::endl; }

      // initialize cSpace
      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          cSpace[i * size + j] = false;//初化为值为false，表示？
        }
      }

      // shape rotation
      for (int j = 0; j < 4; ++j) {
        // translate point to origin
        temp.x = p[j].x - c.x;//以车的中心为坐标原点
        temp.y = p[j].y - c.y;

        // rotate and shift back:将车旋转theta角度
        nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
        nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
      }//nP存储的是车旋转theta角度的坐标

      // create the next angle 下个角度(360度分成72份，步长是: deltaHeadingRad=2*pi/72)
      theta += Constants::deltaHeadingRad;

      // cell traversal clockwise
      for (int k = 0; k < 4; ++k) {
        // create the vectors clockwise
        if (k < 3) {
          start = nP[k];
          end = nP[k + 1];
        } else {
          start = nP[k];
          end = nP[0];
        }//找到始点、终点

        //set indexes
        X = (int)start.x;
        Y = (int)start.y;
        //      std::cout << "StartCell: " << X << "," << Y << std::endl;
        cSpace[Y * size + X] = true;
        t.x = end.x - start.x;
        t.y = end.y - start.y;//t为从始点到终点的向量
        stepX = sign(t.x);//方向
        stepY = sign(t.y);

        // width and height normalized by t
        if (t.x != 0) {
          tDeltaX = 1.f / std::abs(t.x);
        } else {
          tDeltaX = 1000;
        }

        if (t.y != 0) {
          tDeltaY = 1.f / std::abs(t.y);
        } else {
          tDeltaY = 1000;
        }

        // set maximum traversal values 设置行走值，即离一个cell偏离了多少(取值)
        if (stepX > 0) {
          tMaxX = tDeltaX * (1 - (start.x - (long)start.x));
        } else {
          tMaxX = tDeltaX * (start.x - (long)start.x);
        }

        if (stepY > 0) {
          tMaxY = tDeltaY * (1 - (start.y - (long)start.y));
        } else {
          tMaxY = tDeltaY * (start.y - (long)start.y);
        }

        while ((int)end.x != X || (int)end.y != Y) {
          // only increment x if the t length is smaller and the result will be closer to the goal
          if (tMaxX < tMaxY && std::abs(X + stepX - (int)end.x) < std::abs(X - (int)end.x)) {
            tMaxX = tMaxX + tDeltaX;//更新
            X = X + stepX;
            cSpace[Y * size + X] = true;//标记已走
            // only increment y if the t length is smaller and the result will be closer to the goal
          } else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)end.y) < std::abs(Y - (int)end.y)) {
            tMaxY = tMaxY + tDeltaY;
            Y = Y + stepY;
            cSpace[Y * size + X] = true;
          } else if (2 >= std::abs(X - (int)end.x) + std::abs(Y - (int)end.y)) {
            if (std::abs(X - (int)end.x) > std::abs(Y - (int)end.y)) {
              X = X + stepX;
              cSpace[Y * size + X] = true;
            } else {
              Y = Y + stepY;
              cSpace[Y * size + X] = true;
            }
          } else {
            // this SHOULD NOT happen
            std::cout << "\n--->tie occured, please check for error in script\n";
            break;
          }
        }
      }

      // FILL THE SHAPE
      for (int i = 0; i < size; ++i) {
        // set inside to false
        inside = false;

        for (int j = 0; j < size; ++j) {
          // determine horizontal crossings
          for (int k = 0; k < size; ++k) {
            if (cSpace[i * size + k] && !inside) {
              hcross1 = k;
              inside = true;
            }

            if (cSpace[i * size + k] && inside) {
              hcross2 = k;
            }
          }

          // if inside fill
          if (j > hcross1 && j < hcross2 && inside) {
            cSpace[i * size + j] = true;
          }
        }
      }

      // GENERATE THE ACTUAL LOOKUP
      count = 0;//生成查找表

      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          if (cSpace[i * size + j]) {
            // compute the relative position of the car cells
            lookup[q * Constants::headings + o].pos[count].x = j - (int)c.x;
            lookup[q * Constants::headings + o].pos[count].y = i - (int)c.y;
            // add one for the length of the current list
            count++;
          }
        }
      }

      lookup[q * Constants::headings + o].length = count;

      if (DEBUG) {
        //DEBUG
        for (int i = 0; i < size; ++i) {
          std::cout << "\n";

          for (int j = 0; j < size; ++j) {
            if (cSpace[i * size + j]) {
              std::cout << "#";
            } else {
              std::cout << ".";
            }
          }
        }

        //TESTING
        std::cout << "\n\nthe center of " << q* Constants::headings + o << " is at " << c.x << " | " << c.y << std::endl;

        for (int i = 0; i < lookup[q * Constants::headings + o].length; ++i) {
          std::cout << "[" << i << "]\t" << lookup[q * Constants::headings + o].pos[i].x << " | " << lookup[q * Constants::headings + o].pos[i].y << std::endl;
        }
      }
    }
  }

  std::cout << " done!" << std::endl;
}

}
}
#endif // LOOKUP

