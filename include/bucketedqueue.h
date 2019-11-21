/**
 * @file bucketedqueue.h
 * @brief 该文件是Dynamic Voronoi Diagram的一部分，定义和实现Priority Queue
 * 从注释上看，
 * 1）PriorityQueue使用存储桶将具有相同优先级的元素进行分组，同一个桶的元素
 * 无须进行排序；使用存储桶的优势在于当元素很大的时候效率很高。
 * 2）存储桶的优先次序是通过平方欧氏距离(取整)进行比较运算的
 * 
 * @date 2019-11-19 
 */

#ifndef _PRIORITYQUEUE2_H_
#define _PRIORITYQUEUE2_H_

#define MAXDIST 1000
#define RESERVE 64

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include "point.h"

namespace HybridAStar {
//! Priority queue for integer coordinates with squared distances as priority.
/** A priority queue that uses buckets to group elements with the same priority.
    The individual buckets are unsorted, which increases efficiency if these groups are large.
    The elements are assumed to be integer coordinates, and the priorities are assumed
    to be squared euclidean distances (integers).
*/
class BucketPrioQueue {

 public:
  //! Standard constructor
  /** Standard constructor. When called for the first time it creates a look up table
      that maps square distanes to bucket numbers, which might take some time...
  */
  BucketPrioQueue();//构造函数
  //! Checks whether the Queue is empty
  bool empty();//检验队列是否为空
  //! push an element
  void push(int prio, INTPOINT t);//插入元素
  //! return and pop the element with the lowest squared distance */
  INTPOINT pop();

 private:

  static void initSqrIndices();
  static std::vector<int> sqrIndices;//记录pri的位置，sqrIndices[pri]=index
  static int numBuckets;
  int count;//有效元素数量
  int nextBucket;

  std::vector<std::queue<INTPOINT> > buckets;
};
}
#endif
