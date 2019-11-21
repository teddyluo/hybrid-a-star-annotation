/**
 * @file bucketedqueue.cpp
 * @brief 该文件是Dynamic Voronoi Diagram的一部分，定义和实现Priority Queue
 * 从注释上看，
 * 1）PriorityQueue使用存储桶将具有相同优先级的元素进行分组，同一个桶的元素
 * 无须进行排序；使用存储桶的优势在于当元素很大的时候效率很高。
 * 2）存储桶的优先次序是通过平方欧氏距离(取整)进行比较运算的
 * 
 * @date 2019-11-19 
 */

#include "bucketedqueue.h"

#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

using namespace HybridAStar;

std::vector<int> BucketPrioQueue::sqrIndices;
int BucketPrioQueue::numBuckets;


BucketPrioQueue::BucketPrioQueue() {
  // make sure the index array is created
  if (sqrIndices.size()==0) initSqrIndices();
  nextBucket = INT_MAX;
    
  // now create the buckets
  //    buckets = std::vector<MyQueue2<INTPOINT> >(numBuckets);
  buckets = std::vector<std::queue<INTPOINT> >(numBuckets);

  // reset element counter 
  count = 0;
}

bool BucketPrioQueue::empty() {
  return (count==0);
}

/**
 * @brief 插入一个元素到优先队列中
 * 
 * @param prio 优先级
 * @param t  元素
 */
void BucketPrioQueue::push(int prio, INTPOINT t) {
  if (prio>=(int)sqrIndices.size()) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  int id = sqrIndices[prio]; //id取值在[0, 2*MAXDIST^2]之间
  if (id<0) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  buckets[id].push(t);//buckets[id]为同一个pri的元素集合
  //    printf("pushing %d with prio %d into %d. Next: %d\n", t.x, prio, id, nextBucket);
  if (id<nextBucket) nextBucket = id;
  //    printf("push new next is %d\n", nextBucket);
  count++;
}

INTPOINT BucketPrioQueue::pop() {
  int i;
  assert(count>0);
  //    printf("next: %d\n", nextBucket);
  for (i = nextBucket; i<(int)buckets.size(); i++) {
    if (!buckets[i].empty()) break;	//找到最小距离的一串数，跳出循环
  }
  assert(i<(int)buckets.size());//确保在有效范围
  nextBucket = i;
  //    printf("pop new next %d\n", nextBucket);
  count--;
  INTPOINT p = buckets[i].front();//取出队首元素
  buckets[i].pop();//从队列移除队头元素
  return p;
}


void BucketPrioQueue::initSqrIndices() {
  //    std::cout << "BUCKETQUEUE Starting to build the index array...\n";
  //  std::set<int> sqrNumbers;

  sqrIndices = std::vector<int>(2*MAXDIST*MAXDIST+1, -1);

  int count=0;
  for (int x=0; x<=MAXDIST; x++) {
    for (int y=0; y<=x; y++) {
      int sqr = x*x+y*y;// sqr的最大值为 MAXDIST^2 + MAXDIST^2 = 2*MAXDIST^2 
      sqrIndices[sqr] = count++;
    }
  }
  numBuckets = count;
  //    std::cout << "BUCKETQUEUE Done with building the index arrays.\n";
}
