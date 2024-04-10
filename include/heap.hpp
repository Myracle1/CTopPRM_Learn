/*
实现了一个通用的堆数据结构，用于存储和处理元素对象，如优先队列、最小/最大堆等
 */

#pragma once
#include <cmath>
#include <map>
#include <set>

#include "common.hpp"
#include "vector"

// using logger;

template <typename T> struct HeapNode;

template <typename T> struct HeapNode {
   // 默认初始化的构造函数
  HeapNode() {
    id = 0;
    this->data = T();
    previous_point = NULL;
    distance_from_start = 0;
    heap_position = 0;
    cluster_id = -1;
    is_border = false;
  }
   // 带数据初始化的构造函数
  HeapNode(T _data) {
    id = 0;
    this->data = _data;
    previous_point = NULL;
    distance_from_start = 0;
    heap_position = 0;
    cluster_id = -1;
    is_border = false;
  }

  bool city_node = false;// 表示是否是城市节点
  int id;// 节点的唯一标识符
  T data;// 节点存储的数据
  int cluster_id = -1;// 节点所属的簇的标识符，默认为-1
  bool is_border = false;// 节点是否是边界点，默认为false
  // pointer and distance to neighbor nodes
  std::unordered_map<HeapNode<T> *, double> visibility_node_ids;// 存储邻居节点指针及与邻居节点之间的距离
  std::multimap<double, HeapNode<T> *> dist_sorted_visibility_node_ids;// 存储按距离排序的邻居节点指针
  double distance_from_start;// 节点到起始点的距离
  HeapNode<T> *previous_point;// 前驱节点指针
  int heap_position; // bool visited;节点在堆中的位置
};

//#define HEAPCOST_GET(i) heapVector[i]->getDistanceFromStart()
//#define HEAPCOST_SET(i,cost) heapVector[i]->setDistanceFromStart( cost )
// 计算左孩子、右孩子和父节点索引的宏
#define LCHILD(x) 2 * x + 1
#define RCHILD(x) 2 * x + 2
#define PARENT(x) (x - 1) / 2

// 访问和修改 HeapNode 的 distance_from_start 宏函数
#define HEAPCOST_GET(i) (heapVector[i]->distance_from_start)
#define HEAPCOST_SET(i, cost)                                                  \
  { heapVector[i]->distance_from_start = cost; }
#define HEAP_SET_ELEMENT_POSITION(element, i)                                  \
  { element->heap_position = i; }
#define HEAP_GET_ELEMENT_POSITION(element) (element->heap_position)

template <class T> class Heap {
public:
  Heap();// 默认构造函数
  Heap(T *array, int arrayLength); // 使用数组和长度初始化堆
  Heap(std::vector<T> data);// 使用向量初始化堆
  ~Heap();// 析构函数
  void push(T newValue); // 将新元素添加到堆中
  void updateCost(int position, double cost);// 更新指定位置节点的代价
  void updateCost(T value, double cost);// 更新指定值的节点的代价
  T pop();// 弹出并返回堆顶元素
  T get(); // 获取堆顶元素，不删除
  T get(int id);// 获取指定 ID 的元素
  void replace(int id, T newValue);// 替换指定 ID 的元素为新值
  int size();
  void clear();
  bool empty();
  void remove(int id); // 根据 ID 移除元素
  bool checkOrdering();// 检查堆是否满足顺序性质
  bool checkIndex(int index);
  std::vector<T> getHeapVector();// 获取底层表示堆的向量

private:
  void sort();// 对堆进行排序
  void BubbleDown(int index);// 从指定索引开始向下冒泡
  void BubbleUp(int index);// 从指定索引开始向上冒泡
  std::vector<T> heapVector;// 存储堆元素的向量
};

// Heap 类方法的实现
template <class T> Heap<T>::Heap() {}

template <class T>
Heap<T>::Heap(T *array, int arrayLength) : heapVector(arrayLength) {
  for (int var = 0; var < arrayLength; ++var) {
    heapVector[var] = array[var];
    HEAP_SET_ELEMENT_POSITION(heapVector[var], var);
  }
}

template <class T> Heap<T>::Heap(std::vector<T> data) : heapVector(data) {
  for (int var = 0; var < data.size(); ++var) {
    HEAP_SET_ELEMENT_POSITION(heapVector[var], var);
  }
  sort();
}

template <class T> Heap<T>::~Heap() {
  /*
  //if deleting the pointers is ever needed
  for (auto p : heapVector) {
    delete p;
  }*/
}

template <class T> void Heap<T>::sort() {
  int size = this->heapVector.size();
  for (int var = size - 1; var >= 0; --var) {
    BubbleDown(var);
  }
}

template <class T> void Heap<T>::BubbleDown(int index) {
  int size = heapVector.size();

  int leftChildIndex = LCHILD(index);
  int rightChildIndex = RCHILD(index);
  if (leftChildIndex >= size) {
    return; // index is a leaf，index 是叶节点
  }
  int minIndex = index;
  if (HEAPCOST_GET(index) > HEAPCOST_GET(leftChildIndex)) {
    minIndex = leftChildIndex;
  }

  if ((rightChildIndex < size) &&
      (HEAPCOST_GET(minIndex) > HEAPCOST_GET(rightChildIndex))) {
    minIndex = rightChildIndex;
  }

  if (minIndex != index) {
    // need swap，交换
    T temp = heapVector[index];
    heapVector[index] = heapVector[minIndex];
    HEAP_SET_ELEMENT_POSITION(heapVector[index], index);
    heapVector[minIndex] = temp;
    HEAP_SET_ELEMENT_POSITION(heapVector[minIndex], minIndex);
    BubbleDown(minIndex);
  }
}

template <class T> void Heap<T>::BubbleUp(int index) {
  if (index == 0)
    return;

  int parentIndex = PARENT(index); //(index - 1) / 2;
  if (HEAPCOST_GET(parentIndex) > HEAPCOST_GET(index)) {
    T temp = heapVector[parentIndex];
    heapVector[parentIndex] = heapVector[index];
    HEAP_SET_ELEMENT_POSITION(heapVector[parentIndex], parentIndex);
    heapVector[index] = temp;
    HEAP_SET_ELEMENT_POSITION(heapVector[index], index);
    BubbleUp(parentIndex);// 递归调用 BubbleUp 方法，将当前元素向上冒泡直到找到正确位置
  }
}

template <class T> void Heap<T>::push(T newValue) {
  int newIndex = heapVector.size();
  heapVector.push_back(newValue);
  HEAP_SET_ELEMENT_POSITION(newValue, newIndex);
  BubbleUp(newIndex);// 调用 BubbleUp 方法，将新元素放置在正确的位置
}

template <class T> T Heap<T>::pop() {
  T min = NULL;// 定义一个变量存储堆顶元素的值
  const int size = heapVector.size();
  // WVARIABLE(size);
  if (size > 0) {
    min = this->get(); // 获取堆顶元素的值
    // WINFO("pos before "<<HEAP_GET_ELEMENT_POSITION(heapVector[size - 1]));
    heapVector[0] = heapVector[size - 1];// 将最后一个元素移到堆顶
    HEAP_SET_ELEMENT_POSITION(heapVector[0], 0);
    // WINFO("pos "<<HEAP_GET_ELEMENT_POSITION(heapVector[0]));
    heapVector.pop_back(); // 移除最后一个元素
    BubbleDown(0);// 调用 BubbleDown 方法，将新的堆顶元素放置在正确的位置
  } 
  // else {
  //   ERROR("empty heap");
  // }

  return min;// 返回堆顶元素的值
}

template <class T> T Heap<T>::get() { return heapVector[0]; }

template <class T> T Heap<T>::get(int id) { return heapVector[id]; }

template <class T> void Heap<T>::remove(int id) {
  const int size = heapVector.size();
  heapVector[id] = heapVector[size - 1];
  heapVector.pop_back();
  BubbleDown(id);
}

template <class T> void Heap<T>::replace(int id, T newValue) {
  double oldCost = HEAPCOST_GET(id);
  double newCost = newValue->getValue();
  heapVector[id] = newValue;
  HEAP_SET_ELEMENT_POSITION(heapVector[id], id);
  if (newCost < oldCost) {
    // zmenseni hodnoty -- chce to jit nahoru
    BubbleUp(id);
  } else {
    BubbleDown(id);
  }
}

template <class T> int Heap<T>::size() { return heapVector.size(); }

template <class T> void Heap<T>::updateCost(int position, double cost) {
  double oldCost = HEAPCOST_GET(position); // 获取给定位置的旧花费值
  HEAPCOST_SET(position, cost);// 更新给定位置的花费值
  if (oldCost > cost) {
    // zmenseni hodnoty -- chce to jit nahoru
    BubbleUp(position); // 如果新花费值小于旧花费值，调用 BubbleUp 方法，将元素向上冒泡直到找到正确位置
  } else {
    BubbleDown(position);// 否则调用 BubbleDown 方法，将元素放置在正确的位置
  }
}

template <class T> void Heap<T>::updateCost(T value, double cost) {
  int size = heapVector.size();
  // INFO("value->heap_position "<<value->heap_position);
  updateCost(value->heap_position, cost);// 根据元素的堆位置更新其花费值，并调整元素的位置维持堆的性质

  /*
   INFO("bad updateCost begin");
   for (int var = 0; var < size; ++var) {
   ERROR("bad update cost "<<var)
   if (heapVector[var] == value) {
   INFO("found right heap at pos "<<var)
   INFO("heap position "<<value->heap_position);
   updateCost(var, cost);
   break;
   }
   }
   INFO("bad updateCost end");
   */
}

template <class T> void Heap<T>::clear() { heapVector.clear(); }// 清空堆中的所有元素

template <class T> bool Heap<T>::empty() { return this->heapVector.empty(); }// 判断堆是否为空

template <class T> std::vector<T> Heap<T>::getHeapVector() {
  return this->heapVector;// 返回堆中所有元素的 vector
}

template <class T> bool Heap<T>::checkOrdering() { return checkIndex(0); }// 检查堆中元素是否符合堆序性质

template <class T> bool Heap<T>::checkIndex(int index) {
  if (index < heapVector.size()) {
    int lchild = LCHILD(index);
    int rchild = RCHILD(index);
    if (lchild < heapVector.size()) {
      if (HEAPCOST_GET(lchild) < HEAPCOST_GET(index)) {
        ERROR("heap inconsistency lchild" << lchild);// 检查左子节点是否小于父节点，如果不满足则输出错误信息
        exit(1);
      }
      checkIndex(lchild); // 递归检查左子树
    }
    if (rchild < heapVector.size()) {
      if (HEAPCOST_GET(rchild) < HEAPCOST_GET(index)) {// 检查右子节点是否小于父节点，如果不满足则输出错误信息
        ERROR("heap inconsistency rchild" << rchild);
        exit(1);
      }
      checkIndex(rchild); // 递归检查右子树
    }
  }
  return true;// 堆序性质满足
}
