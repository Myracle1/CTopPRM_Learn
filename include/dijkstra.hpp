/*
实现 Dijkstra 算法的 C++ 类，用于在加权图中找到从源节点到所有其他节点的最短路径
✨代码分析
①  path_with_length：保存路径信息，包括路径上的节点和路径长度。
②  HeapNode：堆中使用的节点，包括节点 ID、距离起点的距离、前一个节点、是否是城市节点等信息。
③  Heap：使用 std::vector 实现的最小堆，用于进行 Dijkstra 算法中的优先队列操作。
④  Dijkstra：Dijkstra 算法的主类，包括寻找最短路径的函数 findPath 和扩展节点的函数expandBestNode。
⑤  findPath：通过传入起点和多个终点，返回一组路径信息，包括路径上经过的节点、路径长度等。
⑥  expandBestNode：从堆中取出距离起点最近的节点，并更新与其相邻的节点的距离信息。
 */

#pragma once

#include <algorithm>
#include <vector>

#include "common.hpp"
#include "float.h"
#include "heap.hpp"

#define DIJKSTRA_INF DBL_MAX

template <typename T> struct path_with_length {
  std::vector<HeapNode<T> *> plan;   // 计划路径上的节点
  double length;   // 路径长度
  int from_id;  // 起始节点 ID
  int to_id;  // 终止节点 ID
  double calc_path_length() const {  // 计算路径长度的函数
    double length_together = 0;
    // std::cout << new_path.plan[0]->id << " ";
    for (size_t nppi = 1; nppi < plan.size(); nppi++) {
      // std::cout << plan[nppi]->id << " ";
      length_together += (plan[nppi]->data - plan[nppi - 1]->data).norm();
      // 使用欧几里得距离计算节点间距离
      //(new_path.plan[nppi]->data - new_path.plan[nppi - 1]->data).norm();
    }
    return length_together;
  };
  void print() {
    // 打印路径上的节点
    std::cout << "plan:" << std::endl;
    for (size_t nppi = 0; nppi < plan.size(); nppi++) {
      std::cout << plan[nppi]->id << " ";
    }
    std::cout << std::endl;
  };
};

// template<typename T>
// struct path_with_length {
//   std::vector<HeapNode<T>*> path;
//   double path_length;
// };

template <class T> class Dijkstra {
public:
  Dijkstra();
  virtual ~Dijkstra();
  // plan_with_length findPath(int start_index, int goal_index,
  // std::vector<HeapNode*> & connectedPoints);
  std::vector<path_with_length<T>>
  findPath(int start_index, std::vector<int> goal_indexes,
           std::vector<HeapNode<T> *> &connectedPoints);
            // 寻找最短路径的函数

private:
  HeapNode<T> *expandBestNode(); // 扩展最优节点的函数
  Heap<HeapNode<T> *> *heap; // 堆数据结构，用于优先队列操作
};

template <class T>
std::ostream &operator<<(std::ostream &o, const HeapNode<T> &p); // 重载输出运算符，用于输出 HeapNode 对象


template <typename T> Dijkstra<T>::Dijkstra() { heap = NULL; }

template <typename T> Dijkstra<T>::~Dijkstra() {
  if (heap != NULL) {
    delete heap;
  }
}

template <typename T>
std::vector<path_with_length<T>>
Dijkstra<T>::findPath(int start_index, std::vector<int> goal_indexes,
                      std::vector<HeapNode<T> *> &visibility_graph) {
  // INFO("begin Dijkstra::findPath()");
  /*********** init dijkstra *************/
   // 初始化 Dijkstra 算法
  for (int var = 0; var < visibility_graph.size(); ++var) {
    visibility_graph[var]->distance_from_start = DIJKSTRA_INF;// 设置节点到起点的距离为无穷大
    visibility_graph[var]->previous_point = NULL;// 前一个节点为空
  }
  visibility_graph[start_index]->distance_from_start = 0; // 起点到起点的距离为0
  visibility_graph[start_index]->previous_point = visibility_graph[start_index]; // 起点的前一个节点为自身

  // need to create heap after setting points
  heap = new Heap<HeapNode<T> *>(visibility_graph);// 创建堆对象，用于进行优先队列操作
  bool stopped = false;
  HeapNode<T> *actualBestNode;
  std::set<int> goal_indexes_non_visited(goal_indexes.begin(),
                                         goal_indexes.end()); // 未访问的终点集合
  int numLooped = 0;
  // int loopCounter = 0;
  // INFO("loop points");
  while (!stopped) {
    actualBestNode = expandBestNode(); // 扩展距离起点最近的节点

    if (actualBestNode == NULL ||
        actualBestNode->distance_from_start == DIJKSTRA_INF) {
      stopped = true;// 无法找到最短路径，停止算法
      // INFO("DIJKSTRA not found")
    }

    if (!stopped && actualBestNode->city_node) {
      // can be one of goal
      auto search = goal_indexes_non_visited.find(actualBestNode->id);
      if (search != goal_indexes_non_visited.end()) {
        goal_indexes_non_visited.erase(search); // 标记终点已访问
      }

      if (goal_indexes_non_visited.empty()) {
        stopped = true;// 所有终点都已访问，停止算法
      }
    }

    numLooped++;
    /*
    loopCounter++;
    if ( loopCounter >= 1000 ) {
      loopCounter = 0;
      //INFO("looped " << numLooped << " points");
    }
    */
  }

  delete heap;// 释放堆对象
  heap = NULL;

  std::vector<path_with_length<T>> plans_w_length;

  for (int var = 0; var < goal_indexes.size(); ++var) {
    std::vector<HeapNode<T> *> plan;
    HeapNode<T> *actualPoint = visibility_graph[goal_indexes[var]];
    // INFO("createPlan from goal " << actualPoint<< " node id
    // "<<actualPoint->node_id <<" is city "<<actualPoint->city_node);
    if (actualPoint->previous_point != NULL) { // 如果存在路径，则构建路径信息
      plan.push_back(actualPoint);
      while (actualPoint != visibility_graph[start_index]) {
        // WINFO(actualPoint << " get previous point ");
        actualPoint = actualPoint->previous_point;
        // INFO("add point " << actualPoint<< " node id "<<actualPoint->node_id
        // <<" is city "<<actualPoint->city_node);
        plan.push_back(actualPoint);
      }
      std::reverse(plan.begin(), plan.end());
      path_with_length<T> plan_w_length;
      plan_w_length.length =
          visibility_graph[goal_indexes[var]]->distance_from_start;
      plan_w_length.plan = plan;
      plan_w_length.from_id = start_index;
      plan_w_length.to_id = goal_indexes[var];
      plans_w_length.push_back(plan_w_length);
    } else {// 如果不存在路径，则路径长度设为无穷大
      path_with_length<T> plan_w_length;
      plan_w_length.length = DIJKSTRA_INF;
      plan_w_length.plan = plan;
      plan_w_length.from_id = start_index;
      plan_w_length.to_id = goal_indexes[var];
      plans_w_length.push_back(plan_w_length);
    }
  }
  // INFO("end Dijkstra::findPath()");

  return plans_w_length; // 返回路径信息
}

template <typename T> HeapNode<T> *Dijkstra<T>::expandBestNode() {
  HeapNode<T> *expandingNode = heap->pop();// 从堆中取出距离起点最近的节点

  if (expandingNode != NULL) {
    for (auto connectedNode : expandingNode->visibility_node_ids) {
      double calculated_new_distance =
          expandingNode->distance_from_start + connectedNode.second;

      if (calculated_new_distance < connectedNode.first->distance_from_start) {
        // test point if better distance found
        connectedNode.first->previous_point = expandingNode;// 更新节点的前一个节点
        heap->updateCost(connectedNode.first, calculated_new_distance);// 更新节点在堆中的代价值
      }
      }
    }
  }
  return expandingNode;
}
