/*
用于搜索无权图中最短路径的算法，基于深度优先遍历（DFS）实现。该算法找到一条从起点到终点的最短路径，并保证不会出现重复的路径。
✨代码分析
①  DistinctPathDFS()：构造函数，用于初始化 DistinctPathDFS 类的对象。参数 nodes 是所有节点的向量，start 是起点节点，end 是终点节点。
②  findPaths()：找到所有不同的最短路径。返回一个向量，其中包含所有不同的最短路径以及它们的长度。
③  expand()：深度优先遍历（DFS）扩展节点，用于搜索所有的可能路径。参数 node 是当前要扩展的节点，path 是当前路径及其访问过的节点和路径长度的结构体，max_steps 是扩展步数的限制。
*/
#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

#include "common.hpp"
#include "dijkstra.hpp"
#include "esdf_map.hpp"
#include "tree_node.hpp"

// double distance(Vector<3> from, Vector<3> to) { return (from - to).norm(); }
/*path_with_visited 结构体保存一条路径及其访问过的节点和路径长度，
DistinctPathDFS 类通过搜索所有的可能路径来找到所有不同的最短路径，
其中 expand 函数用于遍历所有的可能路径。*/
template<typename T>
struct path_with_visited {
  std::unordered_set<HeapNode<T>*> visited_nodes;// 保存已经访问过的节点
  std::vector<HeapNode<T>*> path;// 保存当前路径
  double path_length;// 保存路径长度
};

template<typename T>
static constexpr auto comparator_path_with_length =
  [](const path_with_length<T>& a, const path_with_length<T>& b) -> bool {
  return a.length < b.length;
};

template<class T>
class DistinctPathDFS {
 public:
  DistinctPathDFS(std::vector<HeapNode<T>*> nodes, HeapNode<T>* start,
                  HeapNode<T>* end);
  std::vector<path_with_length<T>> findPaths();// 找到所有不同的最短路径

 private:
  void expand(HeapNode<T>* node, path_with_visited<T> path,
              const int& max_steps); // DFS扩展节点
  std::vector<HeapNode<T>*> nodes_; // 所有节点
  HeapNode<T>* start_;// 起点
  HeapNode<T>* end_;//终点
  std::vector<path_with_visited<T>> distinct_paths; // 所有不同的最短路径
};


template<class T>
DistinctPathDFS<T>::DistinctPathDFS(std::vector<HeapNode<T>*> nodes,
                                    HeapNode<T>* start, HeapNode<T>* end) {
  nodes_ = nodes;
  start_ = start;
  end_ = end;
  // INFO("DistinctPathDFS created")
}

template<class T>
std::vector<path_with_length<T>> DistinctPathDFS<T>::findPaths() {
  // INFO("findPaths dfs begin")
  distinct_paths.clear();// 清空路径
  // for (int var = 0; var < nodes_.size(); ++var) {
  //   nodes_[var]->visited = false;
  // }

  // end_->visited = false;
  // start_->pvisited = false;

  // distinct_paths.push_back();
  // INFO("start added")
  // INFO("start is " << start_)
  // INFO("end is " << end_)
  path_with_visited<T> start_path;
  start_path.path = {};// 起点路径为空
  start_path.visited_nodes = {};// 起点未访问过
  start_path.path_length = 0;// 起点路径长度为0
  int max_steps = nodes_.size();// / 4.0;
  expand(start_, start_path, max_steps);// 从起点开始搜索
  std::vector<path_with_length<T>> paths;
  for (size_t i = 0; i < distinct_paths.size(); i++) {
    path_with_length<T> pwl;
    pwl.plan = distinct_paths[i].path;
    pwl.length = distinct_paths[i].path_length;
    paths.push_back(pwl); // 把所有不同的最短路径加入到列表中
  }
  // INFO("findPaths dfs end")
  return paths;
}

template<class T>
void DistinctPathDFS<T>::expand(HeapNode<T>* node,
                                path_with_visited<T> path_of_node,
                                const int& max_steps) {
  // HeapNode<T>* node = path_of_node.path.back();
  // INFO("expand " << node)
  if (path_of_node.visited_nodes.size() > max_steps) {
    // INFO("too much steps")
    return;// 如果扩展步数超出限制，返回
  }

  if (path_of_node.path.size() > 0) {
    path_of_node.path_length +=
      (path_of_node.path.back()->data - node->data).norm();
      // distance(path_of_node.path.back()->data, node->data);
  } // 计算路径长度

  path_of_node.path.push_back(node);
  path_of_node.visited_nodes.insert(node);

  if (node == end_) {// 如果到达终点，保存路径并返回
    distinct_paths.push_back(path_of_node);
    return;
  }

  for (const auto& nb : node->visibility_node_ids) {
    if (path_of_node.visited_nodes.count(nb.first) > 0) {
      // already visited
      // 如果已经访问过该节点，跳过
      continue;
    }


    // INFO("expand recursion")
    // path_of_node.path.push_back(nb.first);
    // path_of_node.visited_nodes.insert(nb.first);
    // INFO("\t size before expand " << path_of_node.path.size())
    expand(nb.first, path_of_node, max_steps);// 递归扩展下一个节点
    // INFO("\t size after expand " << path_of_node.path.size())
  }


  // for (const auto& nb : node->visibility_node_ids) {
  //   if (nb.first->visited || nb.first == end_) {
  //     // already visited or goal
  //     continue;
  //   }
  // }
}
