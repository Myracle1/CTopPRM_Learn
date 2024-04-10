/*
该代码实现的是论文中V.RESULT部分用于比较的算法4：P-D-PRM
这篇论文是Path deformation roadmaps: Compact graphs with useful cycles for motion planning，发表于2008年
即具有路径扭曲和路径优化功能的PRM算法。
PDR在PRM的基础上，加入了路径扭曲和路径优化的步骤，
以提高路径的可行性和质量，适用于复杂环境下的路径规划问题。
使用的是PRM的变种：Informed PRM
*/

/*
✨代码分析
①sample_point 函数用于从地图中采样点，并将新采样的节点加入到规划器中。
②deform_path 函数用于扭曲路径，检查路径中的节点是否可以被替换为新的节点以减小路径长度。
③setBorders 函数用于设置地图边界。
④saveRoadmap/savePath函数用于将概率路图/路径保存到文件中。
⑤shorten_path 函数用于缩短路径，通过采样和检查碰撞来生成更短的路径。
⑥findDistinctPaths 函数用于查找不同的路径。
⑦testVisibilityGraph 函数用于测试可见图。
⑧removeEquivalentPaths 函数用于移除等效路径，即在多条路径中只保留一条最短的等效路径。
⑨removeTooLongPaths 函数用于移除过长的路径，即移除长度超过阈值的路径。
⑩find_geometrical_paths 函数用于找到几何路径，根据输入的参数进行路径搜索。
*/
#pragma once

#include <algorithm>
#include <flann/flann.hpp>
#include <limits>
#include <memory>
#include <vector>
#include <time.h>
#include <filesystem>

#include "common.hpp"
#include "esdf_map.hpp"
#include "base_map.hpp"
#include "tree_node.hpp"
#include "distinct_path_dfs.hpp"
#include "prm.hpp"
#include "dijkstra.hpp"

#define PRECISION_PRM_CLUSTERING (10e-6)
#define PRECISION (1E-4)

template<class T>
class pdr {
 public:
  // 构造函数，接受planner_config、map、start和end作为参数
  pdr(const YAML::Node& planner_config, std::shared_ptr<BaseMap> map,
                 T start, T end);

  void sample_point();// 从地图中采样点
  void deform_path();// 扭曲路径
  void setBorders(Vector<3> min_position, Vector<3> max_position);// 设置边界
  void saveRoadmap(std::string filename);// 将概率路图保存到文件中
  static void savePath(std::string filename, std::vector<HeapNode<T>*> path);// 将路径保存到文件中
  static path_with_length<T> shorten_path(path_with_length<T> path,// 缩短路径
                                          bool forward = true);
  std::vector<path_with_length<T>> findDistinctPaths();  // 查找不同的路径
  std::vector<path_with_length<T>> testVisibilityGraph();// 测试可见图
  static std::vector<std::vector<path_with_length<Vector<3>>>> // 找到几何路径
  find_geometrical_paths(const YAML::Node &planner_config,
                         std::shared_ptr<BaseMap> map,
                         std::vector<Vector<3>> &gates_with_start_end_poses,
                         std::string output_folder);
  static std::vector<path_with_length<T>>// 移除等效路径
  removeEquivalentPaths(std::vector<path_with_length<T>> paths);
  static std::vector<path_with_length<T>> // 移除过长的路径
  removeTooLongPaths(std::vector<path_with_length<T>> paths);

 private:

  static std::string to_string_raw(T data);// 将数据转为字符串
  // 计算两个点之间的距离
  static double distance(Vector<3> from, Vector<3> to) { return (from - to).norm(); }

  // 获取组件中可见的节点
  std::vector<HeapNode<T>*> getVisibleinComp(HeapNode<T>* node, int comp);
  void setConnectors();// 设置连接器
  std::vector<HeapNode<T>*> getVisibleGuards(HeapNode<T>* node);// 获取可见的保卫节点
  std::vector<HeapNode<T>*> getVisibleNodes(HeapNode<T>* node);// 获取可见的节点
  void TestVisibSubRoadmap(HeapNode<T>* node);// 测试可见子路网
  std::map<HeapNode<T>*, double> nodesBetweenNodes(HeapNode<T>* node1,
                                                   HeapNode<T>* node2);// 获取两个节点之间的节点集合
  // static double distance(HeapNode<T>* from, HeapNode<T>* to);


  // std::vector<HeapNode<T>*> cities_nodes_;
  std::vector<HeapNode<T>*> guard_nodes_;// 保卫节点向量
  std::vector<HeapNode<T>*> connection_nodes_;// 连接器节点向量
  std::vector<std::vector<HeapNode<T>*>> components;// 组件向量
  static std::shared_ptr<BaseMap> map_;// 地图指针
  static double collision_distance_check_;// 碰撞距离检查
  static double min_clearance_;// 最小间隙
  static int num_samples;// 采样点数目
  double ellipse_ratio_major_axis_focal_length_;// 椭圆长轴焦距比
  static double cutoof_distance_ratio_to_shortest_; // 切断距离与最短路径的比率
  HeapNode<T>* start_; // 起点指针
  HeapNode<T>* end_;
  bool planar_;// 是否平面规划

  Vector<3> min_position_, max_position_, position_range_;// 最小位置、最大位置和位置范围
};

template <class T> std::shared_ptr<BaseMap> pdr<T>::map_;
template <class T>
double pdr<T>::collision_distance_check_;
template <class T> int pdr<T>::num_samples;
template <class T> double pdr<T>::min_clearance_;
template <class T> double pdr<T>::cutoof_distance_ratio_to_shortest_;



// 将地图对象和起点、终点坐标保存到成员变量中。
// 从配置信息中加载路径规划器的相关参数，包括是否为平面规划、最小间隙、截断距离比例、碰撞距离检查、椭圆形状参数等。
// 检查碰撞距离检查参数是否设置，如果没有设置，则输出错误信息并退出程序。
// 将起点和终点添加到守卫节点列表中。
// 创建起点和终点所在的组件，并将它们添加到组件列表中。
template<class T>
pdr<T>::pdr(const YAML::Node& planner_config, //路径规划器的配置信息，以YAML格式传入。
                                  std::shared_ptr<BaseMap> map, T start,
                                  T end) {
  // 将地图对象和起点、终点坐标保存到成员变量中
  map_ = map;
  start_ = new HeapNode<T>(start);
  end_ = new HeapNode<T>(end);
  // 从配置信息中加载路径规划器的相关参数
  planar_ = planner_config["planar"].as<bool>();
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");
  cutoof_distance_ratio_to_shortest_ = 
    loadParam<double>(planner_config, "cutoof_distance_ratio_to_shortest");
  collision_distance_check_ =
    loadParam<double>(planner_config, "collision_distance_check");
  ellipse_ratio_major_axis_focal_length_ = loadParam<double>(
    planner_config, "ellipse_ratio_major_axis_focal_length");
  num_samples = loadParam<int>(
    planner_config, "num_samples_between_gate");
  // 检查碰撞距离检查参数是否设置
  if (collision_distance_check_ == 0) {
    ERROR(
      "you need to specify collision_distance_check for sampling-based motion "
      "planning");
    exit(1);
  }
  // 将起点和终点添加到守卫节点列表中
  guard_nodes_.push_back(start_);
  guard_nodes_.push_back(end_);

  // 创建起点和终点所在的组件，并将它们添加到组件列表中
  std::vector<HeapNode<T>*> tmp1;
  tmp1.push_back(start_);
  components.push_back(tmp1);
  std::vector<HeapNode<T>*> tmp2;
  tmp2.push_back(end_);
  components.push_back(tmp2);
}



// 该函数用于在地图上采样点，并将其添加到路径规划器的守卫节点列表中。
// 如果新采样的点可以与某个组件中的节点相互可见，将它们连接起来，形成一条边，并两个组件合并。
// 如果新采样的点与任何一个组件的节点都不可见，则将其作为一个新的组件添加到组件列表中。
template<class T>
void pdr<T>::sample_point() {
    // random point
    // // INFO("sample point")
    // // INFO("whaaaaat")
    HeapNode<T>* new_node = new HeapNode<T>();// 随机采样一个点
    // // INFO("new node created")

    // // INFO("new_node " << new_node)
    // // INFO("start_ " << start_)
    // // INFO("end_ " << end_)
    // fillRandomState(new_node);
  do {// 在地图上随机生成一个点，并将其存储在 HeapNode 对象 new_node 的 data 成员中
    map_->fillRandomStateInEllipse(new_node, start_, end_,
                                    ellipse_ratio_major_axis_focal_length_, planar_);
  } while (map_->isInCollision(new_node->data, min_clearance_));

    bool found = false;
    int component1 = -1;
    bool node1 = false;
    HeapNode<T>* g_vis;
    // 循环遍历所有已存在的组件，查找当前采样点是否与某个组件中的节点相互可见
    for (int i=0;i<components.size();i++) {
        std::vector<HeapNode<T>*> visible_cities = getVisibleinComp(new_node, i);
        // 如果当前采样点与某个组件中的节点相互可见，则将它们连接起来，形成一条边，并将这两个组件合并
        if (visible_cities.size() > 0) {
            if (!node1) {
                node1 = true;
                component1 = i;
                g_vis = visible_cities[0];
            }
            else {
                const double dist1 = distance(visible_cities[0]->data, new_node->data);
                new_node->visibility_node_ids[visible_cities[0]] = dist1;
                visible_cities[0]->visibility_node_ids[new_node] = dist1;
                const double dist2 = distance(new_node->data, g_vis->data);
                new_node->visibility_node_ids[g_vis] = dist2;
                g_vis->visibility_node_ids[new_node] = dist2;
                connection_nodes_.push_back(new_node);
                components[component1].insert(components[component1].end(), components[i].begin(), components[i].end());
                components[i].clear();
                found = true;
            }
        }
        if (found) {
            break;
        }
    }
    // 如果当前采样点与任何一个组件的节点都不可见，则将其作为一个新的组件添加到组件列表中
    if (!node1) {
        std::vector<HeapNode<T>*> visible_cities = getVisibleGuards(new_node);
        if (visible_cities.size() == 0) {
          guard_nodes_.push_back(new_node);
          std::vector<HeapNode<T>*> tmp;
          tmp.push_back(new_node);
          components.push_back(tmp);
        }
    }
}


// 扭曲路径，检查路径中的节点是否可以被替换为新的节点以减小路径长度
template<class T>
void pdr<T>::deform_path() {
    // random point
    // // INFO("sample point")
    // // INFO("whaaaaat")
    HeapNode<T>* new_node = new HeapNode<T>();
    // // INFO("new node created")
    // // INFO("new_node " << new_node)
    // // INFO("start_ " << start_)
    // // INFO("end_ " << end_)
    // fillRandomState(new_node);

    map_->fillRandomStateInEllipse(new_node, start_, end_, ellipse_ratio_major_axis_focal_length_, planar_);
   //fillRandomStateInEllipse函数在base_map函数中定义了
    TestVisibSubRoadmap(new_node);
}



// 检查当前节点与所有已知的保护点之间是否存在遮挡。
// 如果当前节点与某个可见的保护点之间没有任何遮挡，则将它们连接起来，
// 并将连接的边添加到 connection_nodes_ 列表中。
template<class T>
void pdr<T>::TestVisibSubRoadmap(
  HeapNode<T>* node) {
  std::vector<HeapNode<T>*> visible_cities = getVisibleGuards(node);// 获取当前节点可见的所有保护点
  for (int i=0;i<visible_cities.size();i++) {// 遍历所有可见的保护点对，检查它们之间是否存在遮挡
    for (int j=i+1;j<visible_cities.size();j++) {

      bool distinct = true;
      // 获取两个保护点之间的中间节点以及它们与中间节点之间的距离
      std::map<HeapNode<T>*, double> between_nodes =
        nodesBetweenNodes(visible_cities[i], visible_cities[j]);

      // 对于每个中间节点，检查它们是否会遮挡 visible_cities[i] 和 visible_cities[j] 之间的路径
      for (const auto& nb : between_nodes) {
        // check if one path visible_cities[0] new_node visible_cities[1] is
        // deformable to other visible_cities[0] nb.first visible_cities[1]
        //   bool is_deformable = isDeformablePathBetween(
        //   visible_cities[0], visible_cities[1], new_node, nb.first);
        // 检查 visible_cities[i] 和 visible_cities[j] 之间是否有一条可变形的路径通过当前中间节点 nb.first
        bool is_deformable = map_->isDeformablePathBetween(
          
            visible_cities[i], visible_cities[j], node, nb.first,
            min_clearance_, collision_distance_check_);
        if (is_deformable) {
          distinct = false;
          break;
        }
      }
      // 如果当前可见点对之间没有任何遮挡，则将它们连接起来，并将连接的边添加到 connection_nodes_ 列表中
      // if (distinct /*&& (between_nodes.size() > 0 || visible_cities.size() == 2)*/) {
      if (distinct && (between_nodes.size() > 0 || visible_cities.size() == 2)) {
        const double dist1 = distance(visible_cities[i]->data, node->data);
        node->visibility_node_ids[visible_cities[i]] = dist1;
        visible_cities[i]->visibility_node_ids[node] = dist1;
        const double dist2 = distance(node->data, visible_cities[j]->data);
        node->visibility_node_ids[visible_cities[j]] = dist2;
        visible_cities[j]->visibility_node_ids[node] = dist2;
        connection_nodes_.push_back(node);
        // INFO("new path " << node->data.transpose())
        return;
      }
    }
 }
}



// 函数遍历所有的guard_nodes_，并判断每个保护点与当前节点之间是否存在简单路径，
// 并且路径上没有障碍物干扰。
// 如果存在这样的路径，则将当前保护点添加到可见保护点列表(visible_cities)中。
// 最后，函数返回可见保护点列表。
template<class T>
std::vector<HeapNode<T>*> pdr<T>::getVisibleGuards(
  HeapNode<T>* node) {
  std::vector<HeapNode<T>*> visible_cities;
  // for (size_t i = 0; i < cities_nodes_.size(); i++) {
  //   if (isSimplePathFreeBetweenNodes(cities_nodes_[i], node)) {
  //     visible_cities.push_back(cities_nodes_[i]);
  //   }
  // }
  // 遍历所有的保护点，检查它们与当前节点之间是否存在简单路径，即不被障碍物遮挡的路径
  for (size_t i = 0; i < guard_nodes_.size(); i++) {
    // 判断从当前保护点到当前节点之间是否存在简单路径，且路径上没有障碍物干扰
    if (map_->isSimplePathFreeBetweenGuards(guard_nodes_[i]->data, node->data, 
                            min_clearance_, collision_distance_check_)) {
      visible_cities.push_back(guard_nodes_[i]);// 如果存在简单路径，则将当前保护点添加到可见保护点列表中
    }
  }
  return visible_cities;
}



// 获取当前节点可见的保护点和连接点列表。
// 函数首先遍历所有的guard_nodes_，并判断每个保护点与当前节点之间是否存在简单路径，
// 并且路径上没有障碍物干扰。
// 如果存在这样的路径，则将当前保护点添加到可见节点列表（visible_cities）中。
// 接着，函数遍历所有的连接点（connection_nodes_），并进行同样的判断和操作。
// 最后，函数返回可见节点列表。
template<class T>
std::vector<HeapNode<T>*> pdr<T>::getVisibleNodes(
  HeapNode<T>* node) {
  std::vector<HeapNode<T>*> visible_cities;
  // for (size_t i = 0; i < cities_nodes_.size(); i++) {
  //   if (isSimplePathFreeBetweenNodes(cities_nodes_[i], node)) {
  //     visible_cities.push_back(cities_nodes_[i]);
  //   }
  // }
  // 遍历所有的保护点，检查它们与当前节点之间是否存在简单路径，即不被障碍物遮挡的路径
  for (size_t i = 0; i < guard_nodes_.size(); i++) {
    // 判断从当前保护点到当前节点之间是否存在简单路径，且路径上没有障碍物干扰
    if (map_->isSimplePathFreeBetweenGuards(guard_nodes_[i]->data, node->data, 
                            min_clearance_, collision_distance_check_)) {
      visible_cities.push_back(guard_nodes_[i]);// 如果存在简单路径，则将当前保护点添加到可见保护点列表中
    }
  }

  // 遍历所有的连接点，检查它们与当前节点之间是否存在简单路径，即不被障碍物遮挡的路径
  for (size_t i = 0; i < connection_nodes_.size(); i++) {
    // 判断从当前连接点到当前节点之间是否存在简单路径，且路径上没有障碍物干扰
    if (map_->isSimplePathFreeBetweenGuards(connection_nodes_[i]->data, node->data, 
                            min_clearance_, collision_distance_check_)) {
      // 如果存在简单路径，则将当前连接点添加到可见节点列表中
      visible_cities.push_back(connection_nodes_[i]);
    }
  }
  return visible_cities;
}



// 函数用于获取给定组件中与给定节点可见的节点列表。
// 函数遍历指定组件中的所有节点，判断每个节点与给定节点之间是否存在简单路径，并且路径上没有障碍物干扰。
// 如果存在这样的路径，则将当前节点添加到可见节点列表中。
template<class T>
std::vector<HeapNode<T>*> pdr<T>::getVisibleinComp(
  HeapNode<T>* node, int comp) {
  std::vector<HeapNode<T>*> visible_cities;
  // for (size_t i = 0; i < cities_nodes_.size(); i++) {
  //   if (isSimplePathFreeBetweenNodes(cities_nodes_[i], node)) {
  //     visible_cities.push_back(cities_nodes_[i]);
  //   }
  // }
  // 遍历指定组件中的所有节点
  for (size_t i = 0; i < components[comp].size(); i++) {
    if (map_->isSimplePathFreeBetweenGuards(components[comp][i]->data, node->data, 
                            min_clearance_, collision_distance_check_)) {// 判断当前组件中的节点与给定节点之间是否存在简单路径，且路径上没有障碍物干扰
      visible_cities.push_back(components[comp][i]);// 如果存在简单路径，则将当前节点添加到可见节点列表中
    }
  }
  return visible_cities;
}


//函数用于获取两个给定节点之间的可见节点集合及距离。
// 函数遍历节点1的可见节点集合，查找其中是否存在节点2，
// 如果存在，则将节点1及其到节点2的距离添加到 between_nodes 中。
template<class T>
std::map<HeapNode<T>*, double> pdr<T>::nodesBetweenNodes(
  HeapNode<T>* node1, HeapNode<T>* node2) {

  std::map<HeapNode<T>*, double> between_nodes;
  typename std::unordered_map<HeapNode<T>*, double>::iterator it;
  // std::vector<HeapNode<T>*> between_nodes;
  for (const auto& vn1 : node1->visibility_node_ids) {// 遍历节点1的可见节点集合
    it = vn1.first->visibility_node_ids.find(node2);// 查找节点1可见节点集合中是否存在节点2
    if (it != vn1.first->visibility_node_ids.end()) {
      between_nodes.insert(// 如果节点1可见节点集合中存在节点2，则将节点1及其到节点2的距离添加到between_nodes中
        std::pair<HeapNode<T>*, double>(vn1.first, vn1.second + it->second));
    }
  }
  return between_nodes;
}




// 函数用于设置连接点。
// 函数遍历所有保护点的组合，判断每一对保护点之间是否存在可见节点，
// 如果存在，则将该可见节点添加到连接点列表中。
// 最后，将新的连接点列表赋值给 connection_nodes_。
template<class T>
void pdr<T>::setConnectors() {

  std::vector<HeapNode<T>*> new_connection_nodes_;
  // 遍历所有保护点
  for (int i=0;i<guard_nodes_.size();i++) {
    // INFO("guard " << guard_nodes_[i]->data.transpose())
    for (int j=i+1;j<guard_nodes_.size();j++) {// 遍历保护点i的可见节点集合
      typename std::unordered_map<HeapNode<T>*, double>::iterator it;
      for (const auto& vn1 : guard_nodes_[i]->visibility_node_ids) {// 查找保护点i的可见节点集合中是否存在保护点j
        it = vn1.first->visibility_node_ids.find(guard_nodes_[j]);
        if (it != vn1.first->visibility_node_ids.end()) {// 如果保护点i的可见节点集合中存在保护点j，则将该可见节点添加到连接点列表中
          new_connection_nodes_.push_back(vn1.first);
        }
      }
    }
  }

  connection_nodes_ = new_connection_nodes_;
}


// setBorders 函数用于设置地图的边界。
// 将最小位置和最大位置保存下来，并计算出地图的范围。
template<class T>
void pdr<T>::setBorders(Vector<3> min_position,
                                   Vector<3> max_position) {
  // 设置地图边界
  min_position_ = min_position;
  max_position_ = max_position;
  position_range_ = max_position - min_position;
}

// saveRoadmap 函数用于将路图保存到文件中。
// 函数依次遍历每个保护点和连接点，将节点及其可见节点写入文件。
template<class T>
void pdr<T>::saveRoadmap(std::string filename) {
  // INFO("save pdr map to file " << filename);
  // 将路网保存到文件中
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    // 依次遍历每个保护点
    for (size_t i = 0; i < guard_nodes_.size(); i++) {
      std::string city_node_str = to_string_raw(guard_nodes_[i]->data);
      myfile << city_node_str << std::endl;
      // 遍历当前保护点的可见节点集合
      for (const auto& vn1 : guard_nodes_[i]->visibility_node_ids) {
        std::string neighbor_str = to_string_raw(vn1.first->data);
        ss_connections << city_node_str << "," << neighbor_str << std::endl;
      }
    }
    // 依次遍历每个连接点
    for (size_t i = 0; i < connection_nodes_.size(); i++) {
      std::string city_node_str = to_string_raw(connection_nodes_[i]->data);
      myfile << city_node_str << std::endl;
      for (const auto& vn1 : connection_nodes_[i]->visibility_node_ids) {// 遍历当前连接点的可见节点集合
        std::string neighbor_str = to_string_raw(vn1.first->data);
        ss_connections << city_node_str << "," << neighbor_str << std::endl;
      }
    }
    // 将连接关系写入文件
    myfile << ss_connections.str();
    myfile.close();
  }
}



// savePath 函数用于将路径保存到文件中。
// 函数依次遍历路径上的节点，将节点之间的连线写入文件。
template<class T>
void pdr<T>::savePath(std::string filename,
                                 std::vector<HeapNode<T>*> path) {
  // INFO("save pdr map to file " << filename);
  std::ofstream myfile;// 将路径保存到文件中
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    // 依次遍历路径上的节点
    for (size_t ni = 1; ni < path.size(); ni++) {
      std::string from_str = to_string_raw(path[ni - 1]->data);
      std::string to_str = to_string_raw(path[ni]->data);
      myfile << from_str << "," << to_str << std::endl;
    }
    myfile.close();
  }
}


// findDistinctPaths 函数用于查找不重复的路径。
// 函数首先构造一个包含所有保护点和连接点的节点列表，
// 然后通过调用 DistinctPathDFS 类的 findPaths 函数找到不重复的路径。
template<class T>
std::vector<path_with_length<T>> pdr<T>::findDistinctPaths() {
// 查找不重复的路径
  std::vector<HeapNode<T>*> all_nodes;
  for (int i=0;i<guard_nodes_.size();i++) {
    if (guard_nodes_[i]->data != start_->data && guard_nodes_[i]->data != end_->data)
      all_nodes.push_back(guard_nodes_[i]);
  }
  all_nodes.insert(all_nodes.end(), connection_nodes_.begin(),
                   connection_nodes_.end());   // do not add start and end，即不添加起点和终点
  DistinctPathDFS<T> dpDFS(all_nodes, start_, end_);
  std::vector<path_with_length<T>> distinct_paths = dpDFS.findPaths();
  // INFO("findDistinctPaths end")
  return distinct_paths;
}



// testVisibilityGraph 函数用于测试可见图的生成。
// 函数首先构造一个包含所有保护点和连接点的节点列表，
// 然后通过调用 Dijkstra 类的 findPath 函数找到最短路径。
// 如果最短路径为空，则返回空的路径列表。
template<class T>
std::vector<path_with_length<T>> pdr<T>::testVisibilityGraph() {
// 测试可见图的生成
  std::vector<HeapNode<T>*> all_nodes = guard_nodes_;
  all_nodes.insert(all_nodes.end(), connection_nodes_.begin(),
                   connection_nodes_.end());  // do not add start and end
//   DistinctPathDFS<T> dpDFS(all_nodes, start_, end_);
//   std::vector<path_with_length<T>> distinct_paths = dpDFS.findPaths();


  Dijkstra<T> dijkstra;
  std::vector<path_with_length<T>> shortest_plan =
      dijkstra.findPath(0, {1}, all_nodes);
  // INFO("findDistinctPaths end")
  if (shortest_plan[0].plan.size() == 0) {
    std::vector<path_with_length<T>> empty_plan;
    return empty_plan;
  }
  return shortest_plan;
}


// 实现步骤
// 1、创建一个新的空路径shortened，并将其长度初始化为0。
// 2、根据路径的长度和碰撞检查距离计算采样点的数量。
// 3、使用采样点对原始路径进行插值，得到一系列采样点的坐标。
// 4、根据参数forward的取值，确定路径的起点，并将其添加到缩短后的路径中。
// 5、遍历采样点，对每个采样点进行碰撞检查。
// 6、如果当前采样点与前一个点之间存在碰撞，则根据碰撞点计算新的路径点，并将其添加到缩短后的路径中。
// 7、将缩短后的路径的最后一个节点与原始路径的最后一个节点连接起来，形成完整的路径。
// 8、如果forward为false，将缩短后的路径进行反转。
// 9、返回缩短后的路径。
template <class T>
path_with_length<T>
pdr<T>::shorten_path(path_with_length<T> path,
                                          bool forward) {
  path_with_length<T> shortened;// 创建一个空的缩短后的路径结构体
  shortened.length = 0;// 初始化缩短后路径的长度为0

  // return path;
  // 计算采样点数量，根据路径长度和碰撞检查距离计算，并向上取整
  const double num_samples = ceil(path.length / collision_distance_check_) + 1;
  // INFO("path.path size " << path.path.size())
  // INFO("num_samples " << num_samples);
  // INFO("collision_distance_check_ " << collision_distance_check_);
  // 使用采样点对路径进行插值，得到一系列采样点的坐标
  std::vector<T> sampled = map_->samplePath(path.plan, path.length, num_samples);
  // INFO("num samples " << sampled.size());

  // int dir = 1;
  int start_iter = 1;
  if (forward) {
    shortened.plan.push_back(path.plan.front());// 如果按正向顺序缩短路径，则将路径起点添加到缩短后的路径中
  } else {
    shortened.plan.push_back(path.plan.back());// 否则，将路径终点添加到缩短后的路径中，并反转采样点的顺序
    std::reverse(sampled.begin(), sampled.end());
    // dir = -1;
    // start_iter = sampled.size() - 2;
  }


  // for (size_t i = sampled.size()-2; i < sampled.size(); i += dir) {
  for (size_t i = 1; i < sampled.size(); i++) {
    HeapNode<T>* start_node = shortened.plan.back();// 获取当前缩短后路径的最后一个节点作为起点
    // INFO("from " << start_node->data.transpose() << " to "
    //              << sampled[i].transpose())


    std::pair<bool, Vector<3>> is_free =
      map_->isSimplePathFreeBetweenNodes(start_node->data, sampled[i], min_clearance_, collision_distance_check_);
      // 检查起点和当前采样点之间是否存在碰撞，并返回是否自由以及碰撞点的坐标
    if (!is_free.first) {
      // INFO("not free")
      const Vector<3> collision_place = is_free.second;// 获取碰撞点的坐标
      const auto [gradient_in_place, voxel_center] =
        map_->gradientInVoxelCenter(collision_place);// 获取碰撞点处的梯度向量和对应体素的中心坐标
      // INFO("collision in pos " << collision_place.transpose())
      // INFO("gradient_in_place " << gradient_in_place.transpose())
      // go perpendicular to line shortened.path.back()->data, sampled[i]
      // and towards shortest distance to line shortened.path.back()->data,
      // sampled[i-1]
      // const Vector<3> old_point_vector = sampled[i - 1] - start_node->data;


      const Vector<3> new_point_vector = sampled[i] - start_node->data;// 计算起点与当前采样点之间的向量
      // INFO_VAR(new_point_vector.transpose())
      const Vector<3> normal_gradient_new_point =
        gradient_in_place.cross(new_point_vector);// 计算起点、采样点和碰撞点处梯度向量构成的法向量
      // INFO_VAR(normal_gradient_new_point.transpose())

      // const Vector<3> normal_new_old_start =
      //   old_point_vector.cross(new_point_vector);
      Vector<3> vec_from_collision =
        new_point_vector.cross(normal_gradient_new_point);
      vec_from_collision.normalize();// 计算与碰撞点法向量垂直的向量并归一化
      // INFO_VAR(vec_from_collision.transpose())

      // const double clearance_collision =
      // map_->getClearence(collision_place); INFO("clearance_collision " <<
      // clearance_collision)
      const double ds = collision_distance_check_;// 碰撞检查步长

      // double count_check =
      // std::min(collision_distance_check_ / clearance_collision, 1.0);
      // INFO("count_check " << count_check)
      bool added_after_collision = false;
      for (double tci = min_clearance_; tci <= min_clearance_ * 4; tci += ds) {
        const Vector<3> new_point = voxel_center + vec_from_collision * tci;// 在沿着碰撞点法向量方向上的不同距离处生成新的路径点进行碰撞检查
        // INFO("test collision in new place " << new_point.transpose())
        if (!map_->isInCollision(new_point, min_clearance_)) {
          HeapNode<T>* new_node = new HeapNode<T>(new_point);
          // HeapNode<T>* back_old = shortened.path.back();
          const double dist_new_node = distance(start_node->data, new_point);
          // 计算新节点与起点之间的距离
          start_node->visibility_node_ids[new_node] = dist_new_node;
          new_node->visibility_node_ids[start_node] = dist_new_node;
          shortened.length += dist_new_node;// 更新缩短后路径的长度
          shortened.plan.push_back(new_node);// 将新节点添加到缩短后的路径中
          added_after_collision = true;
          break;
        } else {
          // INFO("in collision wiht value" << map_->getClearence(new_point))
        }
      }

      if (!added_after_collision) {
        ERROR_RED("no point added to shortened path after collision");
        return path;// 如果无法在碰撞后添加节点，则返回原始路径
        exit(1);
      }
    }
  }


  // INFO("shortened from " << path.length << " to " << shortened.length)
  // INFO("shortened.path.size() " << shortened.path.size())
  // INFO("shorten_path end")

  HeapNode<T>* last_node_original;
  if (forward) {
    last_node_original = path.plan.back();// 获取原始路径的最后一个节点
  } else {
    last_node_original = path.plan.front();// 获取原始路径的第一个节点
  }

  HeapNode<T>* back_old = shortened.plan.back();
  const double dist_new_node =
    distance(back_old->data, last_node_original->data);// 计算最后一个新节点与原始路径最后一个节点之间的距离
  back_old->visibility_node_ids[last_node_original] = dist_new_node;
  last_node_original->visibility_node_ids[back_old] = dist_new_node;
  shortened.length += dist_new_node;// 更新缩短后路径的长度
  shortened.plan.push_back(last_node_original);// 将原始路径最后一个节点添加到缩短后的路径中

  if (!forward) {
    reverse(shortened.plan.begin(), shortened.plan.end());// 如果按反向顺序缩短路径，则将路径进行反转
  }


  // debuging begin
  // check distance
  double calc_distance = 0;
  for (size_t i = 1; i < shortened.plan.size(); i++) {
    calc_distance +=
      distance(shortened.plan[i - 1]->data, shortened.plan[i]->data);
  }
  if (fabs(calc_distance - shortened.length) > PRECISION || shortened.plan[shortened.plan.size() - 1]->data != path.plan[path.plan.size() - 1]->data) {
    INFO_RED("shortened length does not equal")
    INFO_VAR(shortened.length)
    INFO_VAR(calc_distance)
    exit(1);
  }
  // INFO("calc_distance " << calc_distance)
  // debuging end


  return shortened;// 返回缩短后的路径
}


// 对输入的路径集合进行遍历和比较，
// 移除其中等效的路径，并返回移除等效路径后的路径集合。
// 在比较路径时，其考虑路径的长度以及是否可以通过扭曲变换转化为另一条路径，
// 然后选择最短的路径保留下来。
template<class T>
std::vector<path_with_length<T>> pdr<T>::removeEquivalentPaths(
  std::vector<path_with_length<T>> paths) {
  // // INFO_GREEN("removeEquivalentPaths begin with " << paths.size() << " paths")
  // // INFO_VAR(min_clearance_)
  // // INFO_VAR(collision_distance_check_)
  // // INFO_VAR(map_->getResolution())
  std::vector<path_with_length<T>> paths_copy = paths;// 复制路径集合
  // std::vector<path_with_length<T>> proned;
  if (paths_copy.size() > 1) {// 对每条路径进行比较
    // bool something_removed = true;
    // while (something_removed) {
    // something_removed = false;

    // for (size_t i = 0; i < paths_copy.size(); i++) {
    size_t i = 0;
    while(i < paths_copy.size()) {
      int shortest_path_i = i;// 记录最短路径的索引
      double shortest_length = paths_copy[i].length;// 记录最短路径的长度
      std::vector<int> to_remove_indexes;// 待移除路径的索引集合
      for (size_t j = i + 1; j < paths_copy.size(); j++) {
        // 计算用于检查碰撞的数量
        const double larger_length =
          std::max(paths_copy[i].length, paths_copy[j].length);
        const double num_check_collision =
          ceil(larger_length / collision_distance_check_) + 1;
        bool deformable = map_->isDeformablePath(
          // 判断是否可扭曲为另一条路径
          paths_copy[i].plan, paths_copy[i].length, paths_copy[j].plan,
          paths_copy[j].length, num_check_collision, min_clearance_, collision_distance_check_);
        if (deformable) {
          // 如果可扭曲，则记录待移除路径的索引，并更新最短路径信息
          // // INFO("path " << i << " is deformable to " << j)
          // ith_unique = false;
          to_remove_indexes.push_back(j);
          if (paths_copy[j].length < shortest_length) {
            shortest_path_i = j;
            shortest_length = paths_copy[j].length;
          }
        } else {
          // // INFO("path " << i << " not deformable to " << j)
        }
      }
      // 更新路径集合，移除等效路径
      if (shortest_path_i != i) {
        paths_copy[i] = paths_copy[shortest_path_i];
      }
      for (int tri = to_remove_indexes.size() - 1; tri >= 0; tri--) {
        // INFO("removing " << to_remove_indexes[tri])
        paths_copy.erase(paths_copy.begin() + to_remove_indexes[tri]);
        // // INFO("size " << to_remove_indexes[tri])
      }
      // // INFO("purged")
      //}
      i++;
    }
  }
  // // INFO_GREEN("removeEquivalentPaths end")
  return paths_copy;// 返回移除等效路径后的路径集合
}


// removeTooLongPaths函数用于剪枝一组路径，
// 使得剩余的路径长度都不超过最短路径长度的一定比例。
// 其中，T 是路径中点的数据类型，paths 是待剪枝的路径 vector。
template<class T>
std::vector<path_with_length<T>> pdr<T>::removeTooLongPaths(
  std::vector<path_with_length<T>> paths) {
  std::vector<path_with_length<T>> proned = paths;//复制 paths 到 proned 中。
  // // INFO("removeTooLongPaths begin")

  // find shortest length
  //遍历 proned 中的所有路径，记录最短路径长度。
  double min_length = DBL_MAX;
  for (auto p : proned) {
    if (p.length < min_length) {
      min_length = p.length;
    }
  }

  // remove the one that are longer than cutoof_distance_ratio_to_shortest_ *
  // min_length
  //从 proned 中删除长度大于最短路径长度的一定比例的路径。
  for (int i = proned.size() - 1; i >= 0; i--) {
    if (proned[i].length > cutoof_distance_ratio_to_shortest_ * min_length) {
      proned.erase(proned.begin() + i);
    }
  }
  // // INFO("remove " << (paths.size() - proned.size())
  //                << " paths due to being too long")
  // // INFO("removeTooLongPaths end")
  return proned;//返回剪枝后的路径 vector。
}



// find_geometrical_paths函数，用于寻找两个点之间的几何路径。
// 函数的输入参数包括规划器配置信息planner_config、地图信息map、起点、
// 终点位置列表gates_with_start_end_poses，
// 以及输出文件夹路径output_folder。
// 函数的返回值是一个二维向量，表示每对起点和终点之间找到的路径。
template <class T>
std::vector<std::vector<path_with_length<Vector<3>>>>
pdr<T>::find_geometrical_paths(// 寻找两个点之间的几何路径
    const YAML::Node &planner_config, std::shared_ptr<BaseMap> map,
    std::vector<Vector<3>> &gates_with_start_end_poses,
    std::string output_folder) {

  // INFO("find_geometrical_paths begin")
  // 创建一个空的二维向量，用于存储每对起点和终点之间的路径
  auto begin_c = std::chrono::high_resolution_clock::now();
  // std::shared_ptr<PRM<Vector<3>>> prm;
  std::vector<std::vector<path_with_length<Vector<3>>>> paths_between_gates;
  // 创建一个'pdr'对象的向量，数量为起点和终点的数量减一
  std::vector<std::shared_ptr<pdr<Vector<3>>>> topological_prms;

  paths_between_gates.resize(gates_with_start_end_poses.size() - 1);
  topological_prms.resize(gates_with_start_end_poses.size() - 1);
  for (size_t i = 0; i < topological_prms.size(); i++) {
    // // INFO("gates_with_start_end_poses[i] " << gates_with_start_end_poses[i])
    // // INFO("gates_with_start_end_poses[i+1] "
    //      << gates_with_start_end_poses[i + 1])
    // 创建一个'pdr'对象并存储在 `topological_prms` 向量中
    topological_prms[i] = std::make_shared<pdr<Vector<3>>>(
      planner_config, map, gates_with_start_end_poses[i],
      gates_with_start_end_poses[i + 1]);
    // 设置地图边界
    topological_prms[i]->setBorders(map->getMinPos(), map->getMaxPos());
  }
  // INFO("TopologicalPRM created");

  for (size_t i = 0; i < topological_prms.size(); i++) {
    // 初始化采样点数量为'num_samples'
    int samples_left = num_samples;
    std::vector<path_with_length<Vector<3>>> paths;
    bool path_found = false;
    // 进行多次采样，每次采样一部分点，直到找到路径或者采样点数用尽
    for (size_t si = 0; si < num_samples/2; si++) {
        topological_prms[i]->sample_point();
      }
      samples_left -= num_samples / 2;
      // topological_prms[i]->guard_nodes_ = topological_prms[i]->components[0];
      // std::stringstream ss;
      // ss << "roadmap_all" << i << ".csv";    
      // topological_prms[i]->saveRoadmap(ss.str());

      paths = topological_prms[i]->testVisibilityGraph();
      if (paths.size() > 0) {
        path_found = true;
      }

    while (!path_found && samples_left > 0) {
      for (size_t si = 0; si < num_samples/8; si++) {
        topological_prms[i]->sample_point();
      }
      samples_left -= num_samples / 8;
      // topological_prms[i]->guard_nodes_ = topological_prms[i]->components[0];
      // std::stringstream ss;
      // ss << "roadmap_all" << i << ".csv";    
      // topological_prms[i]->saveRoadmap(ss.str());

      paths = topological_prms[i]->testVisibilityGraph();
      if (paths.size() > 0) {
        path_found = true;
      }
    }

    if (!path_found) {
      INFO("no path found")// 如果未找到路径，则返回空的路径向量
      return paths_between_gates;
    }

    std::vector<path_with_length<Vector<3>>> diff_paths;

    if (samples_left <= 0) {
      diff_paths = paths;
    }
    else { 
      // 对找到的路径进行优化   
      INFO("path found searching with " << samples_left << " samples")
      topological_prms[i]->guard_nodes_ = topological_prms[i]->components[0];
      topological_prms[i]->setConnectors();
      for (size_t si = 0; si < samples_left; si++) {
        topological_prms[i]->deform_path();
      }

      diff_paths = topological_prms[i]->findDistinctPaths();
    }

    INFO("number of paths found " << diff_paths.size())
    
    auto end_c = std::chrono::high_resolution_clock::now();
    auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);
    INFO_GREEN("algorithm time pdr " << elapsed_c.count() * 1e-9)

    // 对优化后的路径进行缩短和去除重复路径
    std::vector<path_with_length<Vector<3>>> shortened_paths = diff_paths;

    for (size_t pi = 0; pi < diff_paths.size(); pi++) {
      // // INFO("shortening path bef " << pi)
      // // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())
      shortened_paths[pi] =
        pdr<Vector<3>>::shorten_path(shortened_paths[pi], false);
      shortened_paths[pi] =
        pdr<Vector<3>>::shorten_path(shortened_paths[pi]);
      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // // INFO("shortening path aft " << pi)
    }

    shortened_paths =
     pdr<Vector<3>>::removeTooLongPaths(shortened_paths);

    shortened_paths =
     pdr<Vector<3>>::removeEquivalentPaths(shortened_paths);

    for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
      // // INFO("shortening path bef " << pi)
      // // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())
      shortened_paths[pi] =
        pdr<Vector<3>>::shorten_path(shortened_paths[pi], false);
      shortened_paths[pi] =
        pdr<Vector<3>>::shorten_path(shortened_paths[pi]);
      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // // INFO("shortening path aft " << pi)
    }
    shortened_paths =
     pdr<Vector<3>>::removeEquivalentPaths(shortened_paths);

    std::sort(shortened_paths.begin(), shortened_paths.end(),
              comparator_path_with_length<Vector<3>>);

    paths_between_gates[i] = shortened_paths;
    // auto end_c = std::chrono::high_resolution_clock::now();
    // auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);
    // INFO_GREEN("post processing time pdr " << elapsed_c.count() * 1e-9)

    for (size_t pi = 0; pi < diff_paths.size(); pi++) {
    //   std::stringstream path_ss;
    //   // INFO(diff_paths[pi].length)
    //   path_ss << "roadmap_path" << i << "_" << pi << ".csv";
    //   pdr<Vector<3>>::savePath(path_ss.str(), diff_paths[pi].plan);
    }
  }
  // INFO("samples created");

  return paths_between_gates;// 返回每对起点和终点之间找到的路径
}