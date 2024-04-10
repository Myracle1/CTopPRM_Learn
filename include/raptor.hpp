/*
该代码实现的是论文中V.RESULT部分用于比较的算法2：RAPTOR
论文是[2]B. Zhou, J. Pan, F. Gao, and S. Shen, “RAPTOR: Robust and perception aware trajectory replanning for quadrotor fast flight,” IEEE Trans. Robot.,vol. 37, no. 6, pp. 1992–2009, Dec. 2021.
实现港科大提出的RAPTOR算法：
其结合稳健的 optimistic 重规划和 perception-aware 规划策略，
提高无人机在未知和危险空间中的安全性和可见性。
在优化轨迹的基础上，偏航角得到规划，以主动探索未知环境的内容。 
✨代码分析
👉函数名和pdr.hpp中的函数名类似，但是实现细节不一样(算法的区别主要体现在函数find_geometrical_paths()中)，详细参考我的报告
①  sample_point()：对地图上的点进行采样，生成候选路径上的节点，并进行处理，包括添加到守卫节点、连接节点等。
②  getVisibleGuards()：获取可见的守卫节点，用于判断新节点与其他节点之间是否存在可行路径。
③  nodesBetweenNodes()：计算两个节点之间的路径，用于判断节点之间是否存在连通路径。
④  saveRoadmap()：将生成的路图保存到文件中。
⑤  findDistinctPaths()：查找不同的路径，使用DFS算法在地图上查找不同的路径。
⑥  shorten_path()：缩短路径的长度，通过采样和碰撞检测等方法对路径进行优化。
⑦  removeEquivalentPaths()：移除等效路径，移除重复或相似的路径。
⑧  removeTooLongPaths()：移除过长的路径，移除超出指定长度的路径。
⑨  find_geometrical_paths()：寻找几何路径，使用Raptor算法在给定地图上寻找起点和终点之间的几何路径。
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

#define PRECISION_PRM_CLUSTERING (10e-6)
#define PRECISION (1E-4)

template<class T>
class Raptor {
 public:
 // Raptor类的构造函数，接受planner_config配置、地图指针、起点和终点作为参数
  Raptor(const YAML::Node& planner_config, std::shared_ptr<BaseMap> map,
                 T start, T end);
  void sample_point();// 对点进行采样
  void setBorders(Vector<3> min_position, Vector<3> max_position); // 设置边界范围
  void saveRoadmap(std::string filename);// 保存路图到文件
  static void savePath(std::string filename, std::vector<HeapNode<T>*> path);// 将路径保存到文件
  static path_with_length<T> shorten_path(path_with_length<T> path,// 缩短路径的长度
                                          bool forward = true);
  std::vector<path_with_length<T>> findDistinctPaths();  // 查找不同的路径
  static std::vector<std::vector<path_with_length<Vector<3>>>>
  find_geometrical_paths(const YAML::Node &planner_config,// 查找几何路径
                         std::shared_ptr<BaseMap> map,
                         std::vector<Vector<3>> &gates_with_start_end_poses,
                         std::string output_folder);
  static std::vector<path_with_length<T>>
  removeEquivalentPaths(std::vector<path_with_length<T>> paths);// 移除等效路径
  static std::vector<path_with_length<T>> 
  removeTooLongPaths(std::vector<path_with_length<T>> paths);// 移除过长的路径

 private:

  static std::string to_string_raw(T data);  // 将数据转换为字符串
  static double distance(Vector<3> from, Vector<3> to) { return (from - to).norm(); }// 计算两个点之间的距离

  std::vector<HeapNode<T>*> getVisibleGuards(HeapNode<T>* node);// 获取可见的守卫节点
  std::map<HeapNode<T>*, double> nodesBetweenNodes(HeapNode<T>* node1,
                                                   HeapNode<T>* node2);
  // static double distance(HeapNode<T>* from, HeapNode<T>* to);


  // std::vector<HeapNode<T>*> cities_nodes_;
  std::vector<HeapNode<T>*> guard_nodes_;
  std::vector<HeapNode<T>*> connection_nodes_;
  static std::shared_ptr<BaseMap> map_;
  static double collision_distance_check_;// 碰撞距离检查阈值
  static double min_clearance_;// 最小间隙
  static int num_samples;// 采样点数量
  double ellipse_ratio_major_axis_focal_length_;// 椭圆长轴焦距比率
  static double cutoof_distance_ratio_to_shortest_;// 距离最短路径的截断距离比率
  HeapNode<T>* start_;
  HeapNode<T>* end_;
  bool planar_;

  Vector<3> min_position_, max_position_, position_range_;
};

template <class T> std::shared_ptr<BaseMap> Raptor<T>::map_;
template <class T>
double Raptor<T>::collision_distance_check_;
template <class T> int Raptor<T>::num_samples;
template <class T> double Raptor<T>::min_clearance_;
template <class T> double Raptor<T>::cutoof_distance_ratio_to_shortest_;


template<class T>
Raptor<T>::Raptor(const YAML::Node& planner_config, 
                                  std::shared_ptr<BaseMap> map, T start,
                                  T end) {
  map_ = map;// 设置地图对象
  start_ = new HeapNode<T>(start);// 创建起点HeapNode对象
  end_ = new HeapNode<T>(end);// 创建终点HeapNode对象
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");// 加载最小间隙参数
  planar_ = planner_config["planar"].as<bool>();// 加载是否平面规划参数
  cutoof_distance_ratio_to_shortest_ = 
    loadParam<double>(planner_config, "cutoof_distance_ratio_to_shortest");// 加载截断距离与最短路径的比例参数
  collision_distance_check_ =
    loadParam<double>(planner_config, "collision_distance_check");// 加载碰撞距离检查参数
  ellipse_ratio_major_axis_focal_length_ = loadParam<double>(
    planner_config, "ellipse_ratio_major_axis_focal_length");// 加载椭圆长轴焦距比率参数
  num_samples = loadParam<int>(
    planner_config, "num_samples_between_gate");// 加载采样点数量参数

  if (collision_distance_check_ == 0) {
    ERROR(
      "you need to specify collision_distance_check for sampling-based motion "
      "planning");
    exit(1);
  }

  guard_nodes_.push_back(start_);// 将起点添加到守卫节点容器中
  guard_nodes_.push_back(end_);// 将终点添加到守卫节点容器中
}

template<class T>
void Raptor<T>::sample_point() {// 随机生成一个采样点
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
// 在起点和终点之间的椭圆区域内填充随机状态

  std::vector<HeapNode<T>*> visible_cities = getVisibleGuards(new_node);// 获取与新节点可见的守卫节点，并保存到visible_cities容器中
  // // INFO("visible_cities.size() " << visible_cities.size())
  if (visible_cities.size() == 0) {// 如果visible_cities容器为空，表示new_node是一个新的守卫节点，将其添加到guard_nodes_容器中
    // // INFO("new guard")
    guard_nodes_.push_back(new_node);
  } else if (visible_cities.size() == 2) {
    bool distinct = true;

    // now there is path between visible_cities[0] new_node and
    // visible_cities[1]
    std::map<HeapNode<T>*, double> between_nodes =// 检查这两个节点之间的路径是否可变形
      nodesBetweenNodes(visible_cities[0], visible_cities[1]);

    for (const auto& nb : between_nodes) {
      // check if one path visible_cities[0] new_node visible_cities[1] is
      // deformable to other visible_cities[0] nb.first visible_cities[1]
      //   bool is_deformable = isDeformablePathBetween(
      //   visible_cities[0], visible_cities[1], new_node, nb.first);

      bool is_deformable = map_->isDeformablePathBetween(
           visible_cities[0], visible_cities[1], new_node, nb.first,
           min_clearance_, collision_distance_check_);
      if (is_deformable) {
        distinct = false;
        const double dist1 = distance(visible_cities[0]->data, new_node->data);
        const double dist2 = distance(new_node->data, visible_cities[1]->data);
        // const double distance_new = distance(visible_cities[0], new_node) +
        //                             distance(new_node, visible_cities[1]);
        if (dist1 + dist2 < nb.second) {// 更新其中一个节点的位置，使得从visible_cities[0]到new_node再到visible_cities[1]的距离小于原有路径的距离
          nb.first->data = new_node->data;
          nb.first->visibility_node_ids[visible_cities[0]] = dist1;
          visible_cities[0]->visibility_node_ids[nb.first] = dist1;
          nb.first->visibility_node_ids[visible_cities[1]] = dist2;
          visible_cities[1]->visibility_node_ids[nb.first] = dist2;
          // // INFO("improved pos " << new_node->data)
        }
        break;
      }
    }

    if (distinct) {
      const double dist1 = distance(visible_cities[0]->data, new_node->data);
      new_node->visibility_node_ids[visible_cities[0]] = dist1;
      visible_cities[0]->visibility_node_ids[new_node] = dist1;
      const double dist2 = distance(new_node->data, visible_cities[1]->data);
      new_node->visibility_node_ids[visible_cities[1]] = dist2;
      visible_cities[1]->visibility_node_ids[new_node] = dist2;
      connection_nodes_.push_back(new_node);// 将new_node添加到connection_nodes_容器中，并更新与两个可见节点之间的距离信息
      // // INFO("new pos " << new_node->data)
    }
  }
}

template<class T>
std::vector<HeapNode<T>*> Raptor<T>::getVisibleGuards(
  HeapNode<T>* node) {
  std::vector<HeapNode<T>*> visible_cities;
  // for (size_t i = 0; i < cities_nodes_.size(); i++) {
  //   if (isSimplePathFreeBetweenNodes(cities_nodes_[i], node)) {
  //     visible_cities.push_back(cities_nodes_[i]);
  //   }
  // }
  // 遍历所有守卫节点
  for (size_t i = 0; i < guard_nodes_.size(); i++) {
    // 判断给定节点与当前守卫节点之间是否存在自由路径
    if (map_->isSimplePathFreeBetweenGuards(guard_nodes_[i]->data, node->data, 
                            min_clearance_, collision_distance_check_)) {
      // 将可见的守卫节点加入到可见节点列表中
      visible_cities.push_back(guard_nodes_[i]);
    }
  }
  return visible_cities;
}

template<class T>
std::map<HeapNode<T>*, double> Raptor<T>::nodesBetweenNodes(
  HeapNode<T>* node1, HeapNode<T>* node2) {

  std::map<HeapNode<T>*, double> between_nodes;
  typename std::unordered_map<HeapNode<T>*, double>::iterator it;
  // 遍历第一个节点的可见节点
  // std::vector<HeapNode<T>*> between_nodes;
  for (const auto& vn1 : node1->visibility_node_ids) {
    // 在第一个节点的可见节点中查找第二个节点
    it = vn1.first->visibility_node_ids.find(node2);// 如果找到了第二个节点
    if (it != vn1.first->visibility_node_ids.end()) {// 计算两个节点之间的距离，并将距离和中间节点存入map中
      between_nodes.insert(
        std::pair<HeapNode<T>*, double>(vn1.first, vn1.second + it->second));
    }
  }
  return between_nodes;
}

template<class T>
void Raptor<T>::setBorders(Vector<3> min_position,// 设置地图的边界范围
                                   Vector<3> max_position) {
  min_position_ = min_position;
  max_position_ = max_position;
  position_range_ = max_position - min_position;
}

template<class T>
void Raptor<T>::saveRoadmap(std::string filename) {
  // 打开文件
  // INFO("save Raptor map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    // 遍历守卫节点
    for (size_t i = 0; i < guard_nodes_.size(); i++) {
      // 将守卫节点坐标写入文件
      std::string city_node_str = to_string_raw(guard_nodes_[i]->data);
      myfile << city_node_str << std::endl;
      // 遍历守卫节点的可见节点
      for (const auto& vn1 : guard_nodes_[i]->visibility_node_ids) {
        // 将可见节点的坐标与守卫节点的坐标写入文件，表示它们之间存在连接关系
        std::string neighbor_str = to_string_raw(vn1.first->data);
        ss_connections << city_node_str << "," << neighbor_str << std::endl;
      }
    }
    // for (size_t i = 0; i < connection_nodes_.size(); i++) {
    //   std::string city_node_str = to_string_raw(connection_nodes_[i]->data);
    //   myfile << city_node_str << std::endl;
    //   for (const auto& vn1 : connection_nodes_[i]->visibility_node_ids) {
    //     std::string neighbor_str = to_string_raw(vn1.first->data);
    //     ss_connections << city_node_str << "," << neighbor_str << std::endl;
    //   }
    // }
    myfile << ss_connections.str();
    myfile.close();
  }
}

template<class T>
void Raptor<T>::savePath(std::string filename,
                                 std::vector<HeapNode<T>*> path) {
  // INFO("save Raptor map to file " << filename);
  std::ofstream myfile;// 打开文件
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t ni = 1; ni < path.size(); ni++) {// 遍历路径中的节点
    // 将节点之间的连接关系写入文件
      std::string from_str = to_string_raw(path[ni - 1]->data);
      std::string to_str = to_string_raw(path[ni]->data);
      myfile << from_str << "," << to_str << std::endl;
    }
    myfile.close();
  }
}

// 查找不同的路径
// 将所有节点（除了起点和终点）加入到一个vector中，
// 然后对这些节点进行深度优先搜索，查找不同的路径。
// 最终返回一个包含不同路径的vector
template<class T>
std::vector<path_with_length<T>> Raptor<T>::findDistinctPaths() {
  // INFO("findDistinctPaths begin")
  std::vector<HeapNode<T>*> all_nodes = connection_nodes_;// 将所有节点加入到一个vector中
  all_nodes.insert(all_nodes.end(), guard_nodes_.begin() + 2,
                   guard_nodes_.end());  // do not add start and end,不添加起点和终点
  // 对所有节点进行深度优先搜索，查找不同的路径
  DistinctPathDFS<T> dpDFS(all_nodes, start_, end_);
  std::vector<path_with_length<T>> distinct_paths = dpDFS.findPaths();
  // INFO("findDistinctPaths end")
  return distinct_paths;
}


// 这个函数的目标是将给定的路径进行缩短。
// 首先，创建一个空的路径shortened，并将其长度length初始化为0。
// 然后根据给定的路径path的长度以及碰撞距离检查参数collision_distance_check_来确定采样点的数量num_samples。
// 接下来，利用地图对象map_对路径进行采样，得到一系列采样点sampled。
// 根据参数forward的值，决定从路径的起点或终点开始遍历采样点。
// 对于每个采样点，判断当前节点与下一个节点之间是否存在碰撞。
// 如果存在碰撞，根据碰撞点、梯度信息和采样点之间的关系，计算出新的节点位置，并判断新位置是否可行。
// 如果可行，则创建一个新的节点，并更新与前一个节点的关系信息，同时更新缩短路径的长度和节点列表。
// 最后，将原始路径的最后一个节点添加到缩短路径中，并返回缩短路径shortened。
template <class T>
path_with_length<T>
Raptor<T>::shorten_path(path_with_length<T> path,
                                          bool forward) {
  path_with_length<T> shortened;
  shortened.length = 0;

  // return path;

  const double num_samples = ceil(path.length / collision_distance_check_) + 1;
  // INFO("path.path size " << path.path.size())
  // INFO("num_samples " << num_samples);
  // INFO("collision_distance_check_ " << collision_distance_check_);
  std::vector<T> sampled = map_->samplePath(path.plan, path.length, num_samples);
  // INFO("num samples " << sampled.size());

  // int dir = 1;
  int start_iter = 1;
  if (forward) {
    shortened.plan.push_back(path.plan.front());
  } else {
    shortened.plan.push_back(path.plan.back());
    std::reverse(sampled.begin(), sampled.end());
    // dir = -1;
    // start_iter = sampled.size() - 2;
  }


  // for (size_t i = sampled.size()-2; i < sampled.size(); i += dir) {
  for (size_t i = 1; i < sampled.size(); i++) {
    HeapNode<T>* start_node = shortened.plan.back();
    // INFO("from " << start_node->data.transpose() << " to "
    //              << sampled[i].transpose())


    std::pair<bool, Vector<3>> is_free =
      map_->isSimplePathFreeBetweenNodes(start_node->data, sampled[i], min_clearance_, collision_distance_check_);

    if (!is_free.first) {
      // INFO("not free")
      const Vector<3> collision_place = is_free.second;
      const auto [gradient_in_place, voxel_center] =
        map_->gradientInVoxelCenter(collision_place);
      // INFO("collision in pos " << collision_place.transpose())
      // INFO("gradient_in_place " << gradient_in_place.transpose())
      // go perpendicular to line shortened.path.back()->data, sampled[i]
      // and towards shortest distance to line shortened.path.back()->data,
      // sampled[i-1]
      // const Vector<3> old_point_vector = sampled[i - 1] - start_node->data;


      const Vector<3> new_point_vector = sampled[i] - start_node->data;
      // INFO_VAR(new_point_vector.transpose())
      const Vector<3> normal_gradient_new_point =
        gradient_in_place.cross(new_point_vector);
      // INFO_VAR(normal_gradient_new_point.transpose())

      // const Vector<3> normal_new_old_start =
      //   old_point_vector.cross(new_point_vector);
      Vector<3> vec_from_collision =
        new_point_vector.cross(normal_gradient_new_point);
      vec_from_collision.normalize();
      // INFO_VAR(vec_from_collision.transpose())

      // const double clearance_collision =
      // map_->getClearence(collision_place); INFO("clearance_collision " <<
      // clearance_collision)
      const double ds = collision_distance_check_;

      // double count_check =
      // std::min(collision_distance_check_ / clearance_collision, 1.0);
      // INFO("count_check " << count_check)
      bool added_after_collision = false;
      for (double tci = min_clearance_; tci <= min_clearance_ * 4; tci += ds) {
        const Vector<3> new_point = voxel_center + vec_from_collision * tci;
        // INFO("test collision in new place " << new_point.transpose())
        if (!map_->isInCollision(new_point, min_clearance_)) {
          HeapNode<T>* new_node = new HeapNode<T>(new_point);
          // HeapNode<T>* back_old = shortened.path.back();
          const double dist_new_node = distance(start_node->data, new_point);
          start_node->visibility_node_ids[new_node] = dist_new_node;
          new_node->visibility_node_ids[start_node] = dist_new_node;
          shortened.length += dist_new_node;
          shortened.plan.push_back(new_node);
          added_after_collision = true;
          break;
        } else {
          // INFO("in collision wiht value" << map_->getClearence(new_point))
        }
      }

      if (!added_after_collision) {
        ERROR_RED("no point added to shortened path after collision");
        return path;
        exit(1);
      }
    }
  }

  // INFO("shortened from " << path.length << " to " << shortened.length)
  // INFO("shortened.path.size() " << shortened.path.size())
  // INFO("shorten_path end")

  HeapNode<T>* last_node_original;
  if (forward) {
    last_node_original = path.plan.back();
  } else {
    last_node_original = path.plan.front();
  }

  HeapNode<T>* back_old = shortened.plan.back();
  const double dist_new_node =
    distance(back_old->data, last_node_original->data);
  back_old->visibility_node_ids[last_node_original] = dist_new_node;
  last_node_original->visibility_node_ids[back_old] = dist_new_node;
  shortened.length += dist_new_node;
  shortened.plan.push_back(last_node_original);

  if (!forward) {
    reverse(shortened.plan.begin(), shortened.plan.end());
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


  return shortened;
}


// 这个函数的目标是移除等效的路径。
// 首先，复制输入的路径列表paths到paths_copy。
// 然后，遍历路径列表中的每个路径，逐一与其他路径进行比较，判断是否存在等效路径。
// 判断两条路径是否等效的标准是：以collision_distance_check_为步长，在路径上进行碰撞检测，判断路径是否可变形。
// 如果存在等效路径，则选择长度最短的路径作为代表，并将其他等效路径从列表中移除。
// 最后，返回移除等效路径后的路径列表paths_copy。
template<class T>
std::vector<path_with_length<T>> Raptor<T>::removeEquivalentPaths(
  std::vector<path_with_length<T>> paths) {
  // // INFO_GREEN("removeEquivalentPaths begin with " << paths.size() << " paths")
  // // INFO_VAR(min_clearance_)
  // // INFO_VAR(collision_distance_check_)
  // // INFO_VAR(map_->getResolution())
  std::vector<path_with_length<T>> paths_copy = paths;
  // std::vector<path_with_length<T>> proned;
  if (paths_copy.size() > 1) {
    // bool something_removed = true;
    // while (something_removed) {
    // something_removed = false;

    // for (size_t i = 0; i < paths_copy.size(); i++) {
    size_t i = 0;
    while(i < paths_copy.size()) {
      int shortest_path_i = i;
      double shortest_length = paths_copy[i].length;
      std::vector<int> to_remove_indexes;
      for (size_t j = i + 1; j < paths_copy.size(); j++) {
        const double larger_length =
          std::max(paths_copy[i].length, paths_copy[j].length);
        const double num_check_collision =
          ceil(larger_length / collision_distance_check_) + 1;
        bool deformable = map_->isDeformablePath(
          paths_copy[i].plan, paths_copy[i].length, paths_copy[j].plan,
          paths_copy[j].length, num_check_collision, min_clearance_, collision_distance_check_);
        if (deformable) {
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
  return paths_copy;
}


// removeTooLongPaths函数，其功能是移除过长的路径。
// 根据设定的阈值cutoof_distance_ratio_to_shortest_，将超过最短路径长度一定倍数的路径从列表中移除，
// 并返回移除过长路径后的路径列表。
template<class T>
std::vector<path_with_length<T>> Raptor<T>::removeTooLongPaths(
  std::vector<path_with_length<T>> paths) {
  std::vector<path_with_length<T>> proned = paths;
  // // INFO("removeTooLongPaths begin")

  // find shortest length
  double min_length = DBL_MAX;
  for (auto p : proned) {
    if (p.length < min_length) {
      min_length = p.length;
    }
  }

  // remove the one that are longer than cutoof_distance_ratio_to_shortest_ *
  // min_length
  for (int i = proned.size() - 1; i >= 0; i--) {
    if (proned[i].length > cutoof_distance_ratio_to_shortest_ * min_length) {
      proned.erase(proned.begin() + i);
    }
  }
  // // INFO("remove " << (paths.size() - proned.size())
  //                << " paths due to being too long")
  // // INFO("removeTooLongPaths end")
  return proned;
}



// 这段代码的功能是通过运行 Raptor 算法，找出多个起始和结束位置之间的几何路径。
// 算法的步骤如下：
// 1、创建存储路径的容器 paths_between_gates 和存储 Raptor 实例的容器 topological_prms。
// 2、调整容器 paths_between_gates 和 topological_prms 的大小。
// 3、对于每个起始和结束位置之间，创建一个 Raptor 实例，并设置地图边界。
// 4、对于每个 Raptor 实例，进行采样，生成路径。
// 5、对生成的路径进行缩短处理，移除过长的路径，移除等效的路径。
// 6、对路径进行排序。
// 7、将路径存储在容器 paths_between_gates 中。
// 8、返回所有路径。
template <class T>
std::vector<std::vector<path_with_length<Vector<3>>>>
Raptor<T>::find_geometrical_paths(
    const YAML::Node &planner_config, std::shared_ptr<BaseMap> map,
    std::vector<Vector<3>> &gates_with_start_end_poses,// 保存路径的容器
    std::string output_folder) {

  // INFO("find_geometrical_paths begin")

  auto begin_c = std::chrono::high_resolution_clock::now();
  // std::shared_ptr<PRM<Vector<3>>> prm;
  std::vector<std::vector<path_with_length<Vector<3>>>> paths_between_gates;
  std::vector<std::shared_ptr<Raptor<Vector<3>>>> topological_prms; // 存储每个起始和结束位置之间的路径

  // 调整容器大小
  paths_between_gates.resize(gates_with_start_end_poses.size() - 1);
  topological_prms.resize(gates_with_start_end_poses.size() - 1);

  // 创建每个起始和结束位置之间的 Raptor 实例
  for (size_t i = 0; i < topological_prms.size(); i++) {
    // // INFO("gates_with_start_end_poses[i] " << gates_with_start_end_poses[i])
    // // INFO("gates_with_start_end_poses[i+1] "
    //      << gates_with_start_end_poses[i + 1])
    topological_prms[i] = std::make_shared<Raptor<Vector<3>>>(
      planner_config, map, gates_with_start_end_poses[i],
      gates_with_start_end_poses[i + 1]);
    topological_prms[i]->setBorders(map->getMinPos(), map->getMaxPos());
  }
  // INFO("TopologicalPRM created");

// 对于每个 Raptor 实例
  for (size_t i = 0; i < topological_prms.size(); i++) {
    // 对于每个采样点
    for (size_t si = 0; si < num_samples; si++) {
      topological_prms[i]->sample_point();
    }

    // std::stringstream ss;
    // ss << "roadmap_all_rap" << i << ".csv";    
    // topological_prms[i]->saveRoadmap(ss.str());
    
    // 找到不同的路径
    std::vector<path_with_length<Vector<3>>> diff_paths =
      topological_prms[i]->findDistinctPaths();
    
    auto end_c = std::chrono::high_resolution_clock::now();
    auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);
    INFO_GREEN("algorithm time raptor " << elapsed_c.count() * 1e-9)
    
    // 缩短路径
    std::vector<path_with_length<Vector<3>>> shortened_paths = diff_paths;

    for (size_t pi = 0; pi < diff_paths.size(); pi++) {
      // // INFO("shortening path bef " << pi)
      // // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())
      shortened_paths[pi] =
        Raptor<Vector<3>>::shorten_path(shortened_paths[pi]);
      shortened_paths[pi] =
        Raptor<Vector<3>>::shorten_path(shortened_paths[pi], false);
      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // // INFO("shortening path aft " << pi)
    }

    shortened_paths =
     Raptor<Vector<3>>::removeTooLongPaths(shortened_paths);// 移除过长的路径

    shortened_paths =
     Raptor<Vector<3>>::removeEquivalentPaths(shortened_paths);// 移除等效的路径

      for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
      // // INFO("shortening path bef " << pi)
      // // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())
      shortened_paths[pi] =
        Raptor<Vector<3>>::shorten_path(shortened_paths[pi]);
      shortened_paths[pi] =
        Raptor<Vector<3>>::shorten_path(shortened_paths[pi], false);
      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // // INFO("shortening path aft " << pi)
    }

    shortened_paths =
     Raptor<Vector<3>>::removeEquivalentPaths(shortened_paths);

    std::sort(shortened_paths.begin(), shortened_paths.end(),
              comparator_path_with_length<Vector<3>>);// 对路径进行排序

    paths_between_gates[i] = shortened_paths;// 将路径存储在容器中
    // auto end_c = std::chrono::high_resolution_clock::now();
    // auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);
    // INFO_GREEN("post processing time raptor " << elapsed_c.count() * 1e-9)

  //   for (size_t pi = 0; pi < diff_paths.size(); pi++) {
  //   //   std::stringstream path_ss;
  //   //   // INFO(diff_paths[pi].length)
  //   //   path_ss << "roadmap_path" << i << "_" << pi << ".csv";
  //   //   Raptor<Vector<3>>::savePath(path_ss.str(), diff_paths[pi].plan);
  //   }
  }
  // INFO("samples created");

  return paths_between_gates;
}