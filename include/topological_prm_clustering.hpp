/*
该代码实现的是论文中提出的算法CTopPRM，也是本篇论文核心算法的实现代码！！！
生成概率路图 -> 迭代使用聚类-> 连接簇心 -> 过滤路径/剪枝 -> 最终得到缩短后的不同路径
最为核心的代码是find_geometrical_paths、addCentroid、clusterGraph这三个
分别对应的实现了论文中给出的算法伪代码1-3
*/
#pragma once

#include <algorithm>
#include <flann/flann.hpp>
#include <limits>
#include <memory>
#include <vector>
#include <chrono>
#include <queue>

#include "base_map.hpp"
#include "common.hpp"
#include "dijkstra.hpp"
#include "distinct_path_dfs.hpp"
#include "esdf_map.hpp"
#include "tree_node.hpp"
#include <filesystem>
#include <tuple>

#define PRECISION_PRM_CLUSTERING (10e-6)
#define PRECISION (1E-4)
#define PRINT (true)//(false)


template <typename T> struct ClusterConnection {
  ClusterConnection(){};
  ClusterConnection(double init_dist) { distance = init_dist; }

  HeapNode<T> *node1 = NULL;
  HeapNode<T> *node2 = NULL;
  double distance = DBL_MAX;
  bool deformation_checked = false;
  bool deformation = false;
};

struct ClusterNode {
  int cluster_id;
  std::unordered_set<int> connected_clusters;
};

struct cmp_tuple {
  bool operator()(const std::tuple<int, int>& x, 
                  const std::tuple<int, int>& y) {
    if (std::get<0>(x) == std::get<0>(y))
    {
      return std::get<1>(x) > std::get<1>(y);
    }
  return std::get<0>(x) > std::get<0>(y);
  }
};



//👉构造函数，接受配置参数、地图、输出文件夹路径以及起点和终点作为输入,集成了代码中的全部算法实现函数
template <class T> class TopologicalPRMClustering {
public:

  TopologicalPRMClustering(const YAML::Node &planner_config,
                           std::shared_ptr<BaseMap> map,
                           std::string output_folder, T start, T end,
                           double start_yaw_deg = NAN,
                           double end_yaw_deg = NAN);

  void sampleMultiple(const int num_samples);//对图进行多次采样
  void setBorders(Vector<3> min_position, Vector<3> max_position);//设置边界，即地图的最小和最大位置
  void saveRoadmapDense(std::string filename);//保存密集的路标图
  static void savePath(std::string filename, std::vector<HeapNode<T> *> path);//保存路径
  static void
  savePathsWithGateId(std::string filename,
                      std::vector<path_with_length<T>> multigatepath);//保存包含门ID的路径
  static void savePathSamples(std::string filename, std::vector<T> path);//保存路径上的采样点
  static path_with_length<T> shorten_path(path_with_length<T> path,
                                          bool forward = true);//对路径进行缩短

  std::vector<path_with_length<T>>
  removeWrongGateDirectionPaths(std::vector<path_with_length<T>> paths);//移除错误门方向的路径
  static std::vector<path_with_length<T>>
  removeTooLongPaths(std::vector<path_with_length<T>> paths);//移除过长的路径
  static std::vector<path_with_length<T>>
  removeEquivalentPaths(std::vector<path_with_length<T>> paths);//移除等效的路径

  std::vector<path_with_length<T>> findDistinctPaths();//找到不同的路径
  std::vector<path_with_length<T>> findDistinctPathsBlockingSpheres();//找到在阻塞球中的不同路径
  std::vector<path_with_length<T>>
  findShortestBetween(std::vector<int> start_nodes_vec,
                      std::vector<int> end_nodes_vec, int depth);
  std::vector<path_with_length<T>> findShortestPath();

  static std::vector<Vector<3>>
  samplePath(std::vector<HeapNode<Vector<3>> *> path, const double length_tot,
             const double num_samples);//对路径进行采样

  std::vector<path_with_length<T>> clusterGraph();//对图进行聚类
  void print_min_max_cluster_distances();//打印最小和最大聚类距离
  void findMinClustertours();//寻找最小聚类路径
  bool isNewHomotopyClass(int i, int j);//判断是否是新的同伦类
  std::vector<HeapNode<T> *> backtrackToStart(HeapNode<T> *from);//回溯到起点
  void wavefrontFill();//波前填充
  void addCentroid();//添加质心
  void saveConnections();//保存连接
  void addCentroidWavefrontFill();//添加质心并进行波前填充
  bool isCollisionFreeTriangle(int cluster_id);//判断三角形是否无碰撞
  bool isInTriangleCFree(int idx0, int idx1, int idx2);//判断点是否在无碰撞的三角形中

  std::vector<std::vector<int>> findDistinctPathsDFSOverClusters(bool dist);//在聚类上进行深度优先搜索，找到不同的路径
  void createClusterNodes();//创建聚类节点
  void
  findPathsRecurse(int end_index, std::vector<std::vector<int>> &distinct_paths,//递归地找到路径
                   std::vector<int> path = std::vector<int>(),
                   std::unordered_set<int> visited = std::unordered_set<int>());
  void
  findPathsRecurse(int end_index, std::vector<std::vector<int>> &distinct_paths,
                   int max_depth, std::vector<int> path = std::vector<int>(),
                   std::unordered_set<int> visited = std::unordered_set<int>());
  void 
  findPathsRecurseMaxLength(int end_index, std::vector<std::vector<int>> &distinct_paths,//递归地找到最大长度的路径
                            double current_length, std::vector<int> path, 
                            std::unordered_set<int> visited = std::unordered_set<int>());
  std::tuple<ClusterConnection<T>, ClusterConnection<T>> get_max_connection();//获取最大连接

  static std::vector<std::vector<path_with_length<Vector<3>>>>
  //找到几何路径，最重要的函数！！！
  find_geometrical_paths(const YAML::Node &planner_config,
                         std::shared_ptr<BaseMap> map,
                         std::vector<Vector<3>> &gates_with_start_end_poses,
                         std::string output_folder);

  //获取椭圆的长轴比例
  double getEllipseRatioMajorAxis() {
    return ellipse_ratio_major_axis_focal_length_;
  };
  //设置椭圆的长轴比例
  void
  setEllipseRatioMajorAxis(const double ellipse_ratio_major_axis_focal_length) {
    ellipse_ratio_major_axis_focal_length_ =
        ellipse_ratio_major_axis_focal_length;
  };
  static void removeRoadmapFiles();//删除路图文件

private:
  // void fillRandomState(HeapNode<T> *positionToFill);
  // void fillRandomStateInEllipse(HeapNode<T> *positionToFill, HeapNode<T>
  // *start,
  //                               HeapNode<T> *goal);

  // std::map<HeapNode<T> *, double> nodesBetweenNodes(HeapNode<T> *node1,
  //                                                   HeapNode<T> *node2);

  static std::string to_string_raw(T data);

  static double distance(Vector<3> from, Vector<3> to);

  // std::vector<HeapNode<T>*> cities_nodes_;
  std::vector<HeapNode<T> *> guard_nodes_;
  std::vector<HeapNode<T> *> nodes_;
  int num_clusters_;

  int max_clusters_;
  int min_clusters_;
  double min_ratio_;
  double min_cluster_distance_;
  double max_path_length_ratio_;

  std::vector<double> path_lengths;

  double max_path_length_;
  
  std::vector<HeapNode<T> *> cluster_seed_nodes_;
  std::vector<std::vector<ClusterConnection<T>>> min_cluster_connection_,
      max_cluster_connection_;
  std::vector<std::vector<std::vector<ClusterConnection<T>>>> all_cluster_connections;
  std::vector<ClusterNode> cluster_nodes_;
  std::vector<std::vector<path_with_length<T>>> min_cluster_paths_;
  static std::shared_ptr<BaseMap> map_;
  static std::string output_folder_;
  static double collision_distance_check_;
  static double min_clearance_;
  static double angle_limit_start_end_;
  static double cutoof_distance_ratio_to_shortest_;
  static double ellipse_sampling_density_;
  static double min_allowed;

  double ellipse_ratio_major_axis_focal_length_;

  HeapNode<T> *start_;
  HeapNode<T> *end_;

  bool planar_;

  Quaternion startq_;
  Quaternion endq_;
  bool constraint_start_;
  bool constraint_end_;
  Vector<3> min_position_, max_position_, position_range_;

  Dijkstra<T> dijkstra;
};

template <class T> std::shared_ptr<BaseMap> TopologicalPRMClustering<T>::map_;
template <class T>
double TopologicalPRMClustering<T>::collision_distance_check_;
template <class T> double TopologicalPRMClustering<T>::min_clearance_;
template <class T> double TopologicalPRMClustering<T>::min_allowed;
template <class T> double TopologicalPRMClustering<T>::angle_limit_start_end_;
template <class T>
double TopologicalPRMClustering<T>::cutoof_distance_ratio_to_shortest_;

template <class T>
double TopologicalPRMClustering<T>::ellipse_sampling_density_;

template <class T>
std::string TopologicalPRMClustering<T>::output_folder_("./");

template <class T>
TopologicalPRMClustering<T>::TopologicalPRMClustering(
    const YAML::Node &planner_config, std::shared_ptr<BaseMap> map,
    std::string output_folder, T start, T end, double start_yaw_deg,
    double end_yaw_deg) {
  map_ = map;// 设置地图
  output_folder_ = output_folder;// 设置输出文件夹
  start_ = new HeapNode<T>(start);// 创建起点堆节点
  start_->city_node = true;// 将起点标记为城市节点
  start_->id = 0;// 设置起点id为0
  end_ = new HeapNode<T>(end);// 创建终点堆节点
  end_->city_node = true;// 将终点标记为城市节点
  end_->id = 1;// 设置终点id为1
  min_allowed = (start - end).norm();// 计算起点和终点之间的最小允许距离

  angle_limit_start_end_ = M_PI_2;// 设置起点和终点的角度限制为π/2
  // INFO_VAR(start_yaw_deg)
  // INFO_VAR(end_yaw_deg)
  if (!isnan(start_yaw_deg)) {// 如果起点偏航角度不是NaN
    startq_ = Quaternion(cos((M_PI / 180.0) * start_yaw_deg / 2.0), 0, 0,
                         sin((M_PI / 180.0) * start_yaw_deg / 2.0));// 根据起点偏航角度计算四元数
    constraint_start_ = true;// 设置存在起点约束
  } else {
    constraint_start_ = false;// 否则不存在起点约束
  }
  if (!isnan(end_yaw_deg)) {// 如果终点偏航角度不是NaN
    endq_ = Quaternion(cos((M_PI / 180.0) * end_yaw_deg / 2.0), 0, 0,
                       sin((M_PI / 180.0) * end_yaw_deg / 2.0));// 根据终点偏航角度计算四元数
    constraint_end_ = true;// 设置存在终点约束
  } else {
    constraint_end_ = false;// 否则不存在终点约束
  }

  // // INFO_VAR(constraint_start_)
  // // INFO_VAR(constraint_end_)
  // // INFO(startq_.w() << " " << startq_.vec().transpose())
  // // INFO(endq_.w() << " " << endq_.vec().transpose())
  num_clusters_ = planner_config["num_clusters"].as<int>();// 从规划器配置中获取集群数量
  max_clusters_ = planner_config["max_clusters"].as<int>();// 从规划器配置中获取最大集群数量
  min_clusters_ = planner_config["min_clusters"].as<int>();// 从规划器配置中获取最小集群数量
  min_ratio_ = planner_config["min_ratio"].as<double>();// 从规划器配置中获取最小比率
  planar_ = planner_config["planar"].as<bool>();// 从规划器配置中获取平面标志
  max_path_length_ratio_ = planner_config["max_path_length_ratio"].as<double>(); // 从规划器配置中获取最大路径长度比率
  cutoof_distance_ratio_to_shortest_ =
      loadParam<double>(planner_config, "cutoof_distance_ratio_to_shortest");// 从规划器配置中加载到最短距离的截断距离比率
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");// 从规划器配置中加载最小间隙
  collision_distance_check_ =
      loadParam<double>(planner_config, "collision_distance_check");// 从规划器配置中加载碰撞距离检查
  ellipse_ratio_major_axis_focal_length_ = loadParam<double>(
      planner_config, "ellipse_ratio_major_axis_focal_length");// 从规划器配置中加载椭圆长轴焦距比率
  if (collision_distance_check_ == 0) {// 如果碰撞距离检查为0
    ERROR("you need to specify collision_distance_check for sampling-based "
          "motion "
          "planning");// 输出错误信息
    exit(1);
  }

  guard_nodes_.push_back(start_);// 将起点加入守卫节点列表
  guard_nodes_.push_back(end_);// 将终点加入守卫节点列表
}



/*👉
* PRM construction phase
概率路图的构建阶段
*/
template <class T>
void TopologicalPRMClustering<T>::sampleMultiple(const int num_samples) {
  // // INFO("sampleMultiple begin")
  std::vector<HeapNode<T> *> samples;// 创建存储样本的向量
  const int num_nn = 14;

  nodes_.clear();// 清空节点集合

  // INFO_VAR(constraint_start_)
  // INFO_VAR(constraint_end_)

  flann::Matrix<float> flann_matrix(new float[num_samples * 3], num_samples, 3);// 创建用于存储样本数据的矩阵
  samples.push_back(start_);// 将起始节点和目标节点添加到样本集合中，并将其数据写入矩阵中
  flann_matrix[0][0] = start_->data(0);
  flann_matrix[0][1] = start_->data(1);
  flann_matrix[0][2] = start_->data(2);
  samples.push_back(end_);
  flann_matrix[1][0] = end_->data(0);
  flann_matrix[1][1] = end_->data(1);
  flann_matrix[1][2] = end_->data(2);

  // 对于剩余的样本数量，进行采样
  for (size_t i = 2; i < num_samples; i++) {
    HeapNode<T> *new_node = new HeapNode<T>();// 创建新的节点
    new_node->id = i;

    // 随机生成节点数据，要求生成的节点不发生碰撞
    do {
      map_->fillRandomStateInEllipse(new_node, start_, end_,
                                     ellipse_ratio_major_axis_focal_length_, planar_);
    } while (map_->isInCollision(new_node->data, min_clearance_));

    // 将新节点添加到样本集合中，并将其数据写入矩阵中
    samples.push_back(new_node);
    // INFO_VAR(new_node->data)
    // INFO_VAR(map_->getClearence(new_node->data));
    // 创建 KD 树索引并构建索引
    flann_matrix[i][0] = new_node->data(0);
    flann_matrix[i][1] = new_node->data(1);
    flann_matrix[i][2] = new_node->data(2);
  }
  // // INFO("points sampled")
  flann::IndexParams params = flann::KDTreeIndexParams(4);
  flann::Index<flann::L2<float>> flann_indexes(flann_matrix, params);
  std::vector<std::vector<int>> indices;
  std::vector<std::vector<float>> dists;
  flann_indexes.buildIndex();
  // // INFO("index built")


  // 使用 K 近邻搜索来查找每个样本的最近邻节点
  const int num_found = flann_indexes.knnSearch(
      flann_matrix, indices, dists, num_nn, flann::SearchParams(128));

  INFO("searched with num_found " << num_found)
  // 将起始节点和目标节点之间的向量进行归一化处理
  Vector<3> vec_start_goal_normalized =
      (end_->data - start_->data).normalized();

  // 定义比较函数，用于排序节点之间的距离
  static constexpr auto comparator_distance_point =
      [](const std::pair<double, HeapNode<T> *> &a,
         const std::pair<double, HeapNode<T> *> &b) -> bool {
    return a.first < b.first;
  };

  // // INFO(indices.size())
  // 对于每个样本节点，寻找其最近邻节点
  for (size_t fromi = 0; fromi < num_samples; ++fromi) {
    HeapNode<T> *from_node = samples[fromi];
    // 存储距离最近的邻居节点和对应的距离
    std::vector<std::pair<double, HeapNode<T> *>> distance_neighbors;

    // 对于每个最近邻节点，计算节点之间的向量，并检查是否存在碰撞
    for (size_t nni = 1; nni < num_nn;
         ++nni) { // skip 0 as it is the same point
      const int nnindex = indices[fromi][nni];
      // INFO("nnindex " << nnindex)
      // INFO("dist " << dists[fromi][nni])

      HeapNode<T> *to_node = samples[nnindex];

      Vector<3> vect_between = to_node->data - from_node->data;

      const double vect_between_norm = vect_between.norm();

      // check collisions
      if (map_->isSimplePathFreeBetweenNodes(from_node->data, to_node->data,// 检查路径是否自由及是否存在碰撞
                    min_clearance_, collision_distance_check_).first) {

        // 将邻居节点添加到当前节点的可见节点集合中
        from_node->visibility_node_ids[to_node] = vect_between_norm;
        // from_node->dist_sorted_visibility_node_ids.insert(
        //     {vect_between_norm, to_node});
        
        // distance_neighbors.push_back({vect_between_norm, to_node});
        // 将当前节点添加到邻居节点的可见节点集合中
        to_node->visibility_node_ids[from_node] = vect_between_norm;
        // to_node->dist_sorted_visibility_node_ids.insert(
        //     {vect_between_norm, from_node});
      }
    }
  

    // std::sort(distance_neighbors.begin(), distance_neighbors.end(),
    //           comparator_distance_point);
    // for (size_t i = 0; i < std::min(distance_neighbors.size(), 4ul); i++) {
    //   from_node->visibility_node_ids[distance_neighbors[i].second] =
    //     distance_neighbors[i].first;
    // }
  }
  // 将样本集合赋值给节点集合
  nodes_ = samples;
  // // INFO("sampleMultiple end")
}

// 函数实现了查找两个不同聚类间的最大距离连接和最大比率连接
template <class T>
std::tuple<ClusterConnection<T>, ClusterConnection<T>>
TopologicalPRMClustering<T>::get_max_connection() {
  // 创建存储最大连接的对象
  ClusterConnection<T> max_connection(0);
  ClusterConnection<T> max_ration_connection(0);
  double max_ration = 0;
  // 遍历最大连接矩阵，查找最大距离和最大比率的连接
  for (int i = 0; i < max_cluster_connection_.size(); i++) {
    for (int j = 0; j < max_cluster_connection_[i].size(); j++) {
      if (i < j && max_cluster_connection_[i][j].distance != 0) {
        // 查找最大距离连接
        if (max_cluster_connection_[i][j].distance > max_connection.distance) {
          max_connection = max_cluster_connection_[i][j];
        }

        // 计算连接距离比率，并查找最大比率连接
        double ratio = max_cluster_connection_[i][j].distance /
                       min_cluster_connection_[i][j].distance;
        if (ratio > max_ration) {
          max_ration = ratio;
          max_ration_connection = max_cluster_connection_[i][j];
        }
      }
    }
  }
  // // INFO("max connection " << max_connection.distance << " clusters "
                        //  << max_connection.node1->cluster_id << " "
                        //  << max_connection.node2->cluster_id << " dist start "
                        //  << max_connection.node1->distance_from_start << " "
                        //  << max_connection.node2->distance_from_start)
  // INFO("max_ration_connection "
      //  << max_ration_connection.distance << " clusters "
      //  << max_ration_connection.node1->cluster_id << " "
      //  << max_ration_connection.node2->cluster_id << " dist start "
      //  << max_ration_connection.node1->distance_from_start << " "
      //  << max_ration_connection.node2->distance_from_start << " ratio "
      //  << max_ration)
  // 返回最大距离连接和最大比率连接
  return {max_connection, max_ration_connection};
}


// 该函数用于判断给定聚类ID的三角形是否无碰撞
template <class T>
bool TopologicalPRMClustering<T>::isCollisionFreeTriangle(int cluster_id) {
  // INFO_CYAN("isCollisionFreeTriangle begin")
  //判断三角形是否无碰撞
  std::unordered_set<int> visited;
  visited.insert(cluster_id);
  std::vector<int> path;
  path.push_back(cluster_id);

  // get distinct triangles (two nodes connected to desired cluster id)
  // 获取不同的三角形（连接到所需聚类ID的两个节点）
  std::set<std::pair<int, int>> distinct_paths;
  findPathsRecurse(cluster_id, distinct_paths, 3, path, visited);
  bool is_triangle = !distinct_paths.empty();
  if (is_triangle) {
    // 输出存在的三角形数
    // INFO_GREEN("has " << distinct_paths.size() << " triangles from "
    //                  << cluster_id)
    for (std::pair<int, int> path_middle : distinct_paths) {
      std::stringstream ss;
      // INFO("path " << cluster_id << " " << path_middle.first << " "
      //             << path_middle.second << " " << cluster_id)
    }
  }

  bool deformable = true;
  // 检查三角形是否无碰撞
  // check if the triangle is collision free
  for (std::pair<int, int> path_middle : distinct_paths) {
    bool collision_free = false;
    // sort the nodes from smallest to largest
    // 将节点从小到大排序
    if (cluster_id < path_middle.first) {
      deformable &=
          isInTriangleCFree(cluster_id, path_middle.first, path_middle.second);
    } else if (cluster_id < path_middle.second) {
      deformable &=
          isInTriangleCFree(path_middle.first, cluster_id, path_middle.second);
    } else {
      deformable &=
          isInTriangleCFree(path_middle.first, path_middle.second, cluster_id);
    }
  }

  if (deformable) {// 输出可形变
    // INFO_RED("!!!!!!!!!!!deformable")
    // exit(1);
  } else {
    // 输出不可形变
    // INFO_GREEN("not deformable")
  }
  return deformable;
}


// 判断由三个聚类节点构成的三角形是否无碰撞
template <class T>
bool TopologicalPRMClustering<T>::isInTriangleCFree(int idx0, int idx1,
                                                    int idx2) {
  // INFO_CYAN("isInTriangleCFree begin " << idx0 << ", " << idx1 << ", " << idx2)
  // 找出三条路径中最长的一条
  int max_idx = 0;
  double largest_length = min_cluster_paths_[idx0][idx1].length;
  // INFO("largest_length " << largest_length << " max_idx " << max_idx)
  if (min_cluster_paths_[idx1][idx2].length > largest_length) {
    largest_length = min_cluster_paths_[idx1][idx2].length;
    max_idx = 1;
    // INFO("largest_length " << largest_length << " max_idx " << max_idx)
  }
  if (min_cluster_paths_[idx0][idx2].length > largest_length) {
    largest_length = min_cluster_paths_[idx0][idx2].length;
    max_idx = 2;
    // INFO("largest_length " << largest_length << " max_idx " << max_idx)
  }

  std::vector<HeapNode<T> *> plan1;
  double length1;
  std::vector<HeapNode<T> *> plan2;
  double length2;
  // 根据最大长度的路径选择合适的规划方案
  if (max_idx == 0) {
    // case 0
    // 情况0：以 idx0 和 idx1 为端点的路径长度最长
    plan1 = min_cluster_paths_[idx0][idx1].plan;
    length1 = min_cluster_paths_[idx0][idx1].length;

    plan2 = min_cluster_paths_[idx0][idx2].plan;
    plan2.pop_back();
    plan2.insert(plan2.end(), min_cluster_paths_[idx1][idx2].plan.rbegin(),
                 min_cluster_paths_[idx1][idx2].plan.rend());
    length2 = min_cluster_paths_[idx0][idx2].length +
              min_cluster_paths_[idx1][idx2].length;
  } else if (max_idx == 1) {
    // case 1
    // 情况1：以 idx1 和 idx2 为端点的路径长度最长
    plan1 = min_cluster_paths_[idx1][idx2].plan;
    length1 = min_cluster_paths_[idx1][idx2].length;

    plan2.insert(plan2.end(), min_cluster_paths_[idx0][idx1].plan.rbegin(),
                 min_cluster_paths_[idx0][idx1].plan.rend());
    plan2.pop_back();
    plan2.insert(plan2.end(), min_cluster_paths_[idx0][idx2].plan.begin(),
                 min_cluster_paths_[idx0][idx2].plan.end());
    length2 = min_cluster_paths_[idx0][idx1].length +
              min_cluster_paths_[idx0][idx2].length;
  } else {
    // case 2
    // 情况2：以 idx0 和 idx2 为端点的路径长度最长
    plan1 = min_cluster_paths_[idx0][idx2].plan;
    length1 = min_cluster_paths_[idx0][idx2].length;

    plan2 = min_cluster_paths_[idx0][idx1].plan;
    plan2.pop_back();
    plan2.insert(plan2.end(), min_cluster_paths_[idx1][idx2].plan.begin(),
                 min_cluster_paths_[idx1][idx2].plan.end());

    length2 = min_cluster_paths_[idx0][idx1].length +
              min_cluster_paths_[idx1][idx2].length;
  }

  // 计算检测碰撞的次数
  const double num_check_collision =
      ceil(largest_length / collision_distance_check_) + 1;
  // 使用地图对象的函数检查路径是否可变形（无碰撞）
  bool deformable = map_->isDeformablePath(plan1, length1, plan2, length2,
                                           num_check_collision, min_clearance_,
                                           collision_distance_check_);

  // INFO_CYAN("isInTriangleCFree end " << idx0 << ", " << idx1 << ", " << idx2
  //                                   << " deformable " << deformable)
  return deformable;// 返回路径是否可变形的结果
}


/*👉
* define which clusters are neighbouring each other
* 定义聚类之后的类与类之间互为邻居的情况
*/
//函数分析：根据最小聚类连接信息创建集群节点。
// 首先，通过resize函数为cluster_nodes_数组分配了空间，
// 然后遍历最小的聚类连接数组，根据连接关系将聚类节点之间建立连接关系。
template <class T> void TopologicalPRMClustering<T>::createClusterNodes() {
  // INFO_CYAN("createClusterNodes begin")
  // cluster_nodes_.clear();
  // 重新分配空间，使其大小等于min_cluster_connection_的大小
  cluster_nodes_.resize(min_cluster_connection_.size());
  // 循环处理每个聚类之间的连接情况
  for (int i = 0; i < min_cluster_connection_.size(); i++) {
    cluster_nodes_[i].cluster_id = i;//将cluster_nodes_[i]的cluster_id设置为i
    // 处理min_cluster_connection_[i]中的每个元素，j从i+1到min_cluster_connection_[i].size()-1
    for (int j = i + 1; j < min_cluster_connection_[i].size(); j++) {
      if (min_cluster_connection_[i][j].node1 != NULL && min_cluster_connection_[i][j].node2 != NULL && min_cluster_connection_[i][j].distance != 0) {
        cluster_nodes_[i].connected_clusters.insert(j);// 将j加入到cluster_nodes_[i]的connected_clusters中
        cluster_nodes_[j].connected_clusters.insert(i);// 将i加入到cluster_nodes_[j]的connected_clusters中
      }
    }
  }
  // INFO_CYAN("createClusterNodes end")
}



/*👉
 * used DFS and recursion to find unique shortest paths
 * keep visited nodes in a set to avoid looping back
 1、使用深度优先搜索和递归找到（特殊）最短的路径
 2、在集合中记录访问过的节点，避免环路查找（应该就是重复查找的意思）
 */


/*
在 findDistinctPathsDFSOverClusters 函数中，
首先调用 createClusterNodes 创建聚类节点，然后初始化路径信息，
并调用 findPathsRecurseMaxLength 来进行最大长度路径的递归搜索，最终返回不同的路径。
findPathsRecurse 和 findPathsRecurseMaxLength 函数分别用于递归查找路径和递归查找最大长度路径。
这两个函数的主要区别在于 findPathsRecurseMaxLength 在递归过程中考虑了路径的长度限制，
并记录了路径长度信息。
*/
template <class T>
std::vector<std::vector<int>>
TopologicalPRMClustering<T>::findDistinctPathsDFSOverClusters(bool dist) {
  // 查找不同的聚类间路径的深度优先搜索算法
  // INFO_CYAN("findDistinctPathsDFSOverClusters begin")
  createClusterNodes();// 创建聚类节点
  std::vector<std::vector<int>> distinct_paths;
  std::vector<int> path{start_->cluster_id};// 起始路径为起始聚类的ID
  std::unordered_set<int> visited{start_->cluster_id};// 存储已访问的聚类ID
  // INFO("end_->cluster_id " << end_->cluster_id)
  findPathsRecurseMaxLength(end_->cluster_id, distinct_paths, 0, path, visited);// 递归查找最大长度的路径
  // INFO_CYAN("findDistinctPathsDFSOverClusters end")
  return distinct_paths;// 返回不同的路径
}


template <class T>
void TopologicalPRMClustering<T>::findPathsRecurse(
    // 递归查找路径的函数
    int end_index, std::vector<std::vector<int>> &distinct_paths,
    int desired_depth, std::vector<int> path, std::unordered_set<int> visited) {
  // // INFO_CYAN("findPathsRecurse begin")
  if (path.size() > desired_depth) {
    // 如果当前路径长度大于期望的深度，则返回
    // // INFO("above max depth " << desired_depth)
    return;
  }
  int last_node = path.back();// 获取路径中最后一个节点
  // // INFO_VAR(last_node)
  // 遍历最后一个节点连接的聚类
  for (int node_id : cluster_nodes_[last_node].connected_clusters) {
    if (node_id == end_index) {
      // path_finished
      // 如果遍历到了目标终点聚类
      path.push_back(node_id);// 将终点聚类添加到路径中
      distinct_paths.push_back(path);// 将完整路径添加到distinct_paths中
      path.pop_back();// 移除终点聚类，以便继续探索其他路径
    } else {
      // 如果未到达终点聚类，则继续递归探索
      // recurse
      if (visited.count(node_id) == 0) {// 如果当前聚类未被访问过
        // add node to path and visited
        visited.insert(node_id);// 将其加入到已访问的聚类集合中
        path.push_back(node_id);// 将其加入到路径中
        findPathsRecurse(end_index, distinct_paths, path,// 递归探索下一层路径
                         visited); // do the recursion
        // remove it from the path
        path.pop_back();// 回溯，移除当前聚类
        visited.erase(node_id);// 从已访问的聚类中移除当前聚类
      }
    }
  }
  // // INFO_CYAN("findPathsRecurse end")
}

template <class T>
void TopologicalPRMClustering<T>::findPathsRecurseMaxLength(
  // 递归查找最大长度路径的函数
    int end_index, std::vector<std::vector<int>> &distinct_paths,
    double current_length, std::vector<int> path, std::unordered_set<int> visited) {
  // // INFO_CYAN("findPathsRecurse begin")
  int last_node = path.back();    // 获取路径中最后一个节点
  // // INFO_VAR(last_node)
  for (int node_id : cluster_nodes_[last_node].connected_clusters) {
    // 遍历最后一个节点连接的聚类
    if (node_id == end_index) {
      // path_finished
      // 如果遍历到了目标终点聚类
      path.push_back(node_id);// 将终点聚类添加到路径中
      distinct_paths.push_back(path);// 将完整路径添加到distinct_paths中
      int min_cl, max_cl;
      if (node_id < last_node) {
        min_cl = node_id;
        max_cl = last_node;
      } else {
        min_cl = last_node;
        max_cl = node_id;
      }
      path_lengths.push_back(current_length + min_cluster_paths_[min_cl][max_cl].length);
      path.pop_back();
    } else {
      // recurse
      // 如果未到达终点聚类，则继续递归探索
      if (visited.count(node_id) == 0) {
        // add node to path and visited
        visited.insert(node_id);
        path.push_back(node_id);
        int min_cl, max_cl;
        if (node_id < last_node) {
          min_cl = node_id;
          max_cl = last_node;
        } else {
          min_cl = last_node;
          max_cl = node_id;
        }
        double d = min_cluster_paths_[min_cl][max_cl].length + current_length;// 计算新路径长度
        if (d > max_path_length_) {// 如果新路径长度超过了设定的最大长度，则返回
        // // INFO("above max depth " << desired_depth)
          return;
        }
        else {
          findPathsRecurseMaxLength(end_index, distinct_paths, d, path,
                          visited); // do the recursion
          // remove it from the path
          path.pop_back();// 回溯，移除当前聚类
          visited.erase(node_id);// 从已访问的聚类中移除当前聚类
        }
      }
    }
  }
  // // INFO_CYAN("findPathsRecurse end")
}


/*
findPathsRecurse 函数，用于递归查找路径
该函数采用深度优先搜索（DFS）的方式，在聚类之间进行路径的探索
感觉和前面641行的函数findPathsRecurse()差不多,只是具体参数略有区别
*/
template <class T>
void TopologicalPRMClustering<T>::findPathsRecurse(
    int end_index, std::vector<std::vector<int>> &distinct_paths,
    std::vector<int> path, std::unordered_set<int> visited) {
  // 递归查找路径的函数
  // 参数：
  // - end_index: 终点聚类的索引
  // - distinct_paths: 存储不同路径的向量
  // - path: 当前的路径
  // - visited: 存储已访问过的聚类索引的哈希集合
  // // INFO_CYAN("findPathsRecurse begin")
  int last_node = path.back();// 获取路径中最后一个节点
  // // INFO_VAR(last_node)
  for (int node_id : cluster_nodes_[last_node].connected_clusters) {// 遍历最后一个节点连接的聚类
    if (node_id == end_index) {// 如果遍历到了目标终点聚类
      // path_finished
      path.push_back(node_id);// 将终点聚类添加到路径中
      distinct_paths.push_back(path);// 将完整路径添加到distinct_paths中
      path.pop_back();// 移除终点聚类，以便继续探索其他路径
    } else {// 如果未到达终点聚类，则继续递归探索
      // recurse
      if (visited.count(node_id) == 0) {// 如果当前聚类未被访问过
        // add node to path and visited
        visited.insert(node_id);// 将其加入到已访问的聚类集合中
        path.push_back(node_id);// 将其加入到路径中
        findPathsRecurse(end_index, distinct_paths, path,
                         visited); // do the recursion,递归探索下一层路径
        // remove it from the path
        path.pop_back();
        visited.erase(node_id);
      }
    }
  }
  // // INFO_CYAN("findPathsRecurse end")
}



/*👉
 * from the cluster_seed_nodes_ do a flood fill algorithm to construct MSF
 * also finds the shortest path between clusters - minimium cluster connections
 1、通过cluster_seed_nodes_做一个洪水覆盖算法(Flood Fill)，从而构建多传感器融合算法，其核心就是扩展卡尔曼滤波EKF
 2、通过最小距离的聚类连接找到聚类之间的最短路径
 */
template <class T> void TopologicalPRMClustering<T>::wavefrontFill() {
  // 波前填充算法，用于进行拓扑PRM聚类
  // INFO_CYAN("wavefrontFill begin")

  const int num_cluster_seed_nodes = cluster_seed_nodes_.size();// 获取集群种子节点数量
  // std::vector<double> sum_cluster_dist;
  // sum_cluster_dist.resize(num_cluster_seed_nodes, 0);

  // 初始化连接矩阵和所有连接列表
  min_cluster_connection_.clear();
  max_cluster_connection_.clear();
  all_cluster_connections.clear();

  min_cluster_connection_.resize(
      num_cluster_seed_nodes,
      std::vector<ClusterConnection<T>>(num_cluster_seed_nodes,
                                        ClusterConnection<T>(DBL_MAX)));
  max_cluster_connection_.resize(
      num_cluster_seed_nodes,
      std::vector<ClusterConnection<T>>(num_cluster_seed_nodes,
                                        ClusterConnection<T>(0)));
                                    
  all_cluster_connections.resize(
      num_cluster_seed_nodes, 
      std::vector<std::vector<ClusterConnection<T>>>(num_cluster_seed_nodes, 
                                                     std::vector<ClusterConnection<T>>(0)));

  min_cluster_paths_.resize(max_clusters_);
  for (int i = 0; i < max_clusters_; i++) {
    min_cluster_paths_[i].resize(max_clusters_);
  }

  // 初始化最小路径矩阵
  cluster_nodes_.resize(max_clusters_);

  // 清空距离和集群标记
  // zero the distances and clusters
  for (int var = 0; var < nodes_.size(); ++var) {
    nodes_[var]->distance_from_start = DIJKSTRA_INF;
    nodes_[var]->previous_point = NULL;
    nodes_[var]->cluster_id = -1;
    nodes_[var]->is_border = false;
  }

  // wavefront_open_lists.resize(num_cluster_seed_nodes);
  // add wavefront seed to the open list
  // 将集群种子节点加入到open list中
  for (int i = 0; i < num_cluster_seed_nodes; i++) {
    cluster_seed_nodes_[i]->distance_from_start = 0;
    cluster_seed_nodes_[i]->cluster_id = i;
    // wavefront_open_list.push(cluster_seed_nodes_[i]);
    // // INFO("cluster " << i << " has node " <<
    // wavefront_open_lists[i].get(0)->id)
  }

  Heap<HeapNode<T> *> heap(nodes_);
  INFO("starting loop")

  bool can_expand = true;
  while (can_expand && heap.size() > 0) {
    // // INFO("expanding")
    // 扩展波前直到不再能扩展或者堆为空
    HeapNode<T> *expandingNode = heap.pop();
    if (expandingNode == NULL or
        expandingNode->distance_from_start == DIJKSTRA_INF) {
      can_expand = false;
      break;
    }


    for (auto connectedNode : expandingNode->visibility_node_ids) {// 遍历节点的可见节点
      double calculated_new_distance =
          expandingNode->distance_from_start + connectedNode.second;

      if (calculated_new_distance < connectedNode.first->distance_from_start || connectedNode.first->cluster_id == -1) {
        // 如果发现更短的路径或者节点未被分配到集群，则更新节点信息
        // test point if better distance found
        connectedNode.first->previous_point = expandingNode;
        connectedNode.first->cluster_id = expandingNode->cluster_id;
        // // INFO(connectedNode.first->data.transpose()
        //      << " id " << expandingNode->cluster_id)
        heap.updateCost(connectedNode.first, calculated_new_distance);
      }
    }

    
        // itterate over the neighbors of the node
        for (auto connectedNode : expandingNode->visibility_node_ids) {// 遍历节点的可见节点
          // for (auto connectedNode :
          // expandingNode->dist_sorted_visibility_node_ids) {

          // const double distanceConnectedNode = connectedNode.first;
          // HeapNode<T> *nodeConnectedNode = connectedNode.second;
          const double distanceConnectedNode = connectedNode.second;
          HeapNode<T> *nodeConnectedNode = connectedNode.first;
          double calculated_new_distance =
              expandingNode->distance_from_start + distanceConnectedNode;
          bool updated = false;

          if (nodeConnectedNode->cluster_id == -1) {
            // can expand if it is not assigned to a cluster
            // add to open list
             // 如果可扩展且未被分配到集群，则加入到open list
            nodeConnectedNode->distance_from_start = calculated_new_distance;
            nodeConnectedNode->cluster_id = expandingNode->cluster_id;
            nodeConnectedNode->previous_point = expandingNode;
            // wavefront_open_list.push(nodeConnectedNode);

          } else if (calculated_new_distance <
                     nodeConnectedNode->distance_from_start) {
            // 如果发现更短的路径，则更新节点信息
            updated = true;
            nodeConnectedNode->distance_from_start = calculated_new_distance; 
            nodeConnectedNode->cluster_id = expandingNode->cluster_id; 
            nodeConnectedNode->previous_point = expandingNode;
            // wavefront_open_list.push(nodeConnectedNode);

          } if ((nodeConnectedNode->cluster_id != expandingNode->cluster_id || updated) && expandingNode->cluster_id > -1)
    {
            // update the shortest distance between the clusters
            // only save the connection to [min_cl][max_cl] trianle of the
            // connection matrix
            // 更新集群间的最短路径和最长路径
            // 计算连接的边的长度
            nodeConnectedNode->is_border = true;
            expandingNode->is_border = true;
            int min_cl, max_cl;
            HeapNode<T> *min_node, *max_node;
            if (nodeConnectedNode->cluster_id < expandingNode->cluster_id) {
              min_cl = nodeConnectedNode->cluster_id;
              max_cl = expandingNode->cluster_id;
              min_node = nodeConnectedNode;
              max_node = expandingNode;
            } else {
              min_cl = expandingNode->cluster_id;
              max_cl = nodeConnectedNode->cluster_id;
              min_node = expandingNode;
              max_node = nodeConnectedNode;
            }
            // distance from other cluster's start + from this cluster start +
            // the coonection edge
            double new_distance = nodeConnectedNode->distance_from_start +
                                  expandingNode->distance_from_start +
                                  distanceConnectedNode;

            ClusterConnection<T> conn;
            conn.node1 = min_node;
            conn.node2 = max_node;
            conn.distance = new_distance;
            all_cluster_connections[min_cl][max_cl].push_back(conn);

            if (new_distance < min_cluster_connection_[min_cl][max_cl].distance)// 更新最小连接
    { min_cluster_connection_[min_cl][max_cl].distance = new_distance;
              min_cluster_connection_[min_cl][max_cl].node1 = min_node;
              min_cluster_connection_[min_cl][max_cl].node2 = max_node;
            }
            if (new_distance > max_cluster_connection_[min_cl][max_cl].distance)// 更新最大连接
    { max_cluster_connection_[min_cl][max_cl].distance = new_distance;
              max_cluster_connection_[min_cl][max_cl].node1 = min_node;
              max_cluster_connection_[min_cl][max_cl].node2 = max_node;
            }
          }
        }

        // sum_cluster_dist[min_cluster_id] =
        //     wavefront_open_lists[min_cluster_id].get()->distance_from_start;
        // if (!some_free_found) {
        //   // if no free is found remove the node from the open list
        //   wavefront_open_lists[min_cluster_id].pop();
        // }
        // }

        
  }
  // // print connections
  // for (int i = 0; i < all_cluster_connections.size(); i++) {
  //   for (int j = 0; j < all_cluster_connections[i].size(); j++) {
  //     // INFO("Cluster connections " << i << " " << j)
  //     for (int k = 0; k < all_cluster_connections[i][j].size(); k++) {
  //       // INFO("connection " << k << " : " << all_cluster_connections[i][j][k].node1->data.transpose() << " " <<
  //           all_cluster_connections[i][j][k].node2->data.transpose() << " " << all_cluster_connections[i][j][k].distance)
  //     }
  //   }
  // }

  // print_min_max_cluster_distances();

  // saveConnections();

  bool ratio = true;
  int cl_min, cl_max;

  while (ratio && cluster_seed_nodes_.size() < max_clusters_){
    // 循环计算连接比例，直到满足条件或者达到最大集群数
    ratio = false;

    // save to csv for visualization in blender
    if (PRINT) {
      // 可视化保存节点信息到csv
      for (int i = 0; i < nodes_.size(); i++) {
        // 遍历节点
        std::ofstream myfile;
        std::stringstream ss;
        if (nodes_[i]->cluster_id >= 0) {
          ss << "roadmap_" << cluster_seed_nodes_.size() << "_cluster_" << nodes_[i]->cluster_id << ".csv";
          myfile.open(ss.str().c_str(), std::ios_base::app);

          if (myfile.is_open()) {
            // 计算连接比例并加入优先级队列
            std::string city_node_str = to_string_raw(nodes_[i]->data);
            myfile << city_node_str << std::endl;
            if (nodes_[i]->previous_point != NULL) {
              std::string neighbor_str = to_string_raw(nodes_[i]->previous_point->data);
                myfile << city_node_str << "," << neighbor_str << std::endl;
            }
            myfile.close();
          }
        }
      }
    }

    std::priority_queue<std::tuple<double, int, int> > priorityQueue;
    for (int i = 0; i < max_cluster_connection_.size(); i++) {
      for (int j = 0; j < max_cluster_connection_[i].size(); j++) {
        // if (i < j && max_cluster_connection_[i][j].distance > max_path) {
        // if (i < j && max_cluster_connection_[i][j].distance != 0 && max_cluster_connection_[i][j].node1->distance_from_start > max_path && max_cluster_connection_[i][j].node2->distance_from_start > max_path) {
        if (i < j && max_cluster_connection_[i][j].distance != 0) {
          // INFO("check for " << i << " " << j)
          double new_ratio = max_cluster_connection_[i][j].distance / min_cluster_connection_[i][j].distance;
          std::tuple<double, int, int> tuple = std::make_tuple(new_ratio, i, j);
          priorityQueue.push(tuple);
          // if (cluster_seed_nodes_.size() < max_clusters_ && new_ratio > min_ratio_ && !isNewHomotopyClass(i, j)){
        }
      }
    }

    double max_dist = 0;
    int cl_minB, cl_maxB;
    while (!priorityQueue.empty()) {
      std::tuple<double, int, int> tuple = priorityQueue.top();
      cl_min = std::get<1>(tuple);
      cl_max = std::get<2>(tuple);
      // INFO("checking in order: " << std::get<0>(tuple) << " " << cl_min << " " << cl_max)
      if (!max_cluster_connection_[cl_min][cl_max].deformation_checked) {
        max_cluster_connection_[cl_min][cl_max].deformation = isNewHomotopyClass(cl_min, cl_max);
        max_cluster_connection_[cl_min][cl_max].deformation_checked = true;
      }
      if (!max_cluster_connection_[cl_min][cl_max].deformation) {
        ratio = true;
        break;
      }
      else if (max_cluster_connection_[cl_min][cl_max].distance > max_dist) {
        max_dist = max_cluster_connection_[cl_min][cl_max].distance;
        cl_minB = cl_min;
        cl_maxB = cl_max;
      }
      priorityQueue.pop();
    }

    if (!ratio) {
      if (cluster_seed_nodes_.size() > min_clusters_) {
        break;
      }
      else {
        cl_min = cl_minB;
        cl_max = cl_maxB;
        ratio = true;
      }
    }


    // save minimum and maximum connections to csv for visualization in blender
    // 可视化保存最小和最大连接到csv
    if (PRINT) {
      std::stringstream ss_min;        
      ss_min << "roadmap_" << cluster_seed_nodes_.size() << "_min.csv";
      // std::vector<HeapNode<T> *> small = {min_cluster_connection_[cl_min][cl_max].node1, min_cluster_connection_[cl_min][cl_max].node2};

      path_with_length<T> min_path, max_path;
      int i, j;
      i = cl_min;
      j = cl_max;
      // 获取最小和最大连接的路径信息
      std::vector<HeapNode<T> *> pathNode1 =
          backtrackToStart(min_cluster_connection_[i][j].node1);
      std::vector<HeapNode<T> *> pathNode2 =
          backtrackToStart(min_cluster_connection_[i][j].node2);
      std::reverse(pathNode1.begin(), pathNode1.end());
      std::vector<HeapNode<T> *> path_mincl_maxcl = pathNode1;
      path_mincl_maxcl.insert(path_mincl_maxcl.end(), pathNode2.begin(),
                              pathNode2.end());
      min_path.plan = path_mincl_maxcl;
      min_path.length =
          min_path.calc_path_length();
      min_path.from_id = i;
      min_path.to_id = j;


      pathNode1 =
          backtrackToStart(max_cluster_connection_[i][j].node1);
      pathNode2 =
          backtrackToStart(max_cluster_connection_[i][j].node2);
      std::reverse(pathNode1.begin(), pathNode1.end());
      path_mincl_maxcl = pathNode1;
      path_mincl_maxcl.insert(path_mincl_maxcl.end(), pathNode2.begin(),
                              pathNode2.end());
      max_path.plan = path_mincl_maxcl;
      max_path.length =
          max_path.calc_path_length();
      max_path.from_id = i;
      max_path.to_id = j;
      savePath(ss_min.str(),min_path.plan);
      std::stringstream ss_max;
      ss_max << "roadmap_" << cluster_seed_nodes_.size() << "_max.csv";
      // std::vector<HeapNode<T> *> big = {max_cluster_connection_[cl_min][cl_max].node1, max_cluster_connection_[cl_min][cl_max].node2};
      savePath(ss_max.str(), max_path.plan);
    }

    if (max_cluster_connection_[cl_min][cl_max].node1->distance_from_start > max_cluster_connection_[cl_min][cl_max].node2->distance_from_start)
      cluster_seed_nodes_.push_back(max_cluster_connection_[cl_min][cl_max].node2);
    else
      cluster_seed_nodes_.push_back(max_cluster_connection_[cl_min][cl_max].node1);
    addCentroid();

    // saveConnections();
  }
  // print_min_max_cluster_distances();


  if (PRINT) {
    // 可视化保存最小和最大连接的详细路径信息到csv
    for (int i = 0; i < max_cluster_connection_.size(); i++) {
      for (int j = 0; j < max_cluster_connection_[i].size(); j++) {
        // if (i < j && max_cluster_connection_[i][j].distance > max_path) {
        // if (i < j && max_cluster_connection_[i][j].distance != 0 && max_cluster_connection_[i][j].node1->distance_from_start > max_path && max_cluster_connection_[i][j].node2->distance_from_start > max_path) {
        if (i < j && max_cluster_connection_[i][j].distance != 0) {
          std::stringstream ss_min;        
          ss_min << "roadmap_" << cluster_seed_nodes_.size() << "_min" << i << j << ".csv";
          // std::vector<HeapNode<T> *> small = {min_cluster_connection_[cl_min][cl_max].node1, min_cluster_connection_[cl_min][cl_max].node2};

          path_with_length<T> min_path, max_path;

          std::vector<HeapNode<T> *> pathNode1 =
              backtrackToStart(min_cluster_connection_[i][j].node1);
          std::vector<HeapNode<T> *> pathNode2 =
              backtrackToStart(min_cluster_connection_[i][j].node2);
          std::reverse(pathNode1.begin(), pathNode1.end());
          std::vector<HeapNode<T> *> path_mincl_maxcl = pathNode1;
          path_mincl_maxcl.insert(path_mincl_maxcl.end(), pathNode2.begin(),
                                  pathNode2.end());
          min_path.plan = path_mincl_maxcl;
          min_path.length =
              min_path.calc_path_length();
          min_path.from_id = i;
          min_path.to_id = j;


          pathNode1 =
              backtrackToStart(max_cluster_connection_[i][j].node1);
          pathNode2 =
              backtrackToStart(max_cluster_connection_[i][j].node2);
          std::reverse(pathNode1.begin(), pathNode1.end());
          path_mincl_maxcl = pathNode1;
          path_mincl_maxcl.insert(path_mincl_maxcl.end(), pathNode2.begin(),
                                  pathNode2.end());
          max_path.plan = path_mincl_maxcl;
          max_path.length =
              max_path.calc_path_length();
          max_path.from_id = i;
          max_path.to_id = j;
          savePath(ss_min.str(),min_path.plan);
          std::stringstream ss_max;
          ss_max << "roadmap_" << cluster_seed_nodes_.size() << "_max" << i << j << ".csv";
          // std::vector<HeapNode<T> *> big = {max_cluster_connection_[cl_min][cl_max].node1, max_cluster_connection_[cl_min][cl_max].node2};
          savePath(ss_max.str(), max_path.plan);
        }
      }
    }
  }

  // INFO_CYAN("wavefrontFill end")
}



/*
保存拓扑PRM聚类中最小连接的路径信息，
并对路径进行优化和缩短，最终得到各个聚类之间的连接关系
*/
template <class T> void TopologicalPRMClustering<T>::saveConnections() {
    for (int i = 0; i < 2; i++) {
      int j = cluster_seed_nodes_.size()-1;
      if (min_cluster_connection_[i][j].distance != DBL_MAX && max_cluster_connection_[i][j].distance > 0) {
        // INFO("cluster " << i << " to " << j << " dist "
        //                 << min_cluster_connection_[i][j].distance)
        std::vector<HeapNode<T> *> pathNode1 =
            backtrackToStart(min_cluster_connection_[i][j].node1);
        std::vector<HeapNode<T> *> pathNode2 =
            backtrackToStart(min_cluster_connection_[i][j].node2);
        std::reverse(pathNode1.begin(), pathNode1.end());
        std::vector<HeapNode<T> *> path_mincl_maxcl = pathNode1;
        path_mincl_maxcl.insert(path_mincl_maxcl.end(), pathNode2.begin(),
                                pathNode2.end());
        min_cluster_paths_[i][j].plan = path_mincl_maxcl;
        min_cluster_paths_[i][j].length =
            min_cluster_paths_[i][j].calc_path_length();
        min_cluster_paths_[i][j].from_id = i;
        min_cluster_paths_[i][j].to_id = j;

        min_cluster_paths_[i][j] = shorten_path(min_cluster_paths_[i][j]);
        min_cluster_paths_[i][j] = shorten_path(min_cluster_paths_[i][j], false);
        
        if (min_cluster_paths_[i][j].length < PRECISION) {
          min_cluster_connection_[i][j].node1 = NULL;
        }
        else {
          cluster_nodes_[i].connected_clusters.insert(j);
          cluster_nodes_[j].connected_clusters.insert(i);
        }
      }
    }

    // for (int var = 0; var < goal_indexes.size(); ++var) {
    //   std::vector<HeapNode<T> *> plan;
    //   HeapNode<T> *actualPoint = visibility_graph[goal_indexes[var]];
    // }
}



/*👉
* variaton of wavefrontFill but grows only from the new centroid
波前填充算法的变种，但是只从新的簇心开始生长（填充）
*/


// ❗❗❗ ClusterGraph对应的是论文中伪代码算法第三部分：Adding new Centroid
// 用于向聚类中心列表中添加一个新的聚类中心，并更新簇之间的连接关系
template <class T> void TopologicalPRMClustering<T>::addCentroid() {
  // // INFO_CYAN("add centroid begin")

  // 获取当前聚类中心列表中种子节点的数量和所有节点的数量
  const int num_cluster_seed_nodes = cluster_seed_nodes_.size();
  const double max_nodes = nodes_.size();
  // INFO("MAX NODES IN CLUSTER " << max_nodes)


   /*
    定义一些辅助数据结构，
    包括check_connections（用于检查连接）、
    new_connections（存储新的连接关系）、
    new_min_connections（存储每个种子节点到其他节点的最小距离连接关系）、
    new_max_connections（存储每个种子节点到其他节点的最大距离连接关系）等
  */
  std::vector<bool> check_connections;
  std::vector<std::vector<ClusterConnection<T>>> new_connections;
  new_connections.resize(num_cluster_seed_nodes, std::vector<ClusterConnection<T>>(0));

  std::vector<ClusterConnection<T>> new_min_connections, new_max_connections;
  new_min_connections.resize(num_cluster_seed_nodes, ClusterConnection<T>(DBL_MAX));
  new_max_connections.resize(num_cluster_seed_nodes, ClusterConnection<T>(0));

  // min_cluster_connection_.resize(
  //     num_cluster_seed_nodes,
  //     std::vector<ClusterConnection<T>>(num_cluster_seed_nodes,
  //                                       ClusterConnection<T>(DBL_MAX)));
  // max_cluster_connection_.resize(
  //     num_cluster_seed_nodes,
  //     std::vector<ClusterConnection<T>>(num_cluster_seed_nodes,
  //                                       ClusterConnection<T>(0)));
                                    
  // all_cluster_connections.resize(
  //     num_cluster_seed_nodes, 
  //     std::vector<std::vector<ClusterConnection<T>>>(num_cluster_seed_nodes, 
  //                                                    std::vector<ClusterConnection<T>>(0)));
  std::vector<HeapNode<T> *> exp;

  // cluster_seed_nodes_[cl1]->distance_from_start = 0;
  // cluster_seed_nodes_[cl1]->cluster_id = cl1;
  // cluster_seed_nodes_[cl2]->distance_from_start = 0;
  // cluster_seed_nodes_[cl2]->cluster_id = cl2;
 

  /*
  将最后一个添加的种子节点（即最新加入的节点）初始化为起始节点，并将其距离设为0。
  将该节点加入到exp队列中，用于下一步的扩展
  */
  cluster_seed_nodes_[num_cluster_seed_nodes-1]->distance_from_start = 0;
  cluster_seed_nodes_[num_cluster_seed_nodes-1]->previous_point = NULL;
  cluster_seed_nodes_[num_cluster_seed_nodes-1]->cluster_id = num_cluster_seed_nodes-1;
  cluster_seed_nodes_[num_cluster_seed_nodes-1]->is_border = false;

  // exp.push_back(cluster_seed_nodes_[cl1]);
  // exp.push_back(cluster_seed_nodes_[cl2]);
  exp.push_back(cluster_seed_nodes_[num_cluster_seed_nodes-1]);
  // // wavefront_open_lists.resize(num_cluster_seed_nodes);
  // // add wavefront seed to the open list
  // for (int i = 0; i < num_cluster_seed_nodes; i++) {
  //   cluster_seed_nodes_[i]->distance_from_start = 0;
  //   cluster_seed_nodes_[i]->cluster_id = i;
  //   // wavefront_open_list.push(cluster_seed_nodes_[i]);
  //   // // INFO("cluster " << i << " has node " <<
  //   // wavefront_open_lists[i].get(0)->id)
  // }

  int cnt = 1;

/*
使用Dijkstra算法计算从起始节点到其他所有节点的最短路径。
从exp队列中取出一个节点进行扩展。
对于节点的每一个可见节点，计算新的距离，并将连接关系更新到新的数据结构中。
如果新的距离小于可见节点当前的距离，
并且当前可见节点没有被分配到任何簇中（cluster_id为-1），
则更新可见节点的相关属性，并将其加入到exp队列中
*/
  Heap<HeapNode<T> *> heap(exp);

  bool can_expand = true;
  while (heap.size() > 0 && can_expand) {

    HeapNode<T> *expandingNode = heap.pop();
    if (expandingNode == NULL or
        expandingNode->distance_from_start == DIJKSTRA_INF) {
      can_expand = false;
      break;
    }
    for (auto connectedNode : expandingNode->visibility_node_ids) {
      double calculated_new_distance =
          expandingNode->distance_from_start + connectedNode.second;

      if ((calculated_new_distance < connectedNode.first->distance_from_start && cnt < max_nodes) || connectedNode.first->cluster_id == -1 ) {
        // test point if better distance found
        connectedNode.first->previous_point = expandingNode;
        connectedNode.first->cluster_id = expandingNode->cluster_id;
        connectedNode.first->distance_from_start = calculated_new_distance;
        connectedNode.first->is_border = false;
        // // INFO(connectedNode.first->data.transpose()
        //      << " id " << expandingNode->cluster_id)
        heap.push(connectedNode.first);
        heap.updateCost(connectedNode.first, calculated_new_distance);
        cnt++;
      }
    }

    /*
    更新簇之间的连接关系，
    包括最小和最大距离的连接关系。
    如果两个节点分别属于不同的簇，则将它们标记为边
    */
    for (auto connectedNode : expandingNode->visibility_node_ids) {
        const double distanceConnectedNode = connectedNode.second;
          HeapNode<T> *nodeConnectedNode = connectedNode.first;
          double calculated_new_distance =
              expandingNode->distance_from_start + distanceConnectedNode;

           if (nodeConnectedNode->cluster_id != expandingNode->cluster_id) {
            
            nodeConnectedNode->is_border = true;
            expandingNode->is_border = true;
            // update the shortest distance between the clusters
            // only save the connection to [min_cl][max_cl] trianle of the
            // connection matrix         
            
            double new_distance = nodeConnectedNode->distance_from_start +
                                  expandingNode->distance_from_start +
                                  distanceConnectedNode;

            ClusterConnection<T> conn;
            conn.node1 = nodeConnectedNode;
            conn.node2 = expandingNode;
            conn.distance = new_distance;
            new_connections[nodeConnectedNode->cluster_id].push_back(conn);

            if (new_distance < new_min_connections[nodeConnectedNode->cluster_id].distance)
    { new_min_connections[nodeConnectedNode->cluster_id].distance = new_distance;
              new_min_connections[nodeConnectedNode->cluster_id].node1 = nodeConnectedNode;
              new_min_connections[nodeConnectedNode->cluster_id].node2 = expandingNode;
            }
            if (new_distance > new_max_connections[nodeConnectedNode->cluster_id].distance)
    { new_max_connections[nodeConnectedNode->cluster_id].distance = new_distance;
              new_max_connections[nodeConnectedNode->cluster_id].node1 = nodeConnectedNode;
              new_max_connections[nodeConnectedNode->cluster_id].node2 = expandingNode;
            }
          }
      }
    }

    // 将新的最小和最大距离连接关系以及新的连接关系添加到对应的簇之间的连接关系列表中
    for (int i=0;i<num_cluster_seed_nodes-1;i++) {
      min_cluster_connection_[i].push_back(new_min_connections[i]);
      max_cluster_connection_[i].push_back(new_max_connections[i]);
      all_cluster_connections[i].push_back(new_connections[i]);
    }


    /*
    对于每一对簇之间的连接关系，检查是否存在无效的连接关系（节点不属于对应的簇或距离为0）。
    如果存在无效连接关系，则将最小和最大连接关系的距离设为最大值和0，并清空相关节点和标记。
    同时，从连接关系列表中删除无效的连接关系。
    然后，更新最小和最大连接关系的距离和节点，以保留最小和最大的连接关系
    */
    for (int i=0;i<num_cluster_seed_nodes-1;i++) {
      int min_cl = i;
      for (int j=i+1;j<num_cluster_seed_nodes;j++) {
        int max_cl = j;
        if (max_cluster_connection_[min_cl][max_cl].distance == 0 || max_cluster_connection_[min_cl][max_cl].node1->cluster_id != i || 
            max_cluster_connection_[min_cl][max_cl].node2->cluster_id != j || min_cluster_connection_[min_cl][max_cl].node1->cluster_id != i ||
            min_cluster_connection_[min_cl][max_cl].node2->cluster_id != j) {
          min_cluster_connection_[min_cl][max_cl].distance = DBL_MAX;
          min_cluster_connection_[min_cl][max_cl].node1 = NULL;
          min_cluster_connection_[min_cl][max_cl].node2 = NULL;
          max_cluster_connection_[min_cl][max_cl].distance = 0;
          max_cluster_connection_[min_cl][max_cl].node1 = NULL;
          max_cluster_connection_[min_cl][max_cl].node2 = NULL;
          max_cluster_connection_[min_cl][max_cl].deformation_checked = false;
          int k = all_cluster_connections[min_cl][max_cl].size() - 1;
          while (k >= 0) {
            if (all_cluster_connections[min_cl][max_cl][k].node1->cluster_id != min_cl ||
                all_cluster_connections[min_cl][max_cl][k].node2->cluster_id != max_cl) {
                    all_cluster_connections[min_cl][max_cl].erase(all_cluster_connections[min_cl][max_cl].begin()+k);
            }
            else if (all_cluster_connections[min_cl][max_cl][k].node1->cluster_id == min_cl && 
                    all_cluster_connections[min_cl][max_cl][k].node2->cluster_id == max_cl){
              if (all_cluster_connections[min_cl][max_cl][k].distance < min_cluster_connection_[min_cl][max_cl].distance) {
                min_cluster_connection_[min_cl][max_cl].distance = all_cluster_connections[min_cl][max_cl][k].distance;
                min_cluster_connection_[min_cl][max_cl].node1 = all_cluster_connections[min_cl][max_cl][k].node1;
                min_cluster_connection_[min_cl][max_cl].node2 = all_cluster_connections[min_cl][max_cl][k].node2;
              }
              if (all_cluster_connections[min_cl][max_cl][k].distance > max_cluster_connection_[min_cl][max_cl].distance) {
                max_cluster_connection_[min_cl][max_cl].distance = all_cluster_connections[min_cl][max_cl][k].distance;
                max_cluster_connection_[min_cl][max_cl].node1 = all_cluster_connections[min_cl][max_cl][k].node1;
                max_cluster_connection_[min_cl][max_cl].node2 = all_cluster_connections[min_cl][max_cl][k].node2;
              }
            }
            k--;
          }
        }
      }
    }  
  // 在函数的最后，根据种子节点的数量调整最小和最大连接关系的大小，并初始化连接关系列表
    min_cluster_connection_.resize(
        num_cluster_seed_nodes,
        std::vector<ClusterConnection<T>>(num_cluster_seed_nodes,
                                          ClusterConnection<T>(DBL_MAX)));
    max_cluster_connection_.resize(
        num_cluster_seed_nodes,
        std::vector<ClusterConnection<T>>(num_cluster_seed_nodes,
                                          ClusterConnection<T>(0)));
    all_cluster_connections.resize(
        num_cluster_seed_nodes,
        std::vector<std::vector<ClusterConnection<T>>>(num_cluster_seed_nodes,
                                          std::vector<ClusterConnection<T>>(0)));
}

// 将每个可见区域分配到不同的聚类集群中，并计算不同聚类集群之间的最短路径和最长路径
template <class T> void TopologicalPRMClustering<T>::addCentroidWavefrontFill() {
  // INFO_CYAN("wavefrontFill begin")

  const int num_cluster_seed_nodes = cluster_seed_nodes_.size();
  // std::vector<double> sum_cluster_dist;
  // sum_cluster_dist.resize(num_cluster_seed_nodes, 0);
  // 初始化聚类间的最短距离、最长距离以及所有聚类间的边的列表
  min_cluster_connection_.clear();
  max_cluster_connection_.clear();
  all_cluster_connections.clear();

  min_cluster_connection_.resize(
      num_cluster_seed_nodes,
      std::vector<ClusterConnection<T>>(num_cluster_seed_nodes,
                                        ClusterConnection<T>(DBL_MAX)));
  max_cluster_connection_.resize(
      num_cluster_seed_nodes,
      std::vector<ClusterConnection<T>>(num_cluster_seed_nodes,
                                        ClusterConnection<T>(0)));
                                    
  all_cluster_connections.resize(
      num_cluster_seed_nodes, 
      std::vector<std::vector<ClusterConnection<T>>>(num_cluster_seed_nodes, 
                                                     std::vector<ClusterConnection<T>>(0)));

  // zero the distances and clusters
  // 将所有节点的距离、聚类 ID、前驱节点以及是否为边界节点等信息进行初始化
  for (int var = 0; var < nodes_.size(); ++var) {
    nodes_[var]->distance_from_start = DIJKSTRA_INF;
    nodes_[var]->previous_point = NULL;
    nodes_[var]->cluster_id = -1;
    nodes_[var]->is_border = false;
  }

  // wavefront_open_lists.resize(num_cluster_seed_nodes);
  // add wavefront seed to the open list
  // 将所有聚类种子节点加入到堆中，并将它们的距离从起点设置为 0
  for (int i = 0; i < num_cluster_seed_nodes; i++) {
    cluster_seed_nodes_[i]->distance_from_start = 0;
    cluster_seed_nodes_[i]->cluster_id = i;
    // wavefront_open_list.push(cluster_seed_nodes_[i]);
    // // INFO("cluster " << i << " has node " <<
    // wavefront_open_lists[i].get(0)->id)
  }
  /*
  实现拓扑PRM:
  首先从堆中取出距离起点最近的节点，并遍历该节点的可见节点，
  更新它们的距离、聚类 ID 和前驱节点，并将它们加入到堆中。
  对于每个可见节点，如果它未被分配到聚类集群中，则将其加入到该聚类集群中。
  如果当前节点与它的可见节点属于不同的聚类集群，则计算它们之间的距离，
  并更新聚类间的边的距离。
  最后，更新聚类间的最短距离、最长距离以及所有聚类间的边的列表
  */
  Heap<HeapNode<T> *> heap(nodes_);
  INFO("starting loop")

  bool can_expand = true;
  while (can_expand && heap.size() > 0) {
    // // INFO("expanding")

    HeapNode<T> *expandingNode = heap.pop();
    if (expandingNode == NULL or
        expandingNode->distance_from_start == DIJKSTRA_INF) {
      can_expand = false;
      break;
    }


    for (auto connectedNode : expandingNode->visibility_node_ids) {
      double calculated_new_distance =
          expandingNode->distance_from_start + connectedNode.second;

      if (calculated_new_distance < connectedNode.first->distance_from_start || connectedNode.first->cluster_id == -1) {
        // test point if better distance found
        connectedNode.first->previous_point = expandingNode;
        connectedNode.first->cluster_id = expandingNode->cluster_id;
        // // INFO(connectedNode.first->data.transpose()
        //      << " id " << expandingNode->cluster_id)
        heap.updateCost(connectedNode.first, calculated_new_distance);
      }
    }

    
        // itterate over the neighbors of the node
        for (auto connectedNode : expandingNode->visibility_node_ids) {
          // for (auto connectedNode :
          // expandingNode->dist_sorted_visibility_node_ids) {

          // const double distanceConnectedNode = connectedNode.first;
          // HeapNode<T> *nodeConnectedNode = connectedNode.second;
          const double distanceConnectedNode = connectedNode.second;
          HeapNode<T> *nodeConnectedNode = connectedNode.first;
          double calculated_new_distance =
              expandingNode->distance_from_start + distanceConnectedNode;
          bool updated = false;

          if (nodeConnectedNode->cluster_id == -1) {
            // can expand if it is not assigned to a cluster
            // add to open list
            nodeConnectedNode->distance_from_start = calculated_new_distance;
            nodeConnectedNode->cluster_id = expandingNode->cluster_id;
            nodeConnectedNode->previous_point = expandingNode;
            // wavefront_open_list.push(nodeConnectedNode);

          } else if (calculated_new_distance <
                     nodeConnectedNode->distance_from_start) {
            updated = true;
            nodeConnectedNode->distance_from_start = calculated_new_distance; 
            nodeConnectedNode->cluster_id = expandingNode->cluster_id; 
            nodeConnectedNode->previous_point = expandingNode;
            // wavefront_open_list.push(nodeConnectedNode);

          } if ((nodeConnectedNode->cluster_id != expandingNode->cluster_id || updated) && expandingNode->cluster_id > -1)
    {
            // update the shortest distance between the clusters
            // only save the connection to [min_cl][max_cl] trianle of the
            // connection matrix
            nodeConnectedNode->is_border = true;
            expandingNode->is_border = true;
            int min_cl, max_cl;
            HeapNode<T> *min_node, *max_node;
            if (nodeConnectedNode->cluster_id < expandingNode->cluster_id) {
              min_cl = nodeConnectedNode->cluster_id;
              max_cl = expandingNode->cluster_id;
              min_node = nodeConnectedNode;
              max_node = expandingNode;
            } else {
              min_cl = expandingNode->cluster_id;
              max_cl = nodeConnectedNode->cluster_id;
              min_node = expandingNode;
              max_node = nodeConnectedNode;
            }
            // distance from other cluster's start + from this cluster start +
            // the coonection edge


            /*
            计算所有聚类集群之间的最短路径和最长路径，并保存它们之间的连接边
            */
            double new_distance = nodeConnectedNode->distance_from_start +
                                  expandingNode->distance_from_start +
                                  distanceConnectedNode;

            ClusterConnection<T> conn;
            conn.node1 = min_node;
            conn.node2 = max_node;
            conn.distance = new_distance;
            all_cluster_connections[min_cl][max_cl].push_back(conn);
    

    /*
    对于每个可见节点和它与之相邻的节点，
    更新它们之间的距离、所属聚类 ID 和前驱节点。
    如果当前节点与它的相邻节点属于不同的聚类集群，
    则将它们之间的距离更新到聚类间的边的列表中
    */
            if (new_distance < min_cluster_connection_[min_cl][max_cl].distance)
    { min_cluster_connection_[min_cl][max_cl].distance = new_distance;
              min_cluster_connection_[min_cl][max_cl].node1 = min_node;
              min_cluster_connection_[min_cl][max_cl].node2 = max_node;
            }
            if (new_distance > max_cluster_connection_[min_cl][max_cl].distance)
    { max_cluster_connection_[min_cl][max_cl].distance = new_distance;
              max_cluster_connection_[min_cl][max_cl].node1 = min_node;
              max_cluster_connection_[min_cl][max_cl].node2 = max_node;
            }
          }
        }

        // sum_cluster_dist[min_cluster_id] =
        //     wavefront_open_lists[min_cluster_id].get()->distance_from_start;
        // if (!some_free_found) {
        //   // if no free is found remove the node from the open list
        //   wavefront_open_lists[min_cluster_id].pop();
        // }
        // }

        
  }
  // // print connections
  // for (int i = 0; i < all_cluster_connections.size(); i++) {
  //   for (int j = 0; j < all_cluster_connections[i].size(); j++) {
  //     // INFO("Cluster connections " << i << " " << j)
  //     for (int k = 0; k < all_cluster_connections[i][j].size(); k++) {
  //       // INFO("connection " << k << " : " << all_cluster_connections[i][j][k].node1->data.transpose() << " " <<
  //           all_cluster_connections[i][j][k].node2->data.transpose() << " " << all_cluster_connections[i][j][k].distance)
  //     }
  //   }
  // }
  // print_min_max_cluster_distances();

  // INFO_CYAN("wavefrontFill end")
}


/*👉
 * bactrack from node to the start and return the nodes on the path
 从给定节点开始回溯，直到起点，并返回路径上的所有节点
 */
template <class T>
std::vector<HeapNode<T> *>
TopologicalPRMClustering<T>::backtrackToStart(HeapNode<T> *from) {
  // 从给定节点开始回溯到起点，并返回路径上的所有节点
  std::vector<HeapNode<T> *> path;
  path.push_back(from);

  HeapNode<T> *current_node = from;
  while (current_node->previous_point != NULL) {
    current_node = current_node->previous_point;
    path.push_back(current_node);
  }
  return path;
}



/*👉
* check if connections between neighbouring clusters are deformable
检查相邻簇心之间的连接是否是同伦（可变换的）的
*/
template <class T> bool TopologicalPRMClustering<T>::isNewHomotopyClass(int i, int j) {
// 判断是否为新的同伦类
    path_with_length<T> min_path, max_path;

    // 回溯到起点，构建最小连接路径
    std::vector<HeapNode<T> *> pathNode1 =
        backtrackToStart(min_cluster_connection_[i][j].node1);
    std::vector<HeapNode<T> *> pathNode2 =
        backtrackToStart(min_cluster_connection_[i][j].node2);
    std::reverse(pathNode1.begin(), pathNode1.end());
    std::vector<HeapNode<T> *> path_mincl_maxcl = pathNode1;
    path_mincl_maxcl.insert(path_mincl_maxcl.end(), pathNode2.begin(),
                            pathNode2.end());
    min_path.plan = path_mincl_maxcl;
    min_path.length =
        min_path.calc_path_length();
    min_path.from_id = i;
    min_path.to_id = j;



    // 回溯到起点，构建最大连接路径
    pathNode1 =
        backtrackToStart(max_cluster_connection_[i][j].node1);
    pathNode2 =
        backtrackToStart(max_cluster_connection_[i][j].node2);
    std::reverse(pathNode1.begin(), pathNode1.end());
    path_mincl_maxcl = pathNode1;
    path_mincl_maxcl.insert(path_mincl_maxcl.end(), pathNode2.begin(),
                            pathNode2.end());
    max_path.plan = path_mincl_maxcl;
    max_path.length =
        max_path.calc_path_length();
    max_path.from_id = i;
    max_path.to_id = j;


    // 对最小连接路径进行缩短
    min_path = shorten_path(min_path, false);
    min_path = shorten_path(min_path);

    // 对最大连接路径进行缩短
    max_path = shorten_path(max_path);
    max_path = shorten_path(max_path, false);

    // 计算更长的路径长度，并根据碰撞距离检查设置碰撞次数
    const double larger_length =
      std::max(min_path.length, max_path.length);
    const double num_check_collision =
      ceil(larger_length / collision_distance_check_) + 1;


    min_cluster_paths_[i][j] = min_path;
    
    // 调用地图类中函数判断是否为可变形路径
    return map_->isDeformablePath(min_path.plan, min_path.length, max_path.plan, max_path.length,
                                  num_check_collision, min_clearance_, collision_distance_check_);

    // for (int var = 0; var < goal_indexes.size(); ++var) {
    //   std::vector<HeapNode<T> *> plan;
    //   HeapNode<T> *actualPoint = visibility_graph[goal_indexes[var]];
    // }
  }


/*👉
 * find paths between clusters using the min_cluster_connection_
 使用查找最小簇路径的算法，找到聚类之间的最短路径
 */
/*
1、回溯到起点，获取最小距离连接关系的两个节点分别回溯得到的路径；
2、将这两个路径拼接成从一个簇到另一个簇的完整路径；
3、计算完整路径的长度，并将其记录到最小簇路径列表中；
4、对计算出来的最小簇路径进行优化（缩短路径长度）；
5、判断路径长度是否小于一个给定的精度值，如果小于，则将最小连接关系的其中一个节点设为NULL。
*/
template <class T> void TopologicalPRMClustering<T>::findMinClustertours() {
  // 查找最小簇路径的算法
  // INFO_CYAN("findMinClustertours start")
  // min_cluster_paths_.resize(min_cluster_connection_.size());
  // // INFO("res 1")
  for (int i = 0; i < min_cluster_connection_.size(); i++) {
    // min_cluster_paths_[i].resize(min_cluster_connection_[i].size());
    // // INFO("res 2")
    for (int j = 0; j < min_cluster_connection_[i].size(); j++) {
      if (i<j && !max_cluster_connection_[i][j].deformation_checked && 
                  min_cluster_connection_[i][j].distance != DBL_MAX && max_cluster_connection_[i][j].distance > 0) {
        // 如果最大距离连接关系没有被检查过，并且最小距离连接关系存在且距离大于0
        // INFO("cluster " << i << " to " << j << " dist "
        //                 << min_cluster_connection_[i][j].distance)

        // 回溯到起点，获取最小距离连接关系的两个节点分别回溯得到的路径
        std::vector<HeapNode<T> *> pathNode1 =
            backtrackToStart(min_cluster_connection_[i][j].node1);
        std::vector<HeapNode<T> *> pathNode2 =
            backtrackToStart(min_cluster_connection_[i][j].node2);
        std::reverse(pathNode1.begin(), pathNode1.end());

        // 将这两个路径拼接成从一个簇到另一个簇的完整路径
        std::vector<HeapNode<T> *> path_mincl_maxcl = pathNode1;
        path_mincl_maxcl.insert(path_mincl_maxcl.end(), pathNode2.begin(),
                                pathNode2.end());

        // 计算完整路径的长度，并将其记录到最小簇路径列表中
        min_cluster_paths_[i][j].plan = path_mincl_maxcl;
        min_cluster_paths_[i][j].length =
            min_cluster_paths_[i][j].calc_path_length();
        min_cluster_paths_[i][j].from_id = i;
        min_cluster_paths_[i][j].to_id = j;


        // 对计算出来的最小簇路径进行优化（缩短路径长度）
        min_cluster_paths_[i][j] = shorten_path(min_cluster_paths_[i][j]);
        min_cluster_paths_[i][j] = shorten_path(min_cluster_paths_[i][j], false);
        
        // 判断路径长度是否小于一个给定的精度值
        if (min_cluster_paths_[i][j].length < PRECISION) {
          min_cluster_connection_[i][j].node1 = NULL;// 如果路径长度小于精度值，则将最小连接关系的其中一个节点设为NULL
        }
      }
    }

    // for (int var = 0; var < goal_indexes.size(); ++var) {
    //   std::vector<HeapNode<T> *> plan;
    //   HeapNode<T> *actualPoint = visibility_graph[goal_indexes[var]];
    // }
  }

  // INFO_CYAN("findMinClustertours end")
}


/*👉
* print ratios of cluster connections
打印聚类连接的情况，最大距离、最小距离、最大最小距离的比值
*/
template <class T>
void TopologicalPRMClustering<T>::print_min_max_cluster_distances() {
  for (int i = 0; i < max_cluster_connection_.size(); i++) {
    for (int j = 0; j < max_cluster_connection_[i].size(); j++) {
      if (i < j && max_cluster_connection_[i][j].distance != 0 && min_cluster_connection_[i][j].node1 != NULL) {
        INFO("cluster " << i << " to " << j)
        INFO("min "<< min_cluster_connection_[i][j].distance << " " << min_cluster_connection_[i][j].node1->data.transpose()
          << " "<< min_cluster_connection_[i][j].node2->data.transpose())
        INFO("max "<< max_cluster_connection_[i][j].distance << " " << max_cluster_connection_[i][j].node1->data.transpose()
          << " "<< max_cluster_connection_[i][j].node2->data.transpose())
        INFO("ratio " << max_cluster_connection_[i][j].distance / min_cluster_connection_[i][j].distance)

        // if (max_cluster_connection_[i][j].distance >
        // max_connection.distance)
        // {
        //   max_connection = max_cluster_connection_[i][j];
        // }
      }
    }
  }
}


/*👉
 * create clusters in the PRM graph
 在PRM中创建聚类
 */

// ❗❗❗ ClusterGraph对应的是论文中伪代码算法第二部分：Graph Clustering
// 实现图聚类的算法
template <class T> 
std::vector<path_with_length<T>>
TopologicalPRMClustering<T>::clusterGraph() {
  // INFO_CYAN("clusterGraph begin")

  // first randomly select n nodesfrom the prm and create initial number of
  // clusters int num_cluster_ = 3;

  // 首先从prm中随机选择n个节点，并创建初始数量的簇
  std::unordered_set<int> node_ids_used;

  // 将起点和终点作为种子节点
  cluster_seed_nodes_.push_back(start_);
  cluster_seed_nodes_.push_back(end_);
  node_ids_used.insert(start_->id);
  node_ids_used.insert(end_->id);
  while (cluster_seed_nodes_.size() < num_clusters_) {
    const int rand_node_idx = randIntMinMax(2, nodes_.size() - 1);// 从nodes_中随机选择一个节点
    // INFO_VAR(rand_node_idx)
    // INFO_VAR(nodes_[rand_node_idx]->id)
    HeapNode<T> *rand_node = nodes_[rand_node_idx];
    if (node_ids_used.count(nodes_[rand_node_idx]->id) == 0) {// 确保该节点未被使用过
      // check if the new node is connected directly to other cluster
      bool to_add = true;// 标记是否需要将该节点添加到种子节点中
      int num_free_path_between = 0;// 两个簇之间的自由路径数量
      int num_no_free_path_between = 0;// 两个簇之间的非自由路径数量
      for (int i = 0; i < cluster_seed_nodes_.size(); i++) {
        // check simple connection first
        // 检查节点间是否存在简单连接
        if (cluster_seed_nodes_[i]->visibility_node_ids.count(rand_node) > 0) {
          to_add = false;
          // // INFO("simple connection " << cluster_seed_nodes_.size())
          break;
        }
        //

        // 检查两个节点之间是否存在自由路径
        std::pair<bool, Vector<3>> path_between =
            map_->isSimplePathFreeBetweenNodes(cluster_seed_nodes_[i]->data,
                                               rand_node->data, min_clearance_,
                                               collision_distance_check_);
        if (path_between.first) {
          num_free_path_between++;
        }
        else {
          num_no_free_path_between++;
        }
        if (num_free_path_between >= 1) {  
          // // INFO("free path between too many times" << cluster_seed_nodes_.size())
          to_add = false;
          break;
        }
      }


      // 如果满足条件则将该节点添加到种子节点中
      if (to_add && num_no_free_path_between >= 2) {
        cluster_seed_nodes_.push_back(nodes_[rand_node_idx]);
        node_ids_used.insert(nodes_[rand_node_idx]->id);
      }
    }
  }
  // INFO("we have clusters")
  // std::vector<path_with_length<Vector<3>>> shortest_test = findShortestPath();
  // max_path_length_ = shortest_test[0].length * max_path_length_ratio_;


  // 计算波前填充
  auto begin2 = std::chrono::high_resolution_clock::now();
  wavefrontFill();
  // createClusterNodes();  
  auto end2 = std::chrono::high_resolution_clock::now();
  auto elapsed2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2);
  INFO_GREEN("computation time wavefront " << elapsed2.count() * 1e-9)

  findMinClustertours();  // 找到最小簇之间的路径

  // // INFO_GREEN("distinct_cluster_paths.size() no limitation " << distinct_cluster_paths_0.size())
  // // INFO_GREEN("computation time " << elapsed.count() * 1e-9)

  auto begin = std::chrono::high_resolution_clock::now();
  

  // 使用深度优先搜索找到不同簇之间的路径
  std::vector<std::vector<int>> distinct_cluster_paths =
      findDistinctPathsDFSOverClusters(true);

  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  INFO_GREEN("computation time DFS " << elapsed.count() * 1e-9)

  std::vector<path_with_length<T>> distinct_paths;

  // construct paths as a set of waypoints from an ordered set of clusters
  // 从有序的簇集合中构建路径
  for (int i = 0; i < distinct_cluster_paths.size(); ++i) {
    std::vector<HeapNode<T> *> path;
    // INFO("path " << i)
    for (int j = 1; j < distinct_cluster_paths[i].size(); ++j) {
      const int from_id = distinct_cluster_paths[i][j - 1];
      const int to_id = distinct_cluster_paths[i][j];
      // INFO("from_id " << from_id << " to_id " << to_id)
      if (!path.empty()) {
        // do not repeate same node in the path
        path.pop_back();
      }
      if (from_id < to_id) {
        path.insert(path.end(),
        min_cluster_paths_[from_id][to_id].plan.begin(),
                    min_cluster_paths_[from_id][to_id].plan.end());
      } else {
        path.insert(path.end(),
                    min_cluster_paths_[to_id][from_id].plan.rbegin(),
                    min_cluster_paths_[to_id][from_id].plan.rend());
      }
    }
    if (path[path.size()-1]->data == end_->data) {
      path_with_length<T> new_path;
      new_path.plan = path;
      new_path.length = path_lengths[i];
      distinct_paths.push_back(new_path);
    }
  }


  // save all important information to csv for blender visualization
  // 将重要信息保存到csv文件中用于可视化
  if (PRINT) {
    // 保存最小簇之间的路径
    for (int i = 0; i < min_cluster_paths_.size(); i++) {
      for (int j = 0; j < min_cluster_paths_[i].size(); j++) {
        if (!min_cluster_paths_[i].empty() && i < j && cluster_nodes_[i].connected_clusters.count(j) > 0) {
          std::stringstream ss_name;
          ss_name << "roadmap_path_cluster_" << i << "_" << j << ".csv";
          savePath(ss_name.str(), min_cluster_paths_[i][j].plan);
        }
      }
    }


    // 保存不同簇之间的路径
    for (int i = 0; i < distinct_cluster_paths.size(); ++i) {
      std::stringstream ss_name;
      ss_name << "roadmap_distinct_path_" << i << ".csv";

      std::vector<HeapNode<T> *> path;
      // INFO("path " << i)
      for (int j = 1; j < distinct_cluster_paths[i].size(); ++j) {
        const int from_id = distinct_cluster_paths[i][j - 1];
        const int to_id = distinct_cluster_paths[i][j];
        // INFO("from_id " << from_id << " to_id " << to_id)
        if (!path.empty()) {
          // do not repeate same node in the path
          path.pop_back();
        }
        if (from_id < to_id) {
          path.insert(path.end(),
          min_cluster_paths_[from_id][to_id].plan.begin(),
                      min_cluster_paths_[from_id][to_id].plan.end());
        } else {
          path.insert(path.end(),
                      min_cluster_paths_[to_id][from_id].plan.rbegin(),
                      min_cluster_paths_[to_id][from_id].plan.rend());
        }
      }
      if (path[path.size()-1]->data == end_->data) {  
        savePath(ss_name.str(), path);
      }
    }

    // //print it
    // for (int i = 0; i < nodes_.size(); i++) {
    //   // INFO(i << " cluster " << nodes_[i]->cluster_id << " id " <<
    //   nodes_[i]->id)
    // }


    // save it to file
    // 保存节点信息到文件
    for (int i = 0; i < nodes_.size(); i++) {
      std::ofstream myfile;
      std::stringstream ss;
      if (nodes_[i]->cluster_id >= 0) {
        ss << "roadmap_cluster_" << nodes_[i]->cluster_id << ".csv";
        myfile.open(ss.str().c_str(), std::ios_base::app);

        if (myfile.is_open()) {

          std::string city_node_str = to_string_raw(nodes_[i]->data);
          myfile << city_node_str << std::endl;
          if (nodes_[i]->previous_point != NULL) {
            std::string neighbor_str = to_string_raw(nodes_[i]->previous_point->data);
              myfile << city_node_str << "," << neighbor_str << std::endl;
          }
          // for (const auto &vn1 : nodes_[i]->visibility_node_ids) {
          //   if (vn1.first->cluster_id >= 0) /* or
          //       nodes_[i]->distance_from_start < vn1.first->distance_from_start)*/ {
          //     std::string neighbor_str = to_string_raw(vn1.first->data);
          //     myfile << city_node_str << "," << neighbor_str << std::endl;
          //   }
          // }
          myfile.close();
        }
      }
    }


    // 保存种子节点信息到文件
    std::ofstream seed_file;
    seed_file.open("roadmap_seeds_cluster.csv");
    if (seed_file.is_open()) {
      for (int i = 0; i < cluster_seed_nodes_.size(); i++) {
        std::string city_node_str = to_string_raw(cluster_seed_nodes_[i]->data);
        seed_file << city_node_str << std::endl;
      }
      seed_file.close();
    }

  }

  // INFO_CYAN("clusterGraph end")
  return distinct_paths;// 返回不同簇之间的路径
}


// 设置边界
template <class T>
void TopologicalPRMClustering<T>::setBorders(Vector<3> min_position,
                                             Vector<3> max_position) {
  min_position_ = min_position;
  max_position_ = max_position;
  position_range_ = max_position - min_position;
}

// 保存稠密的路图
template <class T>
void TopologicalPRMClustering<T>::saveRoadmapDense(std::string filename) {
  // // INFO("save TopologicalPRM map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {

    for (size_t i = 0; i < nodes_.size(); i++) {
      if (nodes_[i]->cluster_id >= 0) {
        std::string city_node_str = to_string_raw(nodes_[i]->data);
        myfile << city_node_str << std::endl;
        for (const auto &vn1 : nodes_[i]->visibility_node_ids) {
          if (vn1.first->cluster_id >= 0) {
            std::string neighbor_str = to_string_raw(vn1.first->data);
            ss_connections << city_node_str << "," << neighbor_str << std::endl;
          }
        }
      }
    }
    myfile << ss_connections.str();
    myfile.close();
  }
}


// 保存路径
template <class T>
void TopologicalPRMClustering<T>::savePath(std::string filename,
                                           std::vector<HeapNode<T> *> path) {
  // // INFO("save TopologicalPRM map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t ni = 1; ni < path.size(); ni++) {
      std::string from_str = to_string_raw(path[ni - 1]->data);
      std::string to_str = to_string_raw(path[ni]->data);
      myfile << from_str << "," << to_str << std::endl;
    }
    myfile.close();
  }
}

// 保存带有GateID的路径
template <class T>
void TopologicalPRMClustering<T>::savePathsWithGateId(
    std::string filename, std::vector<path_with_length<T>> multigatepath) {
  // // INFO("save TopologicalPRM map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t gi = 0; gi < multigatepath.size(); gi++) {
      std::vector<HeapNode<T> *> &path = multigatepath[gi].plan;
      for (size_t ni = 1; ni < path.size(); ni++) {
        std::string from_str = to_string_raw(path[ni - 1]->data);
        std::string to_str = to_string_raw(path[ni]->data);
        myfile << gi << "," << from_str << "," << to_str << std::endl;
      }
    }
    myfile.close();
  }
}


// 保存采样的路径
template <class T>
void TopologicalPRMClustering<T>::savePathSamples(std::string filename,
                                                  std::vector<T> path) {
  // // INFO("savePathSamples to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t ni = 0; ni < path.size(); ni++) {
      std::string from_str = to_string_raw(path[ni]);
      myfile << from_str << std::endl;
    }
    myfile.close();
  }
}


// 使用深度优先搜索（DFS）算法找到起点到终点的不同路径
template <class T>
std::vector<path_with_length<T>>
TopologicalPRMClustering<T>::findDistinctPaths() {
  // INFO("findDistinctPaths begin")
  std::vector<HeapNode<T> *> all_nodes = nodes_;
  all_nodes.insert(all_nodes.end(), guard_nodes_.begin() + 2,
                   guard_nodes_.end()); // do not add start and end
  DistinctPathDFS<T> dpDFS(all_nodes, start_, end_);
  std::vector<path_with_length<T>> distinct_paths = dpDFS.findPaths();
  // INFO("findDistinctPaths end")
  return distinct_paths;
}



// 使用Dijkstra算法找到从节点0到节点{1}的最短路径
template <class T>
std::vector<path_with_length<T>>
TopologicalPRMClustering<T>::findShortestPath() {
  // Dijkstra<T> dijkstra;
  std::vector<path_with_length<T>> shortest_plan =
      dijkstra.findPath(0, {1}, nodes_);
  // // INFO("shortest_plan size " << shortest_plan.size())
  // // INFO_VAR(shortest_plan[0].length)
  // // INFO_VAR(shortest_plan[0].plan.size())
  return shortest_plan;
}


/*👉
* prune too long paths
剪枝掉太长的路径
*/
/*
移除给定路径列表中过长的路径。
首先，函数会找到路径列表中最短的路径长度，并将其保存在min_length中
然后，函数遍历路径列表，
将长度大于最短路径长度乘以截断距离比例或小于最小允许长度的路径从列表中移除
*/
template <class T>
std::vector<path_with_length<T>>
TopologicalPRMClustering<T>::removeTooLongPaths(
    std::vector<path_with_length<T>> paths) {
  std::vector<path_with_length<T>> proned = paths;
  // // INFO("removeTooLongPaths begin")

  // find shortest length
  // 查找最短路径长度
  double min_length = DBL_MAX;

  for (auto p : proned) {
    if (p.length < min_length && p.length > min_allowed) {
      min_length = p.length;
    }
  }

  // remove the one that are longer than cutoof_distance_ratio_to_shortest_
  // * min_length
  // 移除超过最短路径长度乘以截断距离比例的路径以及长度小于最小允许长度的路径
  for (int i = proned.size() - 1; i >= 0; i--) {
    if (proned[i].length > cutoof_distance_ratio_to_shortest_ * min_length || proned[i].length < min_allowed) {
      proned.erase(proned.begin() + i);
    }
  }
  // // INFO("remove " << (paths.size() - proned.size())
  //                << " paths due to being too long")
  // // INFO("removeTooLongPaths end")
  return proned;
}


/*👉
* prune redundant paths
移除掉多余（等效/同伦）的路径
*/
/*
遍历给定的路径列表，找出其中等效的路径并进行移除。
路径等效的定义是：如果两条路径之间可以通过形变（变形）来互相转换，
即一条路径可以通过改变节点的顺序或插入/删除节点来变成另一条路径，那么它们就是等效的。
在查找等效路径时，会使用碰撞检测算法来判断路径是否是可形变的。
如果存在多条等效路径，则只保留其中最短的一条路径
*/
template<class T>
std::vector<path_with_length<T>> TopologicalPRMClustering<T>::removeEquivalentPaths(
  std::vector<path_with_length<T>> paths) {
  // // INFO_GREEN("removeEquivalentPaths begin with " << paths.size() << " paths")
  // // INFO_VAR(min_clearance_)
  // // INFO_VAR(collision_distance_check_)
  // // INFO_VAR(map_->getResolution())
  std::vector<path_with_length<T>> paths_copy = paths;// 创建一个路径副本
  // std::vector<path_with_length<T>> proned;
  if (paths_copy.size() > 1) {// 如果路径数量大于1
    // bool something_removed = true;
    // while (something_removed) {
    // something_removed = false;

    // for (size_t i = 0; i < paths_copy.size(); i++) {
    size_t i = 0;
    while(i < paths_copy.size()) {// 对于每条路径
      int shortest_path_i = i;
      double shortest_length = paths_copy[i].length;
      std::vector<int> to_remove_indexes;
      // 对于当前路径之后的每一条路径
      for (size_t j = i + 1; j < paths_copy.size(); j++) {
        // 计算两条路径的最大长度和碰撞检查次数
        const double larger_length =
          std::max(paths_copy[i].length, paths_copy[j].length);
        const double num_check_collision =
          ceil(larger_length / collision_distance_check_) + 1;
        // 判断是否为可变形路径
        bool deformable = map_->isDeformablePath(
          paths_copy[i].plan, paths_copy[i].length, paths_copy[j].plan,
          paths_copy[j].length, num_check_collision, min_clearance_, collision_distance_check_);
        if (deformable) {
          // 如果可变形，记录需要移除的路径索引，并更新最短路径索引和长度
          // INFO("path " << i << " is deformable to " << j)
          // ith_unique = false;
          to_remove_indexes.push_back(j);
          if (paths_copy[j].length < shortest_length) {
            shortest_path_i = j;
            shortest_length = paths_copy[j].length;
          }
        } else {
          // INFO("path " << i << " not deformable to " << j)
        }
      }
      // 如果存在更短的可变形路径，更新当前路径为最短路径
      if (shortest_path_i != i) {
        paths_copy[i] = paths_copy[shortest_path_i];
      }
      // 移除被标记的等效路径
      for (int tri = to_remove_indexes.size() - 1; tri >= 0; tri--) {
        // INFO("removing " << to_remove_indexes[tri])
        paths_copy.erase(paths_copy.begin() + to_remove_indexes[tri]);
        // // INFO("size " << to_remove_indexes[tri])
      }
      // // INFO("purged")
      //}
       // 继续处理下一条路径
      i++;
    }
  }
  // // INFO_GREEN("removeEquivalentPaths end")
  return paths_copy;// 返回去除等效路径后的路径集合
}


/*👉
shorten the path by maximizing the collision-free distance of straight lines
starting from the start and adding points that are obtained from the collision
place and pushed away from obstacles
通过最大化直线间的无碰撞距离，来缩短路径
从起点开始，然后增加远离碰撞的点
在障碍物内的点将被清除掉
*/


// 实现缩短路径的函数
template<class T>
path_with_length<T> TopologicalPRMClustering<T>::shorten_path(path_with_length<T> path,
                                                    bool forward) {
  // INFO_GREEN("shorten_path begin with forward " << forward)
  // INFO("path from " << path.plan.front()->data.transpose() << " and "
  //                   << path.plan.back()->data.transpose())

  // 创建一个空的缩短后的路径 shortened，并初始化路径长度为0
  path_with_length<T> shortened;
  shortened.length = 0;

  // return path;

  // 根据给定的路径 path 的长度和碰撞距离检查参数 collision_distance_check_ 计算采样点的数量 num_samples
  const double num_samples = ceil(path.length / collision_distance_check_) + 1;
  // INFO("path.path size " << path.path.size())
  // INFO("num_samples " << num_samples);
  // INFO("collision_distance_check_ " << collision_distance_check_);

  // 使用地图对象 map_ 对路径 path 中的节点进行采样，得到一组采样点 sampled
  std::vector<T> sampled = map_->samplePath(path.plan, path.length, num_samples);
  // INFO("num samples " << sampled.size());

  // int dir = 1;
  int start_iter = 1;

  // 根据参数 forward 来确定缩短路径的起始节点
  if (forward) {
    shortened.plan.push_back(path.plan.front());// 若 forward 为 true，则将路径 path 的起始节点添加到 shortened 中
  } else {
    shortened.plan.push_back(path.plan.back());
    // 若 forward 为 false，则将路径 path 的结束节点添加到 shortened 中，并将采样点序列 sampled 反转
    std::reverse(sampled.begin(), sampled.end());
    // dir = -1;
    // start_iter = sampled.size() - 2;
  }


  // for (size_t i = sampled.size()-2; i < sampled.size(); i += dir) {
  // 遍历采样点序列 sampled，从第二个采样点开始（即 i=1）
  for (size_t i = 1; i < sampled.size(); i++) {
    // 获取 shortened 路径中的最后一个节点 start_node
    HeapNode<T>* start_node = shortened.plan.back();
    // INFO("from " << start_node->data.transpose() << " to "
    //              << sampled[i].transpose())

    // 判断从 start_node 到 sampled[i] 的简单路径是否自由（即没有碰撞）
    std::pair<bool, Vector<3>> is_free =
      map_->isSimplePathFreeBetweenNodes(start_node->data, sampled[i], min_clearance_, collision_distance_check_);

    if (!is_free.first) {
      // INFO("not free")

      // 若不自由，则找到碰撞点 collision_place 和碰撞处的梯度 gradient_in_place
      const Vector<3> collision_place = is_free.second;
      const auto [gradient_in_place, voxel_center] =
        map_->gradientInVoxelCenter(collision_place);
      // INFO("collision in pos " << collision_place.transpose())
      // INFO("gradient_in_place " << gradient_in_place.transpose())
      // go perpendicular to line shortened.path.back()->data, sampled[i]
      // and towards shortest distance to line shortened.path.back()->data,
      // sampled[i-1]
      // const Vector<3> old_point_vector = sampled[i - 1] - start_node->data;

      // 计算新采样点 sampled[i] 与 start_node 的向量 new_point_vector
      const Vector<3> new_point_vector = sampled[i] - start_node->data;
      // INFO_VAR(new_point_vector.transpose())
      // 计算新采样点梯度的法向量 normal_gradient_new_point
      const Vector<3> normal_gradient_new_point =
        gradient_in_place.cross(new_point_vector);
      // INFO_VAR(normal_gradient_new_point.transpose())


      // 计算从碰撞点出发的垂直于 new_point_vector 和 
      // normal_gradient_new_point 的向量 vec_from_collision，并进行归一化
      // const Vector<3> normal_new_old_start =
      //   old_point_vector.cross(new_point_vector);
      Vector<3> vec_from_collision =
        new_point_vector.cross(normal_gradient_new_point);
      vec_from_collision.normalize();
      // INFO_VAR(vec_from_collision.transpose())

      // const double clearance_collision =
      // map_->getClearence(collision_place); INFO("clearance_collision " <<
      // clearance_collision)
      // 通过逐步增加距离 tci，计算新的采样点 new_point，
      // 并判断是否与碰撞检测距离 min_clearance_ 内无碰撞
      const double ds = collision_distance_check_;

      // double count_check =
      // std::min(collision_distance_check_ / clearance_collision, 1.0);
      // INFO("count_check " << count_check)
      bool added_after_collision = false;
      for (double tci = min_clearance_; tci <= min_clearance_ * 4; tci += ds) {
        const Vector<3> new_point = voxel_center + vec_from_collision * tci;
        // INFO("test collision in new place " << new_point.transpose())
        if (!map_->isInCollision(new_point, min_clearance_)) {
          // 若存在满足条件的 new_point，则创建一个新的节点 new_node，并将其添加到 shortened 中，
          // 并更新路径长度和节点间的可见性关系
          HeapNode<T>* new_node = new HeapNode<T>(new_point);
          // HeapNode<T>* back_old = shortened.path.back();
          const double dist_new_node = distance(start_node->data, new_point);
          start_node->visibility_node_ids[new_node] = dist_new_node;
          new_node->visibility_node_ids[start_node] = dist_new_node;
          shortened.length += dist_new_node;
          shortened.plan.push_back(new_node);
          added_after_collision = true;
          break;
        } 
      }

      if (!added_after_collision) {
        // 若不存在满足条件的 new_point，则输出错误信息并返回原始路径 path
        ERROR_RED("no point added to shortened path after collision");
        return path;
        exit(1);
      }
    }
  }

  // INFO("shortened from " << path.length << " to " << shortened.length)
  // INFO("shortened.path.size() " << shortened.path.size())
  // INFO("shorten_path end")


  // 将路径中最后一个节点 last_node_original 添加到 shortened 中，并更新路径长度和节点间的可见性关系
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
    // 若参数 forward 为 false，则将 shortened 的节点顺序反转
    reverse(shortened.plan.begin(), shortened.plan.end());
  }


  // debuging begin
  // check distance
  // 检查缩短后的路径长度是否与计算得到的长度相等，并且最后一个节点是否与原始路径 path 的最后一个节点相同
  // 如果不相等，则输出错误信息并退出程序
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


  return shortened;// 返回缩短后的路径 shortened
}



// ❗❗❗ 对应的是论文中伪代码算法第一部分：CTopPRM
// 集合了整个算法流程的函数，包括了前面的Graph Clustring 和 Adding New Centroid
// 大致流程为：1、生成概率路图，2、增加簇心并迭代使用聚类，3、连接簇心，4、过滤路径，5、最终得到缩短的路径
// 输出结果为最终的几何(拓扑不同)路径，也就是shortened paths
template <class T>
std::vector<std::vector<path_with_length<Vector<3>>>>
TopologicalPRMClustering<T>::find_geometrical_paths(
    const YAML::Node &planner_config, std::shared_ptr<BaseMap> map,
    std::vector<Vector<3>> &gates_with_start_end_poses,
    std::string output_folder) {
  INFO("find_geometrical_paths begin")

// 该函数实现了整个算法流程，包括了前面的图聚类（Graph Clustering）和添加新的聚类中心（Adding New Centroid）
// 参数：
// - planner_config: 配置文件节点，包含算法的参数设置
// - map: 地图对象，用于路径规划
// - gates_with_start_end_poses: 门之间的起始和终点位置
// - output_folder: 输出结果的文件夹路径
// 返回值：
// - 最终的几何路径集合，即缩短后的路径集合

// 计时开始，便于对比结果中给出计算时间
  auto begin_c = std::chrono::high_resolution_clock::now();
  // gates_with_start_end_poses.resize(2);
  // gates_with_start_end_poses =
  //   std::vector<Vector<3>>(gates_with_start_end_poses.begin() + 1,
  //                          gates_with_start_end_poses.begin() + 3);


  // 创建TopologicalPRMClustering对象数组
  std::vector<std::shared_ptr<TopologicalPRMClustering<Vector<3>>>>
      topological_prms;


  // 存储门之间的路径集合
  std::vector<std::vector<path_with_length<Vector<3>>>> paths_between_gates;
  // // INFO_VAR(gates_with_start_end_poses.size())

  int num_samples_between_gate =
      loadParam<double>(planner_config, "num_samples_between_gate");


  // 根据门的数量调整路径集合的大小
  paths_between_gates.resize(gates_with_start_end_poses.size() - 1);
  topological_prms.resize(gates_with_start_end_poses.size() - 1);

  for (size_t i = 0; i < topological_prms.size(); i++) {
    // for (size_t i = 0; i < 3; i++) {
    // create the prms first
    // INFO("gates_with_start_end_poses[i] " << gates_with_start_end_poses[i])
    // INFO("gates_with_start_end_poses[i+1] "
        //  << gates_with_start_end_poses[i + 1]);

    // // INFO_VAR(first_gate_yaw_deg)
    // // INFO_VAR(second_gate_yaw_deg)

    // double dx = gates_with_start_end_poses[i][0] - gates_with_start_end_poses[i + 1][0];
    // double dy = gates_with_start_end_poses[i][1] - gates_with_start_end_poses[i + 1][1];
    // double dz = gates_with_start_end_poses[i][2] - gates_with_start_end_poses[i + 1][2];
    // max_path_length_ = sqrt(dx*dx + dy*dy + dz*dz) * max_path_length_ratio_;

    // 创建TopologicalPRMClustering对象
    topological_prms[i] = std::make_shared<TopologicalPRMClustering<Vector<3>>>(
        planner_config, map, output_folder, gates_with_start_end_poses[i],
        gates_with_start_end_poses[i + 1]);
    // INFO("map min pos " << map->getMinPos())
    // INFO("map max pos " << map->getMaxPos())

    // 设置地图边界
    Vector<3> min_pos = map->getMinPos();
    Vector<3> max_pos = map->getMaxPos();
    topological_prms[i]->setBorders(min_pos, max_pos);

    // 对当前对象进行多次采样
    auto begin_p = std::chrono::high_resolution_clock::now();
    topological_prms[i]->sampleMultiple(num_samples_between_gate);
    auto end_p = std::chrono::high_resolution_clock::now();
    auto elapsed_p = std::chrono::duration_cast<std::chrono::nanoseconds>(end_p - begin_p);
    INFO_GREEN("prm time clustering " << elapsed_p.count() * 1e-9)

    // 找到最短路径
    std::vector<path_with_length<Vector<3>>> shortest_test =
        topological_prms[i]->findShortestPath();
    auto end_p2 = std::chrono::high_resolution_clock::now();
    auto elapsed_p2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end_p2 - end_p);
    INFO_GREEN("djikstra time clustering " << elapsed_p2.count() * 1e-9)
    if (shortest_test.size() == 0 || shortest_test[0].plan.size() == 0) {// 如果找不到合适的路径，则返回空结果
      INFO("no path found between gates " << i << " and " << (i + 1))
      // exit(1);
      return paths_between_gates;
      
      // if (topological_prms[i]->getEllipseRatioMajorAxis() < 3.0) {
      //   topological_prms[i]->setEllipseRatioMajorAxis(
      //       topological_prms[i]->getEllipseRatioMajorAxis() * 1.5);
      // }
      
      // num_samples_between_gate = num_samples_between_gate * 1.5;
      // topological_prms[i]->sampleMultiple(num_samples_between_gate);
      // shortest_test = topological_prms[i]->findShortestPath();
    }


    // 根据最短路径的长度设置当前对象的最大路径长度
    topological_prms[i]->max_path_length_ = shortest_test[0].length * topological_prms[i]->max_path_length_ratio_;
    INFO("path len " << topological_prms[i]->max_path_length_)


    // 对当前对象进行图聚类，得到不同的路径集合
    std::vector<path_with_length<Vector<3>>> diff_paths = topological_prms[i]->clusterGraph();
  
    // save PRM
     // 如果需要，保存PRM，通过宏定义来保存PRM路图文件即.csv文件
    if (PRINT) {
      std::stringstream ss;
      ss << output_folder_ << "roadmap_all" << i << ".csv";
      topological_prms[i]->saveRoadmapDense(ss.str());
    }


    // 输出算法运行时间
    auto end_c = std::chrono::high_resolution_clock::now();
    auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);
    INFO_GREEN("algorithm time clustering " << elapsed_c.count() * 1e-9)

    INFO("found " << diff_paths.size() << " paths")


    // 缩短路径
    std::vector<path_with_length<Vector<3>>> shortened_paths = diff_paths;
    for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
      // // INFO("shortening path bef " << pi)
      // // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())

      shortened_paths[pi] =
        TopologicalPRMClustering<Vector<3>>::shorten_path(shortened_paths[pi], false);
      shortened_paths[pi] =
        TopologicalPRMClustering<Vector<3>>::shorten_path(shortened_paths[pi]);

      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // // INFO("shortening path aft " << pi)
    }


    // 过滤掉过长的路径
    shortened_paths =
     TopologicalPRMClustering<Vector<3>>::removeTooLongPaths(shortened_paths);
    INFO("found " << shortened_paths.size() << " paths after shortening")
    INFO("used " << topological_prms[i]->cluster_seed_nodes_.size() << " cluster centers")


    // 移除重复的路径
    shortened_paths =
     TopologicalPRMClustering<Vector<3>>::removeEquivalentPaths(shortened_paths);

    for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
      // // INFO("shortening path bef " << pi)
      // // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())

      shortened_paths[pi] =
        TopologicalPRMClustering<Vector<3>>::shorten_path(shortened_paths[pi], false);
      shortened_paths[pi] =
        TopologicalPRMClustering<Vector<3>>::shorten_path(shortened_paths[pi]);

      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // // INFO("shortening path aft " << pi)
    }
    shortened_paths =
     TopologicalPRMClustering<Vector<3>>::removeEquivalentPaths(shortened_paths);
    
    
    // 对路径长度进行排序
    std::sort(shortened_paths.begin(), shortened_paths.end(),
              comparator_path_with_length<Vector<3>>);


    // 将缩短后的路径存储在paths_between_gates数组中
    paths_between_gates[i] = shortened_paths;

    
    // for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
    //   std::stringstream path_ss;
    //   // // INFO("shortened length " << shortened_paths[pi].length)
    //   path_ss << output_folder_ << "roadmap_shortened_unique_path" << i << "_"
    //           << pi << ".csv";
    //   // // INFO("juchuuuuu")
    //   TopologicalPRMClustering<Vector<3>>::savePath(path_ss.str(),
    //                                                 shortened_paths[pi].plan);
    //   // // INFO("saved");
    // }

    // INFO("ended loop")
  }
  // INFO("samples created");
  // exit(1);
  // INFO("return")

  return paths_between_gates;// 返回最终的几何路径集合
}


// 删除掉原有的路图文件(.csv文件)
template <class T> void TopologicalPRMClustering<T>::removeRoadmapFiles() {

  std::stringstream ss_roadmap;
  ss_roadmap << output_folder_ << "roadmap_";
  std::string ss_roadmap_str = ss_roadmap.str();
  std::cout << ss_roadmap_str << std::endl;

  for (const auto &entry :
       std::filesystem::directory_iterator(output_folder_)) {
    // std::cout << entry.path().string() << std::endl;
    const std::string path_name = entry.path().string();
    if (path_name.compare(0, ss_roadmap_str.length(), ss_roadmap_str) == 0) {
      std::cout << "removing " << path_name << std::endl;
      std::filesystem::remove(entry.path());
    }
  }
}
