/*
实现PRM算法的类，通过采样生成初始化的概率路图
✨代码分析
①  createInitialGraph函数实现了初始化PRM图
②  addUniformPoint函数实现了在PRM中添加均匀分布的随机点
③  setBorders函数用于设置自由空间的边界
④  calculateAddedPoints实现了添加新点和连接邻居点的功能
⑤  plan函数用于给出找到的路径
⑥  get_points函数返回处于自由空间的点
*/

/*
👇PRM基本介绍
PRM是一种基于采样点的路径规划方法，
通过在地图上随机采样一定数量的点，建立一个节点集合，
然后利用这些节点构建一个道路网络，
最终在这个网络中使用搜索算法找到一条连接起点和终点的路径。
*/
#pragma once

#include <algorithm>
#include <flann/flann.hpp>
#include <limits>
#include <memory>
#include <vector>

#include "common.hpp"
#include "dijkstra.hpp"
#include "esdf_map.hpp"
#include "tree_node.hpp"

#define OUTPUT_AFTER_ADDED_NUM_PRM 100

template<class T>
class PRM {
 public:
  // 构造函数，从YAML配置和地图中初始化PRM对象
  PRM(const YAML::Node& planner_config, std::shared_ptr<ESDFMap> map);
 
  // 创建初始图形
  void createInitialGraph(std::vector<HeapNode<T>*>& cities_nodes_,
                          int initial_size);
 
  // 添加均匀分布的点
  void addUniformPoints(int num_points_to_add);

  // bool add_point(Point3DOriented point);
  // 计算添加的点
  void calculateAddedPoints(bool add_points = true);
  std::vector<HeapNode<T>*>& get_points();// 获取所有点
  
  // 规划从起点到终点的路径
  path_with_length<T> plan(int start_index, int goal_index);
  
  // 规划从起点到多个终点的路径
  std::vector<path_with_length<T>> plan(int start_index,
                                        std::vector<int> goal_indexes);
  // 设置边界
  void setBorders(Vector<3> min_position, Vector<3> max_position);

 private:
  // FILL RANDUM STATE
  void fillRandomState(HeapNode<T>* positionToFill);// 填充随机状态
  void fillPoint(HeapNode<T>* newNode, T point);//填充点

  std::shared_ptr<ESDFMap> map;

  // TEST POINTS BETWEEN STATES
  // 测试两个节点之间是否有自由路径
  bool isPathFreeBetweenNodes(HeapNode<T>* actual, HeapNode<T>* neigbour);
  // MPNN::ANNpoint createANNpoint(MapPoint<PLANNER_STATE>* node);
  // MPNN::ANNpoint fillANNpoint(MPNN::ANNpoint aNNpoint,
  // MapPoint<PLANNER_STATE>* treeNode); FILL OR CREATE ANN POINT MPNN::ANNpoint
  // fillANNpoint(HeapNode<T> * heepNode, MPNN::ANNpoint aNNpoint = NULL);

  // test collision
  // 检查点是否在碰撞中
  bool isInCollision(T object_position);
  bool isInCollision(HeapNode<T>* node);

  flann::Index<flann::L2<float>>* flann_index;// flann索引

  Dijkstra<T> dijskra_; // 迪杰斯特拉算法对象

  std::vector<HeapNode<T>*> cities_nodes_;// 存储节点的向量

  std::vector<HeapNode<T>*> generated_free_positions_; // 存储自由位置的向量
  std::shared_ptr<ESDFMap> map_;// 地图指针
  double collision_distance_check_;// 碰撞距离检查
  double min_clearance_;// 最小通行空间

  bool continuous_point_adding_;// 是否持续添加点
  int continuous_point_adding_start_id_;// 持续添加点的起始ID

  Vector<3> min_position_, max_position_, position_range_;// 最小位置、最大位置和位置范围
};

// 构造函数实现
template<class T>
PRM<T>::PRM(const YAML::Node& planner_config, std::shared_ptr<ESDFMap> map) {
  // TODO Auto-generated constructor stub
  map_ = map;
  continuous_point_adding_ = false;
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");
  this->collision_distance_check_ =
    loadParam<double>(planner_config, "collision_distance_check");
  continuous_point_adding_start_id_ = 0;

  if (collision_distance_check_ == 0) {
    ERROR(
      "you need to specify collision_distance_check for sampling-based motion "
      "planning");
    exit(1);
  }
}



// 创建初始图形实现
// 创建初始的图形结构。
// 它首先将城市节点添加到生成的自由位置列表中，然后生成指定数量的随机位置，
// 并检查是否与碰撞物相交。
// 最后，使用FLANN库构建一个KD树索引来加快路径搜索速度。
template<class T>
void PRM<T>::createInitialGraph(std::vector<HeapNode<T>*>& cities_nodes_,
                                int initial_size) {
  INFO("generate graph PRM");

  // generated_number_of_neighbors = NUMBER_OF_NEAREST_NEIGBOURS + 1;
  this->cities_nodes_ = cities_nodes_;

  int generateIndex = 0;
  int addedCounter = 0;
  int collisionCounter = 0;
  int collisionCounterTot = 0;
  bool collisionDetected = true;

  // add also cities
  generated_free_positions_.reserve(initial_size + cities_nodes_.size());
  INFO("reserved");

  // try to connect goal， 尝试连接目标节点
  int citiesNumConnectionsAdded = 0;
  INFO("we have " << cities_nodes_.size() << "city nodes")
  for (int cityIndex = 0; cityIndex < cities_nodes_.size(); ++cityIndex) {
    // MPNN::ANNpoint newPoint = fillANNpoint(cities_nodes[cityIndex]);
    INFO("city: " << *cities_nodes_[cityIndex]);
    if (!isInCollision(cities_nodes_[cityIndex])) {
      // kdTree->AddPoint(newPoint, cities_nodes[cityIndex]);
      generated_free_positions_.push_back(cities_nodes_[cityIndex]);
    } else {
      ERROR("collision detected in city with index " << cityIndex
                                                     << "!!!! exiting....");
      exit(1);
    }
  }
  INFO("added " << generated_free_positions_.size() << " cities positions");


  double next_initial_sampling_info = 0.0;
  // generate NUM_GENERATE_POSITIONS_AT_ONCE points
  // 生成 NUM_GENERATE_POSITIONS_AT_ONCE 个点
  for (generateIndex = 0; generateIndex < initial_size; ++generateIndex) {
    collisionDetected = true;
    HeapNode<T>* newNode = new HeapNode<T>();
    // INFO("generated "<<generateIndex);
    newNode->node_id =
      generated_free_positions_.size();      // start and goal are 0 and 1
    newNode->cluster_id = newNode->node_id;  // start and goal are 0 and 1
    newNode->city_node = false;
    while (collisionDetected) {
      fillRandomState(newNode);
      if (!isInCollision(newNode)) {
        generated_free_positions_.push_back(newNode);
        collisionDetected = false;
        addedCounter++;
        // output every OUTPUT_AFTER_ADDED_NUM_PRM generated free positions
        // 每添加 OUTPUT_AFTER_ADDED_NUM_PRM 个生成的自由位置，输出一次信息
        if (((double)addedCounter) / ((double)initial_size) >=
            next_initial_sampling_info) {
          next_initial_sampling_info += 0.1;
          // INFO(	"collision states
          // "<<(100.0*((double)collisionCounter)/((double)
          // (collisionCounter+addedCounter)))<<"%");
          INFO("added " << addedCounter << " configurations out of "
                        << initial_size);
        }
      } else {
        // INFO("collision detected");
        collisionDetected = true;
        collisionCounter++;
        collisionCounterTot++;
      }
    }
  }

  INFO("testing flann");
  // 创建一个存储自由位置的矩阵
  flann::Matrix<float> new_points_matrix(
    new float[generated_free_positions_.size() * 3],
    generated_free_positions_.size(), 3);
  for (int var = 0; var < generated_free_positions_.size(); ++var) {
    new_points_matrix[var][0] = generated_free_positions_[var]->data.x;
    new_points_matrix[var][1] = generated_free_positions_[var]->data.y;
    new_points_matrix[var][2] = generated_free_positions_[var]->data.z;
  }
  // 构建FLANN索引
  flann_index = new flann::Index<flann::L2<float>>(new_points_matrix,
                                                   flann::KDTreeIndexParams(4));
  flann_index->buildIndex();


  // int nn = generated_number_of_neighbors;

  // float sd = 2; //configuration space dimension
  // float num_points = generatedFreePositions.size();


  INFO("generated " << initial_size << " random positions");
  INFO("collisionCounterTot " << collisionCounterTot << " positions");
  continuous_point_adding_start_id_ = 0;
  this->continuous_point_adding_ = true;
  calculateAddedPoints(false);
  INFO("end create_initial_graph");
}


// 用于添加均匀分布的点到生成的自由位置列表中。
// 它也会生成指定数量的随机位置，并检查是否与碰撞物相交。
template<class T>
void PRM<T>::addUniformPoints(int num_points_to_add) {
  // 如果不是连续添加点状态，则设置为连续添加点状态，并记录当前自由位置的起始ID
  if (!this->continuous_point_adding_) {
    this->continuous_point_adding_ = true;
    this->continuous_point_adding_start_id_ = generated_free_positions_.size();
  }
  int generateIndex = 0;
  int addedCounter = 0;
  int collisionCounter = 0;
  int collisionCounterTot = 0;
  bool collisionDetected = true;
  // 循环生成指定数量的均匀分布的随机点
  for (generateIndex = 0; generateIndex < num_points_to_add; ++generateIndex) {
    collisionDetected = true;
    HeapNode<T>* newNode = new HeapNode<T>();
    // INFO("generated "<<generateIndex);
    newNode->node_id =
      generated_free_positions_.size();      // start and goal are 0 and 1
    newNode->cluster_id = newNode->node_id;  // start and goal are 0 and 1
    newNode->city_node = false;// 节点ID为当前自由位置数量，簇ID与节点ID相同
    
    // 检查新生成的节点是否与其他节点发生碰撞，直到没有碰撞为止
    while (collisionDetected) {
      fillRandomState(newNode);// 生成随机状态
      if (!isInCollision(newNode)) {// 如果没有发生碰撞
        generated_free_positions_.push_back(newNode);// 将节点添加到自由位置集合中
        collisionDetected = false;
        addedCounter++;

      } else {
        // INFO("collision detected");
        collisionDetected = true;// 发生了碰撞
        collisionCounter++;
        collisionCounterTot++;
      }
    }
  }
}
// 上面addUniformPoints()函数实现了在PRM图中添加均匀分布的随机点的功能。
// 首先检查是否处于连续添加点的状态，如果不是，则将状态设置为连续添加点，并记录当前自由位置的起始ID。
// 然后循环生成指定数量的随机点，对于每个随机点，检查是否与其他节点发生碰撞，直到没有碰撞为止，
// 然后将节点添加到自由位置集合中。

/*
template<class T>
bool PRM<T>::add_point(Point3DOriented point) {
  if (!this->continuous_point_adding) {
    this->continuous_point_adding = true;
    this->continuous_point_adding_start_id = generatedFreePositions.size();
  }
  bool point_added = false;
  HeapNode<T>* newNode = new HeapNode<T>();
  newNode->node_id =
    generatedFreePositions.size();  // start and goal are 0 and 1
  newNode->cluster_id = generatedFreePositions.size();
  newNode->city_node = false;
  fillPoint(newNode, point);

  // INFO("test collision");
  if (!testCollision(this->obstacles, this->robot, newNode)) {
    // MPNN::ANNpoint newPoint = fillANNpoint(newNode);

    // kdTree->AddPoint(newPoint, newNode);
    generatedFreePositions.push_back(newNode);
    // INFO("new point created, points size "<<generatedFreePositions.size()<<"
    // flann_index.size() " <<flann_index->size()<<"
    // continuous_point_adding_start_id "<<continuous_point_adding_start_id);
    point_added = true;
  } else {
    delete newNode;
  }
  return point_added;
}
*/

//PRM类模板的成员函数setBorders，用于设置自由空间的边界
template<class T>
void PRM<T>::setBorders(Vector<3> min_position, Vector<3> max_position) {
  min_position_ = min_position;// 设置自由空间的最小位置边界
  max_position_ = max_position;// 设置自由空间的最大位置边界
  position_range_ = max_position - min_position;// 计算自由空间的位置范围
}

// 实现了PRM算法中添加新点和连接邻居点的功能
// continuous_point_adding_：连续添加点标志，如果为真，则执行添加和连接操作。
// 1、提取从continuous_point_adding_start_id_开始的所有新点。
// 2、创建一个flann矩阵表示新点的坐标，并将其添加到flann索引中。
// 3、计算配置空间维度和生成的自由位置数量，并根据公式计算最近邻居的数量k。
// 4、使用knnSearch获取每个新点的最近邻点。
// 5、连接每个新点与其最近邻居点，并检查连接是否存在自由路径。
// 6、将连接添加到可见性节点的集合中。
// 7、更新相关计数器和标志。
// 8、最后更新continuous_point_adding_start_id_和continuous_point_adding_的值。
template<class T>
void PRM<T>::calculateAddedPoints(bool add_points) {
  if (continuous_point_adding_) {// 如果连续添加点标志为真
    // INFO("calculate_added_points begin");

    // INFO("new_points bef");
    // 提取从continuous_point_adding_start_id_开始的所有新点
    std::vector<HeapNode<T>*> new_points(
      generated_free_positions_.begin() + continuous_point_adding_start_id_,
      generated_free_positions_.begin() + generated_free_positions_.size());
    // INFO("new_points af");

    // connect new points to the map，即将新点连接到地图上
    flann::Matrix<float> new_points_matrix;
    // INFO("want to add " <<new_points.size() <<" points to flann index");

    // 创建新点的flann矩阵
    new_points_matrix = flann::Matrix<float>(new float[new_points.size() * 3],
                                             new_points.size(), 3);
    for (int var = 0; var < new_points.size(); ++var) {
      new_points_matrix[var][0] = new_points[var]->data.x;
      new_points_matrix[var][1] = new_points[var]->data.y;
      new_points_matrix[var][2] = new_points[var]->data.z;
      // INFO(*new_points[var]);
    }
    if (add_points) {
      flann_index->addPoints(new_points_matrix,
                             2.0);  // 在初始化时不添加点
    }

    // INFO("added points");

    float sd = 3;  // configuration space dimension，配置空间的维度
    float num_points = generated_free_positions_.size();

    int k = M_E * (1 + 1 / sd) * log10(num_points);
    // k += num_headings;
    int k_search = k;

    // INFO_VAR(k);
    // INFO_VAR(k_search);
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<int>> indices_rew;
    std::vector<std::vector<float>> dists;
    // flann::Matrix<int> indices(new int[new_points_matrix.rows * k_search],
    // new_points_matrix.rows, k_search); flann::Matrix<float> dists(new
    // float[new_points_matrix.rows * k_search], new_points_matrix.rows,
    // k_search);

    // 使用knnSearch获取每个新点的最近邻点
    flann_index->knnSearch(new_points_matrix, indices, dists, k_search,
                           flann::SearchParams(128));

    indices_rew = indices;
    int numConnectionsAdded = 0;
    int numConnectionsAlreadyAdded = 0;
    int addedCounter = 0;
    int max_nn_used = 0;
    // INFO("add connections begin with k="<<k);
    // connect all points to generated_number_of_neighbors positions
    // 连接所有点到生成的邻居位置
    for (int generateIndex = continuous_point_adding_start_id_;
         generateIndex < generated_free_positions_.size(); ++generateIndex) {
      HeapNode<T>* actual = generated_free_positions_[generateIndex];

      int connection_per_target = 0;

      for (int neigbourIndex = 0; neigbourIndex < k; ++neigbourIndex) {
        int nnindex = indices[generateIndex - continuous_point_adding_start_id_]
                             [neigbourIndex];
        if (generated_free_positions_[nnindex]->cluster_id ==
            actual->cluster_id) {
          // 如果最近邻点与当前点属于同一集群，则跳过连接
          // INFO("skip same id gi "<<generateIndex<<" ni "<<neigbourIndex<<"
          // for cluster id "<< actual->cluster_id)
          continue;
        }

        HeapNode<T>* neigbour = generated_free_positions_[nnindex];

        // 检查从邻居到当前点是否存在自由路径
        bool freePath_act_neigh = map_->isSimplePathFreeBetweenNodes(neigbour, actual, min_clearance_, collision_distance_check_);
        if (freePath_act_neigh) {
          double distance_act_neigh = actual->data.distance(neigbour->data);
          // INFO("distance_act_neigh "<<distance_act_neigh<<"
          // dists[generateIndex -
          // continuous_point_adding_start_id][neigbourIndex]
          // "<<sqrt(dists[generateIndex -
          // continuous_point_adding_start_id][neigbourIndex]))

         // 将连接添加到可见性节点的集合中
          auto connectionAdded1 =
            actual->visibility_node_ids.insert(std::pair<HeapNode<T>*, double>(
              neigbour, distance_act_neigh));  // addConnection(neigbour,
                                               // bestDist[neigbourIndex]);

          if (connectionAdded1.second) {
            numConnectionsAdded++;
            addedCounter++;
            connection_per_target += 1;
          } else {
            numConnectionsAlreadyAdded++;
          }
        }
      }

      for (int neigbourIndex = 0; neigbourIndex < k; ++neigbourIndex) {
        int nnindex =
          indices_rew[generateIndex - continuous_point_adding_start_id_]
                     [neigbourIndex];
        if (generated_free_positions_[nnindex]->cluster_id ==
            actual->cluster_id) {
          // 如果最近邻点与当前点属于同一集群，则跳过连接
          // INFO("skip same id gi "<<generateIndex<<" ni "<<neigbourIndex<<"
          // for cluster id "<< actual->cluster_id)
          continue;
        }

        HeapNode<T>* neigbour = generated_free_positions_[nnindex];
        // 检查从当前点到邻居是否存在自由路径
        bool freePath_neigh_act = map_->isSimplePathFreeBetweenNodes(neigbour, actual, min_clearance_, collision_distance_check_);
        if (freePath_neigh_act) {
          double distance_neigh_act = neigbour->data.distance(actual->data);
          // 将连接添加到可见性节点的集合中
          auto connectionAdded2 = neigbour->visibility_node_ids.insert(
            std::pair<HeapNode<T>*, double>(actual, distance_neigh_act));
          if (connectionAdded2.second) {
            numConnectionsAdded++;
            addedCounter++;
            connection_per_target += 1;
          } else {
            numConnectionsAlreadyAdded++;
          }
        }
      }
      // INFO("snode "<<actual->node_id<<" gen index "<<generateIndex<<" c added
      // "<<connection_per_target<< " k is "<<k)
    }

    // INFO_RED("max_nn_used "<<max_nn_used);
    this->continuous_point_adding_start_id_ = generated_free_positions_.size();
    this->continuous_point_adding_ = false;
    // INFO("calculate_added_points end");
  } else {
    INFO("no new points");
  }
}


template<class T>
path_with_length<T> PRM<T>::plan(int start_index, int goal_index) {
  // INFO("begin PRM::plan()");

  // INFO("dijskra.findPath ");
  std::vector<int> goal_indexes;// 创建存储目标点索引的向量
  goal_indexes.push_back(goal_index);
  // 调用dijskra_对象的findPath函数，以获取从起点到目标点的路径列表
  std::vector<path_with_length<T>> found_paths =
    dijskra_.findPath(start_index, goal_indexes, generated_free_positions_);

  // INFO("plan length "<<found_paths[0].length);
  // INFO("end PRM::plan()");

  return found_paths[0];// 返回找到的路径列表中的第一个路径
}

template<class T>
std::vector<path_with_length<T>> PRM<T>::plan(int start_index,
                                              std::vector<int> goal_indexes) {
  // INFO("begin PRM::plan()");
  // 调用dijskra_对象的findPath函数，以获取从起点到目标点的路径列表
  std::vector<path_with_length<T>> found_paths =
    dijskra_.findPath(start_index, goal_indexes, generated_free_positions_);

  // INFO("end PRM::plan()");
  return found_paths; // 返回找到的路径列表
}

template<class T>
std::vector<HeapNode<T>*>& PRM<T>::get_points() {
  return this->generated_free_positions_;// 返回生成的自由位置向量的引用
}
