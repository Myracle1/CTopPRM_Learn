/*
实现了一个基础地图类 BaseMap，可以加载地图数据文件，通过getClearence函数计算指定位置的间隙距离
✨代码分析
①  isInCollision()判断给定位置是否存在碰撞
②  isSimplePathFreeBetweenNodes() 判断两个节点之间是否存在自由路径
③  isPathCollisionFree()判断给定路径是否不存在碰撞
④  isDeformablePath()判断两条路径之间是否存在可变形路径
⑤  isDeformablePathBetween() 判断两个节点之间是否存在可变形路径
⑥  samplePath()对给定路径进行采样，生成指定数量的采样点
*/
#pragma once

#include "cnpy.h" // 引入 cnpy 库，用于读取和解析 .npy 文件
#include "common.hpp" // 引入 common.hpp 头文件
#include "dijkstra.hpp" // 引入 dijkstra.hpp 头文件
// #include "heap.hpp" // 引入 heap.hpp 头文件（已注释掉）

#define PRECISION_BASE_MAP (10e-6) // 定义基本映射的精度为 10e-6

class BaseMap {
public:
  BaseMap(){}; // 默认构造函数

  virtual void load(std::string filename){}; // 加载地图数据文件的虚函数（子类需要实现该函数）
  
  virtual double getClearence(const Vector<3> pos) = 0; // 获取指定位置的间隙距离的纯虚函数

  virtual Vector<3> getMinPos() { return Vector<3>::Zero(); }; // 获取地图范围最小位置的虚函数
  virtual Vector<3> getMaxPos() { return Vector<3>::Zero(); }; // 获取地图范围最大位置的虚函数
  //   Vector<3> getMinPos() { return centroid - extents / 2; }
  //   Vector<3> getMaxPos() { return centroid + extents / 2; }

  virtual std::pair<Vector<3>, Vector<3>>
  gradientInVoxelCenter(const Vector<3> pos) {
    return {Vector<3>::Zero(), Vector<3>::Zero()};
  };
  // 在体素中心计算梯度的虚函数

  static cnpy::NpyArray getArray(FILE *fp) {
    std::vector<size_t> shape;
    size_t word_size;
    bool fortran_order;
    cnpy::parse_npy_header(fp, word_size, shape, fortran_order); // 解析 .npy 文件头部信息

    cnpy::NpyArray arr(shape, word_size, fortran_order);
    size_t nread = fread(arr.data<char>(), 1, arr.num_bytes(), fp); // 从文件中读取数据到数组
    if (nread != arr.num_bytes()) {
      // INFO("badly read array");
      exit(1);
    }
    // INFO("get_array with " << arr.num_bytes() << "bytes end with shape")
    for (size_t i = 0; i < arr.shape.size(); i++) {
      // INFO(i << " shape " << arr.shape[i])
    }
    return arr;
  }
  // 从文件中读取数据并返回数组对象的静态函数

  void fillRandomState(HeapNode<Vector<3>> *positionToFill,
                       Vector<3> min_position, Vector<3> position_range);
  // 填充随机状态的函数，用于生成随机位置

  void
  fillRandomStateInEllipse(HeapNode<Vector<3>> *positionToFill,
                           HeapNode<Vector<3>> *start,
                           HeapNode<Vector<3>> *goal,
                           const double ellipse_ratio_major_axis_focal_length,
                           bool planar);
  // 在椭圆内填充随机状态的函数，用于生成椭圆内的随机位置

  // bool isInCollision(Vector<3> object_position);
  bool isInCollision(Vector<3> object_position, const double clearance);
  // 判断给定位置是否存在碰撞的函数

  // std::pair<bool, Vector<3>> isSimplePathFreeBetweenNodes(Vector<3> actual,
  //                                                         Vector<3>
  //                                                         neigbour);
  std::pair<bool, Vector<3>>
  isSimplePathFreeBetweenNodes(Vector<3> actual, Vector<3> neigbour,
                               const double clearance,
                               const double collision_distance_check);
  // 判断两个节点之间是否存在自由路径的函数

  std::pair<bool, Vector<3>>
  isPathCollisionFree(path_with_length<Vector<3>> path, const double clearance,
                      const double collision_distance_check);
  // 判断给定路径是否不存在碰撞的函数

  bool
  isSimplePathFreeBetweenGuards(Vector<3> actual, Vector<3> neigbour,
                                      const double clearance,
                                      const double collision_distance_check);
  // 判断两个守卫之间是否存在自由路径的函数

  // bool isSimplePathFreeBetweenNodes(HeapNode<Vector<3>> *actual,
  //                                   HeapNode<Vector<3>> *neigbour);

  bool isDeformablePath(std::vector<HeapNode<Vector<3>> *> path1,
                        const double length_tot1,
                        std::vector<HeapNode<Vector<3>> *> path2,
                        const double length_tot2,
                        const double num_collision_check,
                        const double clearance,
                        const double collision_distance_check);
  // 判断两条路径之间是否存在可变形路径的函数

  bool isDeformablePathBetween(HeapNode<Vector<3>> *start,
                               HeapNode<Vector<3>> *end,
                               HeapNode<Vector<3>> *between1,
                               HeapNode<Vector<3>> *between2,
                               const double clearance,
                               const double collision_distance_check);
  // 判断两个节点之间是否存在可变形路径的函数

  std::vector<Vector<3>> samplePath(std::vector<HeapNode<Vector<3>> *> path,
                                    const double length_tot,
                                    const double num_samples);
  // 对给定路径进行采样的函数，生成指定数量的采样点
};
