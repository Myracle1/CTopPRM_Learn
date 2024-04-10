/*
表示一个ESDF（Euclidean Signed Distance Field）地图。包含了一些成员函数和成员变量，用于读取、处理和查询地图数据。
✨代码分析
①  ESDFMap()：构造函数，用于创建ESDFMap对象。
②  load(std::string filename)：从文件中读取地图数据，并初始化内部的map_data数组、centroid和extents等成员变量。
③  getClearence(const Vector<3> pos)：获取指定点的碰撞距离（即到障碍物的最短距离）。
④  getMinPos()：获取地图的最小坐标。
⑤  getMaxPos()：获取地图的最大坐标。
⑥  gradientInVoxelCenter(const Vector<3> pos)：计算在体素中心处的梯度向量。
⑦  getRealPosFromIndex(Vector<3> pos_ijk)：从索引坐标计算出真实世界坐标。
⑧  interpolateMapData(Vector<3> pos_ijk)：插值计算地图数据。
*/

#pragma once

#include <string>
#include <vector>

#include "base_map.hpp"
#include "cnpy.h"
#include "common.hpp"

// ESDFMap是基于BaseMap的子类
class ESDFMap : public BaseMap {
 public:
  ESDFMap();// 构造函数
  void load(std::string filename);// 从文件中读取地图数据
  double getClearence(const Vector<3> pos);// 获取指定点的碰撞距离
  // cnpy::NpyArray getArray(FILE* fp);
  Vector<3> getMinPos();// 获取地图的最小坐标
  Vector<3> getMaxPos();// 获取地图的最大坐标

  // 计算在体素中心处的梯度向量
  std::pair<Vector<3>, Vector<3>> gradientInVoxelCenter(const Vector<3> pos);

  // 从索引坐标计算出真实世界坐标
  Vector<3> getRealPosFromIndex(Vector<3> pos_ijk);

 // 插值计算地图数据
  double interpolateMapData(Vector<3> pos_ijk);

 private:
  std::vector<std::vector<std::vector<float>>> map_data;// 存储地图数据
  Vector<3> centroid;// 地图质心坐标
  Vector<3> extents;// 地图尺寸
  double num_vexels_per_axis;// 每个轴上的体素数量
  double max_extents_;// 最大尺寸

 // 填充地图数据
  template<typename T>
  void fill_data(T* data, cnpy::NpyArray& voxels_arr);
};

//将一个一维数组中的数据按照指定的维度填充到一个三维数组中
template<typename T>
void ESDFMap::fill_data(T* data, cnpy::NpyArray& voxels_arr) {
  for (size_t xi = 0; xi < voxels_arr.shape[0]; xi++) {
    for (size_t yi = 0; yi < voxels_arr.shape[1]; yi++) {
      for (size_t zi = 0; zi < voxels_arr.shape[2]; zi++) {
        map_data[xi][yi][zi] =
          data[xi * voxels_arr.shape[0] * voxels_arr.shape[1] +
               yi * voxels_arr.shape[1] + zi];
      }
    }
  }
};