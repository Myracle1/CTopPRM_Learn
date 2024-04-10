/*
定义了一个PCMap类，其中包含了加载点云地图数据的功能。
✨代码分析
①  load函数用于从指定文件中加载地图数据。
②  fill_data函数是一个模板函数，用于将一维数组中的数据填充到二维数组中，在地图中使用。
③  map_data存储了地图数据的二维数组。
④  max_point_axis和min_point_axis分别表示地图中最大和最小点的坐标。
*/

#pragma once

#include "base_map.hpp"// 包含基础地图类的头文件
#include "common.hpp"// 包含通用函数和数据类型的头文件

class PCMap {
 public:
  PCMap();// 默认构造函数
  void load(std::string filename); // 加载地图数据的函数
  // double getClearence(const Vector<3> pos);

  // std::pair<Vector<3>, Vector<3>> gradientInVoxelCenter(const Vector<3> pos);
  // Vector<3> getMinPos();
  // Vector<3> getMaxPos();

 private:
  template<typename T>
  void fill_data(T* data, cnpy::NpyArray& voxels_arr); // 填充地图数据的模板函数
  std::vector<std::vector<float>> map_data;// 存储地图数据的二维数组
  Vector<3> max_point_axis;// 地图中最大点的坐标
  Vector<3> min_point_axis;
};


template<typename T>
void PCMap::fill_data(T* data, cnpy::NpyArray& voxels_arr) {
  map_data.resize(voxels_arr.shape[0]);// 调整地图数据的行数
  for (size_t xi = 0; xi < voxels_arr.shape[0]; xi++) {
    map_data[xi].resize(voxels_arr.shape[1]);// 调整地图数据的列数
    for (size_t yi = 0; yi < voxels_arr.shape[1]; yi++) {
      map_data[xi][yi] = data[xi * voxels_arr.shape[0] + yi];// 填充地图数据
    }
  }
};