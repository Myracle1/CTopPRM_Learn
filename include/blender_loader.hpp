/*
BlenderLoader类的头文件，用于加载和保存Blender软件生成的3D模型文件，提供了静态函数来加载和保存模型
✨代码分析 
①  loadObjectsFromFile：从文件中加载物体并返回一个包含MeshObject指针的向量。
②  saveObjectsToFile：将MeshObject保存到文件中。
③  loadMaterialForObjectFromFile：从文件中加载材质信息，为每个MeshObject分配材质。
*/

#pragma once
#include <fstream>
#include <iostream>
#include <vector>
//#include <glm/glm.hpp>
//#include <GL/glu.h>
//#include "CLog.h"
#include <sstream>
#include <string>

#include "mesh_object.hpp"

#define FILE_HEADER                                                  
  "# Blender v2.72 (sub 0) OBJ File: 'goalConfigurationCubes.blend'" 
    << std::endl                                                     
    << "# www.blender.org" << std::endl

class BlenderLoader {
 public:
  BlenderLoader();
  virtual ~BlenderLoader();

  // 从文件中加载物体并返回一个包含MeshObject指针的向量
  static std::vector<MeshObject*> loadObjectsFromFile(std::string filename);

  // 将MeshObject保存到文件中
  static void saveObjectsToFile(const char* filename, std::vector<MeshObject*>,
                                bool addObjectPosition = true);

 private:
  // 从文件中加载材质信息并为每个MeshObject分配材质
  static void loadMaterialForObjectFromFile(
    std::vector<MeshObject*> meshObjects, std::string matLibFilename);
};
