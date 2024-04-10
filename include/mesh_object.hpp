/*
定义了一个MeshObject类，表示三维物体，
包含了一些成员变量和成员函数，用于表示和处理该物体的顶点、法向量、面以及位置等信息
 */

#pragma once

#include <cfloat>
#include <iostream>
#include <string>
#include <vector>

#include "RAPID.H"
#include "common.hpp"

//#define COLOR_ARRAY
//#define POSITION_IN_MEAN
#define OPTIMIZE_MEMORY_SIZE

//定义MeshObject类，表示三维物体
class MeshObject {
 public:
 // 构造函数，创建MeshObject对象
  MeshObject(std::string objectname = "");
  virtual ~MeshObject();
  int createRAPIDModel();   // 创建RAPID模型，用于快速检测碰撞
  void setPositionToMean(); // 将物体的位置设置为其所有顶点的平均值
  void setPositionToCenter();// 将物体的位置设置为其所有顶点的中心
  void printInfo();// 打印物体的信息，包括名称、顶点数、法向量数、面数等
  void printVerteces(); // 打印物体的顶点坐标
  void printRawVerteces(); // 打印物体的原始顶点坐标
  void addFace(int a, int b, int c);  // 添加面，指定顶点的索引
  void addFace(Face face);// 添加面，指定Face结构体
  void addVertex(double x, double y, double z);// 添加顶点坐标
  void addVertex(Point3D point); // 添加Point3D结构体
  void addNormal(double x, double y, double z);// 添加法向量坐标
  void addNormal(Vector3D normal);// 添加Vector3D结构体
  void dropOutOfXY(double xMin, double xMax, double yMin, double yMax); // 移除物体中位置在指定范围外的顶点
  Position3D getPosition(); // 获取物体的位置
  Position3D* getPositionPntr(); // 获取物体位置的指针
  void setPosition(Position3D pos); // 设置物体的位置
  RAPID_model* getRapidModel();// 获取RAPID模型
  Vector3D* getNormals(); // 获取法向量数组
  Point3D* getVertices(); // 获取顶点数组
  Face* getFaces(); // 获取面数组
#ifdef COLOR_ARRAY
  Color* getColors(); // 获取颜色数组
#endif
  void BeginObject();// 开始添加物体
  void EndObject();// 结束添加物体
  unsigned int getVerticesNum(); // 获取顶点数量
  unsigned int getFacesNum();// 获取面数量
  unsigned int getNormalsNum();
  std::string getName();
  void setMaterialName(std::string materialName);
  std::string getMaterialName();
  void setColor(RGBColor color);
  RGBColor getColor();
  static bool collide(MeshObject* object1, Position3D object1Position, // 判断两个物体是否发生碰撞
                      MeshObject* object2, Position3D object2Position);
  static bool collide(MeshObject* object1, MeshObject* object2);// 判断两个物体是否发生碰撞
  static bool collide(std::vector<MeshObject*>* obstacles, MeshObject* robot, // 判断机器人和障碍物是否发生碰撞
                      Position3D robotPosition);
  static MeshObject* copyObject(MeshObject* object1); // 复制一个MeshObject对象
  // void saveToMatFile(std::string filename, std::string varName);

  double getMinX();
  double getMinY();
  double getMinZ();

  double getMaxX();
  double getMaxY();
  double getMaxZ();

 private:
  Position3D position;
  RAPID_model* rapidModel;
  Point3D* vertices;
  Vector3D* normals;
  Face* faces;
#ifdef COLOR_ARRAY
  Color* colors;
#endif
  RGBColor color;
  int numVertices;
  int numVerticesAlloced;
  int numNormals;
  int numNormalsAlloced;
  int numFacesAlloced;
  int numFaces;
  bool objectStarted;
  bool objectEnded;
  std::string name;
  std::string materialName;
};
