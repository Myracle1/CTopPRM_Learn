/*
主要包含了一些常用的功能和数据结构定义
✨代码分析
①  日志记录功能：使用了log4cxx库来记录日志，包括不同颜色的信息输出，以及根据条件输出不同颜色的信息。
②  数学相关函数和数据结构：定义了一些常用的数学函数，如随机数生成、角度规范化、向量夹角计算等，并且包括了3D对象的定义和相关操作，比如点、面、矩阵、旋转等。
③  文件系统处理：定义了一些文件路径操作相关的宏和函数。
④  配置加载功能：提供了从YAML配置文件中加载参数的函数，以及解析数组参数的函数。
⑤  类型定义：定义了一些常用的类型别名，如矩阵、向量、四元数等。
⑥  TCLAP：在TCLAP命令行参数解析库的基础上对一些自定义类型进行了适配，使得可以从命令行参数中加载自定义类型的值。
 */

#pragma once

#include <math.h>
#include <tclap/CmdLine.h>
#include <yaml-cpp/yaml.h>

#include <cstring>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include "log4cxx/basicconfigurator.h"
#include "log4cxx/helpers/exception.h"
#include "log4cxx/logger.h"
#include "log4cxx/logmanager.h"
#include "log4cxx/propertyconfigurator.h"

/**********************************************************/
/*     logging begin           						      */

// 引入log4cxx库的LoggerPtr类型
extern log4cxx::LoggerPtr logger;

// 初始化日志系统，通过配置文件初始化logger对象
void startLogger(const std::string &name);

// 指定logger名称和配置文件名称，初始化logger对象
void startLogger(const char *loggerName, const char *configName);

// 获取指定名称的logger对象
log4cxx::LoggerPtr getLogger(const char *loggerName);

// 销毁所有logger对象
void quitLogger(void);

// 输出不同颜色的信息宏定义
#define INFO(s) LOG4CXX_INFO(logger, s);
#define ERROR(s) LOG4CXX_ERROR(logger, s);
#define OUTPUT_DEFAULT "\033[0m"
#define OUTPUT_BLACK "\033[30m"
#define OUTPUT_RED "\033[31m"
#define OUTPUT_GREEN "\033[32m"
#define OUTPUT_YELLOW "\033[33m"
#define OUTPUT_BLUE "\033[34m"
#define OUTPUT_MAGENTA "\033[35m"
#define OUTPUT_CYAN "\033[36m"
#define OUTPUT_WHITE "\033[37m"

// 输出红色的ERROR信息
#define ERROR_RED(x) ERROR(OUTPUT_RED << x << OUTPUT_DEFAULT)

// 输出红色的INFO信息
#define INFO_RED(x) INFO(OUTPUT_RED << x << OUTPUT_DEFAULT)

// 输出黄色的INFO信息
#define INFO_YELLOW(x) INFO(OUTPUT_YELLOW << x << OUTPUT_DEFAULT)

// 输出洋红色的INFO信息
#define INFO_MAGENTA(x) INFO(OUTPUT_MAGENTA << x << OUTPUT_DEFAULT)

// 输出青色的INFO信息
#define INFO_CYAN(x) INFO(OUTPUT_CYAN << x << OUTPUT_DEFAULT)

// 输出绿色的INFO信息
#define INFO_GREEN(x) INFO(OUTPUT_GREEN << x << OUTPUT_DEFAULT)

// 输出白色的INFO信息
#define INFO_WHITE(x) INFO(OUTPUT_WHITE << x << OUTPUT_DEFAULT)

// 输出蓝色的INFO信息
#define INFO_BLUE(x) INFO(OUTPUT_BLUE << x << OUTPUT_DEFAULT)

// 输出黑色的INFO信息
#define INFO_BLACK(x) INFO(OUTPUT_BLACK << x << OUTPUT_DEFAULT)

// 根据条件输出INFO信息
#define INFO_COND(cond, x) \
  if (cond) {              \
    INFO(x);               \
  }

// 根据条件输出指定颜色的INFO信息
#define INFO_COND_COLOR(cond, color, x) \
  if (cond) {                           \
    INFO(color << x << OUTPUT_DEFAULT); \
  }

// 输出变量名和变量值
#define VARIABLE_STR(s) #s
#define STR(s) VARIABLE_STR(s)
#define INFO_VAR(x) INFO(STR(x) << " = " << x)

// 打印容器vector或unordered_set元素
#define VALUE_PRINT_DELIMITER (", ")

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "[";
  for (int i = 0; i < v.size(); ++i) {
    os << v[i];
    if (i != v.size() - 1) os << ", ";
  }
  os << "]";
  return os;
}

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::unordered_set<T> &v) {
  os << "{";

  for (auto it = v.begin(); it != v.end(); ++it) {
    if (it != v.begin()) os << ", ";
    os << *it;
  }
  os << "}";
  return os;
}


/*     logging end           						      */
/**********************************************************/

/**********************************************************/
/*     some math stuff begin						      */
/**********************************************************/

// 引入默认的随机数生成器
extern std::default_random_engine random_generator;

// 定义常量
#define M_2PI (2 * M_PI)
#define POW(x) ((x) * (x))
#define MIN(x, y) ((x > y) ? y : x)
#define MAX(x, y) ((x > y) ? (x) : (y))
#define ABS(x) ((x < 0) ? (-(x)) : (x))
#define ANGLE_MIN (0)
#define ANGLE_MAX (M_2PI)

// 随机数种子初始化函数
void seed();

// 生成指定范围内的随机浮点数
double randDoubleMinMax(double min, double max);

// 生成指定范围内的随机整数
int randIntMinMax(int min, int max);

// 规范化角度值到指定范围内
double normalizeAngle(double angle, double min, double max);

// 生成指定范围内的随机浮点数（等概率分布）
double rand_real_uniform(double min, double max);

// 返回参数的符号
int sgn(int val);
float sgn(float val);
double sgn(double val);

// 生成指定范围内的步长为step的一系列浮点数
std::vector<double> range(double min, double max, double step);

// 将值限制在给定范围内
template<typename T>
T clip(const T &n, const T &lower, const T &upper) {
  return std::max(lower, std::min(n, upper));
}

// 计算两个向量之间的夹角
double angleBetween(Eigen::VectorXd a, Eigen::VectorXd b);

// 在指定椭球体内生成随机点，A和B是对应坐标轴的半径，two_major_length是椭球体的两个主半轴长度之和
Eigen::VectorXd random_ellipsoid_point(Eigen::VectorXd A, Eigen::VectorXd B,
                                       double two_major_length);

// 在指定球体内生成随机点，C是球心，radius是半径
Eigen::VectorXd random_ball_point(Eigen::VectorXd C, double radius);

// 计算指定维度的球体体积
double ball_volume(double radius, double dim_double);

// 计算指定维度的椭球体体积，two_major_length是椭球体的两个主半轴长度之和，two_focal_length是椭球体的两个焦距长度之和
double ellipse_volume(double two_major_length, double two_focal_length,
                      double dim_double);


/*     some math stuff end						          */
/**********************************************************/

/*********************************************************/
/*				 for 3d objects start					 */

typedef struct Face {
  unsigned int id1; // 第一个顶点的索引
  unsigned int id2; // 第二个顶点的索引
  unsigned int id3; // 第三个顶点的索引
} Face;

typedef struct Point2D {
  double x; // x坐标
  double y; // y坐标
  static double dim; // 维度（在这里固定为2）

  Point2D() { // 默认构造函数，将x和y初始化为NaN
    this->x = NAN;
    this->y = NAN;
  }

  Point2D(double x_, double y_) { // 构造函数，接受x和y坐标作为参数
    this->x = x_;
    this->y = y_;
  }

  std::vector<double> toVector() { // 将点转换为向量表示
    std::vector<double> vector;
    vector.push_back(x);
    vector.push_back(y);
    return vector;
  }

  double distance(Point2D other) { // 计算与另一个点之间的欧氏距离
    double diffx = this->x - other.x;
    double diffy = this->y - other.y;
    return sqrt(diffx * diffx + diffy * diffy);
  }

  Point2D getStateInDistance(Point2D to, double in_distance) { // 返回到另一个点指定距离处的点坐标
    double mutual_distance = this->distance(to);
    double x_ = this->x + (to.x - this->x) * (in_distance / mutual_distance);
    double y_ = this->y + (to.y - this->y) * (in_distance / mutual_distance);
    return Point2D(x_, y_);
  }

  std::string toString() { // 将点转换为字符串表示
    std::stringstream ss;
    ss << this->x << VALUE_PRINT_DELIMITER << this->y;
    return ss.str();
  }
} Point2D;

typedef struct Point3D {
  double x; // x坐标
  double y; // y坐标
  double z; // z坐标
  static double dim; // 维度（在这里固定为3）

  Point3D() : x(NAN), y(NAN), z(NAN) {} // 默认构造函数，将x、y和z初始化为NaN

  Point3D(const double &_x, const double &_y, const double &_z) // 构造函数，接受x、y和z坐标作为参数
    : x(_x), y(_y), z(_z) {}

  Point3D operator+(const Point3D &p) const { // 重载+运算符，实现两个点的向量相加
    return Point3D(x + p.x, y + p.y, z + p.z);
  }
  Point3D operator-(const Point3D &p) const { // 重载-运算符，实现两个点的向量相减
    return Point3D(x - p.x, y - p.y, z - p.z);
  }
  Point3D operator*(double c) const { // 重载*运算符，实现点与标量的乘法
    return Point3D(c * x, c * y, c * z);
  }
  Point3D operator/(double c) const { // 重载/运算符，实现点与标量的除法
    return Point3D(x / c, y / c, z / c);
  }
  bool operator==(const Point3D &p) const { // 重载==运算符，判断两个点是否相等
    return ((p.x == this->x) && (p.y == this->y) && (p.z == this->z));
  }

  std::vector<double> toVector() { // 将点转换为向量表示
    std::vector<double> vec(3);
    vec[0] = this->x;
    vec[1] = this->y;
    vec[2] = this->z;
    return vec;
  }


  // 定义一个Point3D结构体，表示三维空间中的点
typedef struct Point3D {
  double x; // x坐标
  double y; // y坐标
  double z; // z坐标

  // 默认构造函数，将x、y和z初始化为0.0
  Point3D() : x(0.0), y(0.0), z(0.0), yaw(0.0), pitch(0.0), roll(0.0) {}

  // 带参数的构造函数，接受x、y、z坐标以及yaw、pitch、roll角度作为参数
  Point3D(double x, double y, double z, double yaw, double pitch, double roll) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
  }

  // 设置平移变换
  void setTranslation(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }

  // 设置旋转变换
  void setRotation(double yaw, double pitch, double roll) {
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
  }
} Position3D; // 定义Position3D别名作为结构体的名称

// RotationMatrix结构体，表示旋转矩阵
typedef struct RotationMatrix {
  double M[3][3]; // 3x3的旋转矩阵
} RotationMatrix;

// TransformationMatrix结构体，表示变换矩阵
typedef struct TransformationMatrix {
  double M[4][4]; // 4x4的变换矩阵
} TransformationMatrix;

// 定义一个Point3D类型的别名Vector3D
typedef struct Point3D Vector3D;

// LoadVector结构模板，用于加载一维向量
template<typename T>
struct LoadVector {
  std::vector<T> vector; // 存储向量的容器

  // 重载=运算符，用于从字符串加载一维向量
  LoadVector &operator=(const std::string &str) {
    std::istringstream iss(str);
    T val;
    while (iss >> val) {
      if (iss.fail()) {
        throw TCLAP::ArgParseException(str + " is not a vector");
      }
      vector.push_back(val);
    }
    return *this;
  }
};

// LoadVector2D结构模板，用于加载二维向量
template<typename T, int dim>
struct LoadVector2D {
  std::vector<std::vector<T>> vector; // 存储二维向量的容器

  // 重载=运算符，用于从字符串加载二维向量
  LoadVector2D &operator=(const std::string &str) {
    std::istringstream iss(str);
    T val;
    int val_count = dim;
    while (iss >> val) {
      if (iss.fail()) {
        throw TCLAP::ArgParseException(str + " is not a vector");
      }
      if (val_count == dim) {
        vector.push_back(std::vector<T>());
        val_count = 0;
      }
      vector.back().push_back(val);
      val_count++;
    }
    return *this;
  }
};

// 定义一个名为Vector3D的结构体别名，表示三维向量
typedef struct Point3D Vector3D;

// 表示旋转矩阵的结构体
typedef struct RotationMatrix {
  double M[3][3];
} RotationMatrix;

// 表示变换矩阵的结构体
typedef struct TransformationMatrix {
  double M[4][4];
} TransformationMatrix;

// 三维位置类
typedef struct Position3D {
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
  double rotationMatrix[4][4];
 // 默认构造函数，初始化所有成员变量为0
  Position3D() : x(0.0), y(0.0), z(0.0), yaw(0.0), pitch(0.0), roll(0.0) {}

 // 带参数的构造函数，根据给定的位置和姿态初始化成员变量
  Position3D(double x, double y, double z, double yaw, double pitch,
             double roll) {
    // yaw okolo z
    // pitch okolo y
    // roll okolo x
     // 根据传入的参数初始化成员变量
    this->x = x;
    this->y = y;
    this->z = z;
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
  }

  // 设置平移部分的方法
  void setTranslation(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }

 // 设置旋转部分的方法
  void setRotation(double yaw, double pitch, double roll) {
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
  }

 // 更新旋转矩阵的方法
  void updateRotationMatrix() {
    // XYZ rotation from http://www.songho.ca/opengl/gl_anglestoaxes.html
    // yaw rotation around z
    // pitch rotation around y
    // roll rotation around x
    rotationMatrix[0][3] = this->x;
    rotationMatrix[1][3] = this->y;
    rotationMatrix[2][3] = this->z;
    rotationMatrix[3][3] = 1;

    rotationMatrix[0][0] = cos(this->pitch) * cos(this->yaw);
    rotationMatrix[0][1] = -cos(this->pitch) * sin(this->yaw);
    rotationMatrix[0][2] = sin(this->pitch);

    rotationMatrix[1][0] = sin(this->roll) * sin(this->pitch) * cos(this->yaw) +
                           cos(this->roll) * sin(this->yaw);
    rotationMatrix[1][1] =
      -sin(this->roll) * sin(this->pitch) * sin(this->yaw) +
      cos(this->roll) * cos(this->yaw);
    rotationMatrix[1][2] = -sin(this->roll) * cos(this->pitch);

    rotationMatrix[2][0] =
      -cos(this->roll) * sin(this->pitch) * cos(this->yaw) +
      sin(this->roll) * sin(this->yaw);
    rotationMatrix[2][1] = cos(this->roll) * sin(this->pitch) * sin(this->yaw) +
                           sin(this->roll) * cos(this->yaw);
    rotationMatrix[2][2] = cos(this->roll) * cos(this->pitch);

    rotationMatrix[3][0] = 0;
    rotationMatrix[3][1] = 0;
    rotationMatrix[3][2] = 0;
  }

 // 获取旋转矩阵的方法
  RotationMatrix getRotationMatrix() {
    // 调用updateRotationMatrix方法更新旋转矩阵，并将其转换成RotationMatrix类型返回
    updateRotationMatrix();
    RotationMatrix t;
    memcpy(&t.M[0][0], &(this->rotationMatrix[0][0]), 3 * sizeof(double));
    memcpy(&t.M[1][0], &(this->rotationMatrix[1][0]), 3 * sizeof(double));
    memcpy(&t.M[2][0], &(this->rotationMatrix[2][0]), 3 * sizeof(double));
    return t;
  }

  // 获取变换矩阵的方法
  TransformationMatrix getTransformationMatrix() {
    // 调用updateRotationMatrix方法更新旋转矩阵，并将其转换成TransformationMatrix类型返回
    updateRotationMatrix();
    TransformationMatrix t;
    memcpy(&t.M, this->rotationMatrix, sizeof(TransformationMatrix));
    return t;
  }

// 获取平移向量的方法
  Vector3D getTranslationVector() {
    Vector3D v(this->x, this->y, this->z);
    return v;
  }

 // 生成随机位置的方法
  Position3D random(double posMIN, double posMAX, double rotMIN,
                    double rotMAX) {
    Position3D pos;
    pos.randomFill(posMIN, posMAX, rotMIN, rotMAX);
    return pos;
  }

// 随机填充位置和姿态的方法
  void randomFill(double posMIN, double posMAX, double rotMIN, double rotMAX) {
    this->x = randDoubleMinMax(posMIN, posMAX);
    this->y = randDoubleMinMax(posMIN, posMAX);
    this->z = randDoubleMinMax(posMIN, posMAX);
    this->yaw = randDoubleMinMax(rotMIN, rotMAX);
    this->pitch = randDoubleMinMax(rotMIN, rotMAX);
    this->roll = randDoubleMinMax(rotMIN, rotMAX);
  }
  // 计算与另一个位置的欧氏距离的方法
  double distanceXYZ(Position3D otherPosition) {
    double diffx = otherPosition.getX() - this->x;
    double diffy = otherPosition.getY() - this->y;
    double diffz = otherPosition.getZ() - this->z;
    return sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
  }

// 将位置和姿态转换成向量表示的方法
  std::vector<double> toVector() {
    std::vector<double> vector(6);
    vector[0] = this->x;
    vector[1] = this->y;
    vector[2] = this->z;
    vector[3] = this->yaw;
    vector[4] = this->pitch;
    vector[5] = this->roll;
    return vector;
  }

 // 重载==运算符，判断两个位置是否相等
  inline bool operator==(const Position3D other) {
    if ((getX() != other.x) || (y != other.y) || (z != other.z) ||
        (yaw != other.yaw) || (pitch != other.pitch) || (roll != other.roll)) {
      return false;
    }
    return true;
  }

 // 重载<<运算符，用于输出位置信息
  std::ostream &operator<<(std::ostream &o) {
    o << std::fixed << std::setprecision(6) << "[" << x << "," << y << "," << z
      << "," << yaw << "," << pitch << "," << roll << "]";
    return o;
  }

 // 重载-运算符，计算两个位置之间的差值
  Position3D operator-(const Position3D &other) {
    return Position3D(x - other.x, y - other.y, z - other.z, yaw - other.yaw,
                      pitch - other.pitch, roll - other.roll);
  }

 // 打印位置信息的函数
  void print() {
    // 输出当前位置的信息
    std::cout << std::fixed << std::setprecision(6) << "[" << this->getX()
              << "," << this->getY() << "," << this->getZ() << ","
              << this->getYaw() << "," << this->getPitch() << ","
              << this->getRoll() << "]" << std::endl;
  }

  // 三维位置坐标结构体
typedef struct Position3D {
  double x, y, z; // 三维坐标
  double yaw, pitch, roll; // 偏航角、俯仰角、翻滚角

  // 获取坐标值
  double getX() { return this->x; }
  double getY() { return this->y; }
  double getZ() { return this->z; }
  double getYaw() { return this->yaw; }
  double getPitch() { return this->pitch; }
  double getRoll() { return this->roll; }

  // 设置坐标值
  void setX(double x) { this->x = x; }
  void setY(double y) { this->y = y; }
  void setZ(double z) { this->z = z; }
  void setYaw(double yaw) { this->yaw = yaw; }
  void setPitch(double pitch) { this->pitch = pitch; }
  void setRoll(double roll) { this->roll = roll; }
} Position3D;

// RGB颜色结构体
typedef struct RGBColor {
  float r, g, b; // 红、绿、蓝分量

  // 构造函数
  RGBColor() : r(0.0), g(0.0), b(0.0) {}
  RGBColor(const float &_r, const float &_g, const float &_b)
    : r(_r), g(_g), b(_b) {}

  // 重载加法、减法、乘法、除法运算符
  RGBColor operator+(const RGBColor &p) const {
    return RGBColor(r + p.r, g + p.g, b + p.b);
  }
  RGBColor operator-(const RGBColor &p) const {
    return RGBColor(r - p.r, g - p.g, b - p.b);
  }
  RGBColor operator*(float c) const { return RGBColor(c * r, c * g, c * b); }
  RGBColor operator/(float c) const { return RGBColor(r / c, g / c, b / c); }

  // 打印颜色值
  void print() {
    std::cout << "[" << r << "," << g << "," << b << "]" << std::endl;
  }
} RGBColor;

// 以下为流输出操作符的重载声明
std::ostream &operator<<(std::ostream &o, const Point2D &p);
std::ostream &operator<<(std::ostream &o, const Point3D &p);
std::ostream &operator<<(std::ostream &o, const Position3D &p);
std::ostream &operator<<(std::ostream &o, const RGBColor &c);
std::ostream &operator<<(std::ostream &o, const RotationMatrix &p);
std::ostream &operator<<(std::ostream &o, const TransformationMatrix &p);

/*				 for 3d objects end					 */
/*********************************************************/

// 定义路径分隔符常量为"/"
#define PATH_SEPARATOR std::string("/")

// 获取文件名（不含路径）
std::string getFilename(std::string &fullfilename);

// 获取路径（不含文件名）
std::string getPath(std::string &fullfilename);

/**********************************************************/
/*              文件系统处理相关函数                     */

// 从 YAML 配置中读取参数值，若未找到则报错退出程序
template<typename T>
T loadParam(YAML::Node config, std::string param_name);

// 解析配置文件中的数组类型参数
bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<Point2D> &vector_to_fill);
bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<std::vector<double>> &array2d);
bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<double> &vector_to_fill);

/**********************************************************/
/*                常用类型定义                            */

// 使用 double 作为标量，定义 Matrix 和 Vector 类型
using Scalar = double;
static constexpr Scalar INF = std::numeric_limits<Scalar>::infinity(); // 定义无穷大

template<int rows = Eigen::Dynamic, int cols = rows>
using Matrix = Eigen::Matrix<Scalar, rows, cols>; // 定义矩阵类型

template<int rows = Eigen::Dynamic>
using Vector = Matrix<rows, 1>; // 定义向量类型

template<int rows = Eigen::Dynamic>
using ArrayVec = Eigen::Array<Scalar, rows, 1>; // 定义一维数组类型

// 使用 Eigen 库中的 Quaternion 和 AngleAxis 类型
using Quaternion = Eigen::Quaternion<Scalar>;
using AngleAxis = Eigen::AngleAxis<Scalar>;

// 定义 Eigen 库中的 Ref 和 ConstRef 类型
template<class Derived>
using Ref = Eigen::Ref<Derived>;
template<class Derived>
using ConstRef = const Eigen::Ref<const Derived>;

/**********************************************************/
/*                     TCLAP 命令行解析库                  */

namespace TCLAP {

// 为 Point3D、LoadVector 和 LoadVector2D 类型定义值类型为字符串
template<>
struct ArgTraits<Point3D> {
  typedef StringLike ValueCategory;
};

template<typename T>
struct ArgTraits<LoadVector<T>> {
  typedef StringLike ValueCategory;
};

template<typename T, int dim>
struct ArgTraits<LoadVector2D<T, dim>> {
  typedef StringLike ValueCategory;
};
}  // namespace TCLAP

