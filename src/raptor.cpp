#include "raptor.hpp"

template <>
//模板特化的开头，表示我们要特化一个模板类或函数。
std::string Raptor<Vector<3>>::to_string_raw(Vector<3> data) {
  //特化版本的 Raptor 类的成员函数 to_string_raw，
  //用于将 Vector<3> 类型的数据转换为字符串。
  //在这里，我们使用了 Eigen 矢量库中的 Vector<3> 类型。
  std::stringstream ss;//创建一个字符串输出流对象。
  ss << data(0) << "," << data(1) << "," << data(2);
  //将 data 中的三个元素按顺序写入到字符串输出流对象中
  return ss.str();//将字符串输出流对象中的内容转换为字符串，并返回该字符串
}