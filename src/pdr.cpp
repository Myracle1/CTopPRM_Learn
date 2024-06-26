#include "pdr.hpp"

template <>
std::string pdr<Vector<3>>::to_string_raw(Vector<3> data) {
  std::stringstream ss;
  ss << data(0) << "," << data(1) << "," << data(2);// 将三维向量转换成逗号分隔的字符串
  return ss.str();// 返回字符串
}