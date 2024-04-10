/*
定义了一个计时器类Timer，用于测量代码执行的时间
✨代码分析
该计时器类使用了<sys/time.h>头文件中的函数来获取系统时间：
①  start()函数用于启动计时器，记录开始计时的时间点；
②  stop()函数用于停止计时器，记录停止计时的时间点；
③  reset()函数用于重置计时器，将计时器状态恢复到初始状态；
④  getTimeMS()函数用于获取经过的时间，返回值单位为毫秒。
 */

#pragma once

#include <sys/time.h> // 包含用于获取系统时间的头文件

#include <cstddef> // 包含大小相关的头文件

class Timer {
 public:
  Timer(); // 构造函数
  virtual ~Timer(); // 析构函数
  void start(); // 开始计时
  void stop(); // 停止计时
  void reset(); // 重置计时器
  long getTimeMS(); // 获取经过的时间（毫秒）

 private:
  bool started; // 标记计时器是否已经启动
  time_t start_tv_sec; // 记录开始计时的秒数
  suseconds_t start_tv_usec; // 记录开始计时的微秒数
  time_t stop_tv_sec; // 记录停止计时的秒数
  suseconds_t stop_tv_usec; // 记录停止计时的微秒数
};
