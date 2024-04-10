# CTopPRM_Learn
**This is a detailed study report about a paper published in RA-L, which focused on Motion planning.**
# 前言

> 论文标题：**CTopPRM: Clustering Topological PRM for Planning Multiple Distinct Paths in 3D Environments**
>
> 论文下载地址：[arxiv.org/pdf/2305.13969.pdf](https://arxiv.org/pdf/2305.13969.pdf)
>
> 开源代码地址：[ctu-mrs/CTopPRM: CTopPRM (github.com)](https://github.com/ctu-mrs/CTopPRM)

# 一、项目环境搭建

**系统环境：Ubuntu 20.04**

### 1）下载开源代码

克隆仓库，并更新子模块

```bash
git clone git@github.com:ctu-mrs/CTopPRM.git
```

```bash
cd CTopPRM/
```

```bash
git submodule update --init --recursive
```


```bash
pip install gitman
```


```bash
gitman update
```


###  2）安装依赖项并编译

安装下面的依赖项

```bash
sudo apt-get install build-essential cmake pkg-config ccache zlib1g-dev libomp-dev libyaml-cpp-dev libhdf5-dev libgtest-dev liblz4-dev liblog4cxx-dev libeigen3-dev python3 python3-venv python3-dev python3-wheel python3-opengl
```


 编译子模块中存在的依赖关系

```bash
make dependencies
```


最后，编译代码

```bash
make
```


###  3）准备地图

接下来，我们要从网格 .obj 文件创建 ESDF 映射，使用 python 文件夹中的 map.py 脚本。要拥有所有的依赖项，建议使用 python 环境。启动环境并使用以下方法激活它：

```bash
python3 -m venv env 
source env/bin/activate
```


**【注意】**在每次打开终端之后，我们需要转到工作空间目录，然后使用 **source env/bin/activate** 来激活 python 的虚拟环境，如下所示：

![img](https://img-blog.csdnimg.cn/fb99fc1c3a244ea791d539c4780e19f6.png)

使用 pip 安装python依赖项

```bash
pip install setuptools~=57.5.0
pip install scikit-learn
pip install wheel
pip install pyopengl==3.1.0
pip install numpy trimesh matplotlib mesh_to_sdf python-csv
```

安装依赖项后，在 blender 文件夹中运行以下命令以创建 ESDF 映射（将 MESH_NAME 替换为您要使用的网格文件的名称）：

```bash
./map.py MESH_NAME.obj
```


如下所示：

 ![img](https://img-blog.csdnimg.cn/05fdd46b6aeb42bc8b4fedb306fd172b.png)

------

# 二、项目运行

编译后，我们在工作空间中会看到主二进制文件 main 。

![img](https://img-blog.csdnimg.cn/3b902f9877b1415cbf43069be5375972.png)

为现有地图准备的配置文件存储在config_files文件夹中，可以在其中设置所需的参数和地图。所需的配置文件必须在 main.cpp 中定义。 在代码首次运行前，必须创建一个用于存储结果的目录：

```bash
mkdir prints/
```

最后，在工作空间，我们可以使用以下命令简单地运行代码：

```bash
./main
```


完整过程如下所示：

![img](https://img-blog.csdnimg.cn/54da3999ba524462b059c57e5795fc88.png)

在我们的工作空间会生成很多.csv文件，如下所示：

![img](https://img-blog.csdnimg.cn/e5d6c806f75e49b7a1954ac92526f4fa.png)

------

# 三、项目可视化

**可视化环境：blender**

我们首先需要安装blender：

```bash
sudo apt install blender
```


关于blender的基本介绍和使用，请参考这篇博客[Blender3.5使用python脚本命令的三种方式, 以及后台渲染调用源码示例及说明_blender脚本-CSDN博客](https://blog.csdn.net/vily_lei/article/details/131064566)

关于 .obj 文件和 .mtl 文件请参考这篇博客

[OBJ 模型文件与MTL材质文件 介绍_mtl文件_newchenxf的博客-CSDN博客](https://blog.csdn.net/newchenxf/article/details/121394626)

接下来，我们开始项目的可视化：

**1）终端输入 blender 打开该3D绘图软件**

![img](https://img-blog.csdnimg.cn/7867aefd3e90448cb8b9948785d629f5.png)

软件界面如下所示：![img](https://img-blog.csdnimg.cn/477ee596a24b4feab29d17cbe1052cfd.png)

**2）导入相应的 .obj 文件，我们以 small_random_columns.obj 为例进行说明**

![img](https://img-blog.csdnimg.cn/e18bb0e8312941229ee4ae4c076e5492.png)

我们打开文件夹，选择 small_random_columns.obj 导入，得到模型如下所示：

![img](https://img-blog.csdnimg.cn/c81d7bf87dbd4ec596cac7525ef09e78.png)

**3）随后，打开 blender 文件夹中的 columns_2.py 文件，我们将其进行修改后运行**

修改文件对应的工作路径：本人改为 /home/shczby/ROBOT/Course_Project/CTopPRM，修改位置在python代码column_2.py的185行和541行。

最后，将column_2的代码复制到blender的脚本中，点击运行脚本即可。如下所示：

![img](https://img-blog.csdnimg.cn/ce02a9d411104985a5bfcb4ea47c9d87.png)

虽然可视化成功了，但是出现了一个新的问题，就是最终生成的路径方向和障碍物的平面竟然是垂直的！显然这个可视化出现了问题，需要进行修改才行。

我们打开运行 blender 软件的终端，查看blender软件的运行信息如下：

![img](https://img-blog.csdnimg.cn/06bbb62ad8bb437ebd9ff56067f3211f.png)

**[注意]**
 在该目录下的csv文件为./map/py random_columns.obj之后，再运行./main之后生成的文件
 可视化代码路径导入的csv文件与blender中导入的.obj文件应该要对应起来，否则会出错。
 要生成全部可视化路线图，每种情况都要生成一遍.csv文件，再依次在blender中可视化。 

------

# 四、出现的问题及解决方案

### 问题一：terminate called after throwing an instance 

> terminate called after throwing an instance 
>
> Unable to open file blender/1-2-1.obj.npy 已放弃 (核心已转储)

![img](https://img-blog.csdnimg.cn/000d927070a94c67be67040bd187427c.png)

【解决方案】

```bash
mkdir prints
```

先需要这样创建prints文件夹，然后运行 ./main ，否则terminate called after throwing an instance 

### 问题二：运行可视化代码时报错，没有bqy的包，且无法安装

> 没有bqy的包，且无法安装

【解决方案】
 [fake-bpy-module-2.80 · PyPI](https://pypi.org/project/fake-bpy-module-2.80/#description)

[Blender插件开发：用fake-bpy-module提供代码补全_fake_bpy_modules-CSDN博客](https://blog.csdn.net/ttm2d/article/details/102795545?ops_request_misc=&request_id=&biz_id=102&utm_term=blender2.82对应bpy什么版本&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-2-102795545.142^v96^control&spm=1018.2226.3001.4187)

------
**【复现过程中主要遇到的问题及其解决方案】**

> 1. **\*运行./\******\*main的时候会出现no point added to shortened path\***：正常信息，表示在检测碰撞后，无后续节点加入到缩短后的路径当中。
> 2. **\*blender可视化路径穿墙\***：路径生成脚本和obj文件角度相差90度，旋转过来即可。
> 3. **\*columns_2.py\******\*运行\*****\*时color\******\*[\*******\*id\*******\*]\*******\*报错\****：max_cl最大聚类数不正确，将数值改大即可。

# 五、补充说明（完整版）

CTopPRM论文给出的开源代码，是C++和python结合的，其中C++用于实现核心的算法，而python用于可视化和文件处理，项目借助Blender渲染软件进行可视化，需要先运行map.py脚本将.obj文件转化为能直接使用的.npy文件，然后再运行main二进制文件即可生成很多的.csv路图文件。最后，将obj文件导入到blender中，运行可视化的python脚本文件，将工作路径的csv路图文件可视化即可。

1、按照Github上的Markdown文件在终端依次执行即可，大致步骤有配置环境(python虚拟环境)、下载相应的依赖项(dependencies)、使用gitman进行项目仓库的更新，最后在终端输入make进行编译，使用map.py脚本将.obj文件转换为可以供C++项目使用的环境文件.npy，然后运行二进制文件main即可。

![img](https://img-blog.csdnimg.cn/direct/da77195768784dd19bba6db89217a9be.png)

2、在blender中导入相应的.obj文件，可视化场景环境。

3、然后把columns_2(可视化用这个).py文件复制到blender的scripts脚本编辑器中，运行即可。在运行之前，注意将场景旋转一下。

4、对于其他算法的复现，其实只需要修改main函数的一个类名即可，如下所示：

（main.cpp为用于分别实现4种算法的主函数；在本人代码中的 277 行，本人给出了4种算法的具体实现过程；代码前面部分主要是数据处理和参数配置，后面部分是算法的实现，详细说明请阅读注释~），***\*需要注意两个地方的修改\****：

***\*一个是yaml文件的替换：\****yaml文件中有对应的地图文件，因此我们需要先阅读yaml文件，大致在17行左右，yaml和obj文件是一一对应的，如下所示：

![img](https://img-blog.csdnimg.cn/direct/9fa6b917ac1f439092cc32b4bc80db87.png)


有时候运行不了，显示如下的问题，那是因为没找到对应的文件或者文件夹，如下面的问题就是没有建立prints文件夹，如果是找不到.npy文件，那就是没有运行map.py脚本，将想要复现的obj文件转换为可使用的.npy文件：

![img](https://img-blog.csdnimg.cn/direct/1dfa316a8b8547e889f323c1e3a6a160.png)

在明确了yaml文件对应的地图后，**需要在main.cpp中进行如下改动，\**这\**是作者在Github上没有说明的一点：**

![img](https://img-blog.csdnimg.cn/direct/5e572c39529a44da99003350dec281ff.png)

***\*一个是算法的替换：\****

在思考为何作者能对四种算法进行对比的时候，个人认为应该要在同一场景下，即相同的窗户/柱子/建筑环境下，才能进行算法性能的对比，因此，在精读代码之后，我找到了切换算法的函数接口，如下所示：

![img](https://img-blog.csdnimg.cn/direct/394061e705234224afbf834da5ede445.png)

在main.cpp中***\*修改yaml和函数（算法类）接口\****之后，即可分别实现这4种算法的在相应环境下的运行，得到计算时间和找到路径数量两个结果，用于论文的算法对比。

另外输出的注解如下所示：

![img](https://img-blog.csdnimg.cn/direct/482020bd87024256a58645a780b85daa.png)

在算法类中修改，将注释掉的INFO或者文件处理代码取消注释，可得到不同输出结果。

# 六、附录（主要代码&复现结果）

### 1）main.cpp

```cpp
/* main.cpp文件
✨main.cpp为用于分别实现4种算法的主函数
💎在该代码中的 277 行，本人给出了4种算法的具体实现过程
👉代码前面部分主要是数据处理和参数配置，后面部分是算法的实现，详细说明请阅读注释~
*/
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/logger.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <chrono>

#include "common.hpp"
#include "timer.hpp"
#include "prm.hpp"

#include "topological_prm.hpp"
#include "topological_prm_clustering.hpp"
#include "raptor.hpp"
#include "pdr.hpp"

using namespace log4cxx;

std::string planner_config_file = "./config_files/building.yaml";//不同的场景需要切换yaml配置文件
//最为经典的三个场景：窗户——1-3-1；柱子——small_poles；建筑物——building。

std::string problemFile;

std::string canvasOutput = "";

YAML::Node config;
std::vector<double> start_loaded;
std::vector<double> end_loaded;
std::vector<Point2D> borders_loaded;
std::vector<Point2D> gates_loaded;
std::string world_file;
// std::vector<MeshObject *> objects;
std::vector<std::vector<double>> array;

std::string logfile;
std::string name;
std::string map_file;
std::string map_type;
std::string output_folder{"./prints/"};
Vector<3> start;
Vector<3> goal;

YAML::Node planner_config;
Scalar min_clearance_;
bool check_collisions_;
Scalar collision_distance_check_;

void parse_cmd_args(int argc, char **argv) {
  // overwrite the variables from the yaml with the one from command line

  try {
    TCLAP::CmdLine cmd("Topological planning program", ' ', "0.0");

    TCLAP::ValueArg<std::string> nameArg("", "name", "name", false,
                                         std::string("no_name"), "string", cmd);

    TCLAP::ValueArg<std::string> logfileArg("", "logfile", "logfile", false,
                                            logfile, "string", cmd);

    TCLAP::ValueArg<std::string> output_folderArg("", "output_folder",
                                                  "output_folder", false,
                                                  output_folder, "string", cmd);

    TCLAP::ValueArg<std::string> mapArg("", "map", "map", false, map_file,
                                        "string", cmd);

    TCLAP::ValueArg<LoadVector<double>> start_p_Arg("", "start_p", "start_p",
                                                    false, LoadVector<double>(),
                                                    "Vector<3>", cmd);
    TCLAP::ValueArg<LoadVector<double>> goal_p_Arg(
        "", "goal_p", "goal_p", false, LoadVector<double>(), "Vector<3>", cmd);

    cmd.parse(argc, argv);

    name = nameArg.getValue();
    // INFO("loaded name " << name)
    output_folder = output_folderArg.getValue();
    map_file = mapArg.getValue();
    logfile = logfileArg.getValue();

    // INFO("loaded logfile " << logfile)
    // INFO("map_file " << map_file)
    // INFO("creating output_folder " << output_folder)

    std::filesystem::create_directories(output_folder);

    if (start_p_Arg.isSet()) {
      LoadVector<double> start_cmd = start_p_Arg.getValue();
      Vector<3> new_start(start_cmd.vector[0], start_cmd.vector[1],
                          start_cmd.vector[2]);
      // INFO_CYAN("changing start from " << start.transpose() << " to "
      //                                 << new_start.transpose())
      start = new_start;
    }

    if (goal_p_Arg.isSet()) {
      LoadVector<double> goal_cmd = goal_p_Arg.getValue();
      Vector<3> new_goal(goal_cmd.vector[0], goal_cmd.vector[1],
                         goal_cmd.vector[2]);
      // INFO_CYAN("changing end from " << goal.transpose() << " to "
      //                               << new_goal.transpose())
      goal = new_goal;
      // exit(1);
    }

  } catch (TCLAP::ArgException &e) {
    std::cerr << "cmd args error: " << e.error() << " for arg " << e.argId()
              << std::endl;
    exit(1);
  }
}

std::string to_string_raw(Vector<3> data) {
  std::stringstream ss;
  ss << data(0) << "," << data(1) << "," << data(2);
  return ss.str();
}

void savePath(std::string filename, std::vector<HeapNode<Vector<3>> *> path) {
  // // INFO("save TopologicalPRM map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t ni = 1; ni < path.size(); ni++) {
      std::string from_str = to_string_raw(path[ni - 1]->data);
      std::string to_str = to_string_raw(path[ni]->data);
      myfile << from_str << "," << to_str << std::endl;
    }
    myfile.close();
  }
}

void savePaths(std::string output_folder, std::string method, path_with_length<Vector<3>> path, int pi) {
    std::stringstream path_ss;
    // // INFO("shortened length " << shortened_paths[pi].length)
    path_ss << output_folder << method << "_roadmap_shortened_unique_path" << 0 << "_"
            << pi << ".csv";
    // // INFO("juchuuuuu")
    savePath(path_ss.str(), path.plan);
    // // INFO("saved");
}

void save_double(std::string filename, double d) {
  std::ofstream myfile;
  myfile.open(filename.c_str(), std::ios_base::app);

  if (myfile.is_open()) {
    std::string d_str = std::to_string(d);
    myfile << d_str << std::endl;
    myfile.close();
  }
}

void removeMethodFiles(std::string output_folder) {

  std::stringstream ss_roadmap;
  ss_roadmap << output_folder;
  std::string ss_roadmap_str = ss_roadmap.str();
  std::cout << ss_roadmap_str << std::endl;

  for (const auto &entry :
       std::filesystem::directory_iterator(output_folder)) {
    // std::cout << entry.path().string() << std::endl;
    const std::string path_name = entry.path().string();
    if (path_name.compare(0, ss_roadmap_str.length(), ss_roadmap_str) == 0) {
      std::cout << "removing " << path_name << std::endl;
      std::filesystem::remove(entry.path());
    }
  }
}

void removeRoadmapFiles(std::string output_folder) {

  std::stringstream ss_roadmap;
  ss_roadmap << output_folder << "roadmap";
  std::string ss_roadmap_str = ss_roadmap.str();
  std::cout << ss_roadmap_str << std::endl;

  for (const auto &entry :
       std::filesystem::directory_iterator(output_folder)) {
    // std::cout << entry.path().string() << std::endl;
    const std::string path_name = entry.path().string();
    if (path_name.compare(0, ss_roadmap_str.length(), ss_roadmap_str) == 0) {
      std::cout << "removing " << path_name << std::endl;
      std::filesystem::remove(entry.path());
    }
  }
}

int test_planner(int argc, char **argv) {
  //去除在当前文件夹下输出的保存路径数据的文件
  removeMethodFiles(output_folder);
  removeRoadmapFiles(output_folder);
  removeRoadmapFiles("./");

  planner_config = YAML::LoadFile(planner_config_file);//加载配置信息
  //解析地图
  map_type = loadParam<std::string>(planner_config, "map_type");
  map_file = loadParam<std::string>(planner_config, "map");
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");
  check_collisions_ = loadParam<bool>(planner_config, "check_collisions");
  collision_distance_check_ =
      loadParam<double>(planner_config, "collision_distance_check");
  //获取到起点和终点的位置
  if (planner_config["start"] && planner_config["end"]) {
    // define start pos
    if (planner_config["start"]["position"]) {
      std::vector<double> start_pos;
      parseArrayParam(planner_config["start"], "position", start_pos);
      start(0) = start_pos[0];
      start(1) = start_pos[1];
      start(2) = start_pos[2];
    } else {
      INFO_RED("you must specify start position");
      exit(1);
    }
    if (planner_config["end"]["position"]) {
      std::vector<double> end_pos;
      parseArrayParam(planner_config["end"], "position", end_pos);
      goal(0) = end_pos[0];
      goal(1) = end_pos[1];
      goal(2) = end_pos[2];
    } else {
      INFO_RED("you must specify end position");
      exit(1);
    }
  } else {
    INFO_RED("you must specify start and end position");
    exit(1);
  }
  // SST sst(planner_config, drone_config);
  parse_cmd_args(argc, argv);//解析命令行参数
  // singnal_handler_ = &sst;
  // register singal for killing
  // std::signal(SIGINT, signal_callback_sst);
  // sst.iterate();

  std::shared_ptr<BaseMap> map;
  // load map 加载地图信息
  if (map_type == "ESDF") {
    map = std::make_shared<ESDFMap>();
  } else if (map_type == "PC") {
    ERROR("map type " << map_type << " not implemented")
  } else {
    ERROR("map type " << map_type << " not recognized")
    exit(1);
  }
  map->load(map_file);
  //创建含有起点和终点的容器
  std::vector<Vector<3>> gates_with_start_end_poses;
  gates_with_start_end_poses.push_back(start);
  gates_with_start_end_poses.push_back(goal);
  // INFO("start:" << start.transpose() << " goal " << goal.transpose());
  //实行路径规划
  std::stringstream clT_ss, clL_ss, clN_ss;
  clT_ss << output_folder << planner_config_file << "method_clustering_time.csv";
  clL_ss << output_folder << planner_config_file << "method_clustering_length.csv";
  clN_ss << output_folder << planner_config_file << "method_clustering_num.csv";

  //计算并打印出路径规划的时间，
  //其中4种算法的路径规划均在find_geometrical_paths函数中实现
  auto begin_c = std::chrono::high_resolution_clock::now();
  std::vector<std::vector<path_with_length<Vector<3>>>> paths_between_gates =
      pdr<Vector<3>>::find_geometrical_paths(
        //要复现论文比较的这4种算法的运行结果（主要评价指标：计算时间+不同路径数量）
        //只需要将<Vector>前面的类名修改即可，因为项目给出了4种算法的代码，并封装为类保存在hpp中
        //类名如下：
        //方法1：TopologicalPRMClustering
        //方法2：TopologicalPRM
        //方法3：Raptor
        //方法4：pdr
          planner_config, map, gates_with_start_end_poses, output_folder);
  auto end_c = std::chrono::high_resolution_clock::now();
  auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);
  
  //计算规划时间和找到的路径数量
  //其实这个代码可以注释掉，因为在4种算法的hpp结合cpp的文件中，都已经自带了终端输出计算时间的语句
  // INFO_GREEN("computation time clustering " << elapsed_c.count() * 1e-9)
  //使用不同的算法，即将clustering改为对应的算法名称，即clustring,raptor,sphere和pdr，分别对应的是论文中比较的4种算法
  //想要输出除roadmap_clustering_roadmap_shortened_unique_path0_0.csv（最终路径）之外的其他路径csv文件，
  //在对应算法的hpp文件中将output_folder相关语句取消注释即可。
  //同样，想让CTopPRM算法不输出乱七八糟的文件，在hpp文件中注释掉相关语句即可
  INFO_GREEN("number of paths found " << paths_between_gates[0].size())
  save_double(clT_ss.str(), elapsed_c.count() * 1e-9);
  save_double(clN_ss.str(), paths_between_gates[0].size());

  //保存每条路径的长度和具体信息
  for (int i=0; i<paths_between_gates[0].size(); i++) {
    // INFO("path " << i << " with length " << paths_between_gates[0][i].length)
    savePaths(output_folder, "roadmap_clustering", paths_between_gates[0][i], i);
    save_double(clL_ss.str(), paths_between_gates[0][i].length);
  }
  // INFO_GREEN("finished");
  return 0;
}

int main(int argc, char **argv) {
  startLogger("main");

  seed();

  test_planner(argc, argv);
  return 1;
}
```


### 2）columns_2.py(可视化脚本)

**主要修改了如下几处**：

**1）修改.csv文件保存的路径：**![img](https://img-blog.csdnimg.cn/direct/d7c3e05f432b44d78dad7cc8bd4d2295.png)

**2） 报错的原因是max_cl最大聚类数不正确。可以将其注释掉，或者将数值改大。**

![img](https://img-blog.csdnimg.cn/direct/d02aeda912824d43a9c7c8a91175b6f9.png)

**完整用于可视化的python脚本文件如下所示：** 

```python
import bpy
import glob
import csv, copy
import mathutils
import math
import random
import colorsys

def update_camera(camera, location ,focus_point=mathutils.Vector((0.0, 0.0, 0.0)), distance=10.0):
    """
    Focus the camera to a focus point and place the camera at a specific distance from that
    focus point. The camera stays in a direct line with the focus point.
    """
    looking_direction = location - focus_point
    rot_quat = looking_direction.to_track_quat('Z', 'Y')

    camera.rotation_euler = rot_quat.to_euler()
    camera.location = location

def load_roadmap(file):
    samples = []
    edges = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            col = []
            for c in row:
                col.append(float(c))
            if len(row)==3:
                samples.append(col)
            else:
                edges.append(col)
    return samples, edges


def load_trajectory_samples_sst(file):
    print("load_trajectory_samples ",file)
    edges = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        header = next(csvreader)
        last_pos = None
        for row in csvreader:
            col = []
            for c in row:
                col.append(float(c))
            print(col)
            new_pos = [col[3],col[4],col[5]]
            if last_pos is not None:
                edges.append(last_pos + new_pos)    
            last_pos = new_pos
    #print(edges)
    return edges


def load_trajectory_samples_cpc(file):
    print("load_trajectory_samples ",file)
    edges = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        header = next(csvreader)
        last_pos = None
        for row in csvreader:
            col = []
            for c in row:
                col.append(float(c))
            new_pos = [col[1],col[2],col[3]]
            #print("new pos ",new_pos)
            if last_pos is not None:
                edges.append(last_pos + new_pos)    
            last_pos = new_pos
    #print(edges)
    return edges

def load_trajectory_samples_pmm(file,header=True):
    print("load_trajectory_samples ",file)
    edges = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        if header:
            header = next(csvreader)
        last_pos = None
        for row in csvreader:
            col = []
            for c in row:
                col.append(float(c))
            new_pos = [col[1],col[2],col[3]]
            if last_pos is not None:
                edges.append(last_pos + new_pos)    
            last_pos = new_pos
    #print(edges)
    return edges

def plot_curve(edgelist,name, color = (0.0,1.0,0.0,1.0),width=0.01,material=None):
    crv = bpy.data.curves.new('crv', 'CURVE')
    crv.dimensions = '3D'
    spline = crv.splines.new(type='POLY')
    #one point is there already
    spline.points.add(1) 
    edge = edgelist[0]
    spline.points[-2].co = ([edge[0],edge[1],edge[2], 1.0])
    spline.points[-1].co = ([edge[3],edge[4],edge[5], 1.0])
    if material is None:
        material = bpy.data.materials.new(name+"_material")
        material.diffuse_color = color
        crv.materials.append(material)
    else:
        crv.materials.append(material)
    crv.bevel_depth = width
    
    for edgeid in range(1,len(edgelist)):
        edge = edgelist[edgeid]
        #print(edge)
        spline.points.add(2) 
        #print(type(spline.points[-2]))
        spline.points[-2].co = ([edge[0],edge[1],edge[2], 1.0])
        spline.points[-1].co = ([edge[3],edge[4],edge[5], 1.0])

    obj = bpy.data.objects.new(name, crv)
    bpy.data.scenes[0].collection.objects.link(obj)
    
def point_cloud(ob_name, coords, edges=[], faces=[]):
    """Create point cloud object based on given coordinates and name.

    Keyword arguments:
    ob_name -- new object name
    coords -- float triplets eg: [(-1.0, 1.0, 0.0), (-1.0, -1.0, 0.0)]
    """

    # Create new mesh and a new object
    me = bpy.data.meshes.new(ob_name + "Mesh")
    ob = bpy.data.objects.new(ob_name, me)

    # Make a mesh from a list of vertices/edges/faces
    me.from_pydata(coords, edges, faces)

    # Display name and update the mesh
    ob.show_name = True
    me.update()
    return ob

is_cl = True

c = '2'
max_cl = 8

folder = 'shortened'    
# prm clusters connections shortened
render = False

to_plot = {
    'centers':False,
    'clusters':False, 
    'prm':False,
    'connections':False,
    'shortened':False,
    'paths':False,
    'two':False,
    'conn':False,
    'plus1':False
    }
    
if folder == 'prm':
    to_plot['prm'] = True
    to_plot['two'] = True
if folder == 'clusters':
    to_plot['prm'] = False
    to_plot['clusters'] = True
    to_plot['centers'] = True
if folder == 'connections':
    to_plot['centers'] = True
    to_plot['connections'] = True
if folder == 'shortened':
    to_plot['shortened'] = True
    
colors = {}
for i in range(max_cl):
    rand_color = colorsys.hsv_to_rgb(i/(max_cl),1,1)
    colors[i] = (rand_color[1], rand_color[0], rand_color[2], 1.0)
    
method = '/prints/roadmap_cl*.csv'

name = 'generated_curve'    
#project = '/home/robert/rpg_workspace/droneracing_planner'
#project = '/home/novosma2/Documents/homotopy_planning/topological_planning'
project = '/home/shczby/ROBOT/Course_Project/CTopPRM'
roadmap_shortened_unique_path_files = glob.glob(project+method)
#roadmap_shortened_unique_path_files = glob.glob(project+'/roadmap_shortened_unique_path*.csv')
if is_cl:
    roadmap_path_files = glob.glob(project+'/roadmap_path*.csv')
    roadmap_seeds_cluster_file = project+'/roadmap_seeds_cluster.csv'
    roadmap_files = glob.glob(project+'/prints/roadmap_all*.csv')
    roadmap_conn_files = glob.glob(project+'/roadmap_'+c+'_min*.csv')
    roadmap_con_files = glob.glob(project+'/roadmap_'+c+'_max*.csv')
    #between_cluster_path_files =  glob.glob(project+'/roadmap_path_cluster*.csv')
    trajectory_file_pmm = project+'/samples.csv'
    trajectory_file_sst = project+'/path.csv'
    trajectory_file_sst_dense = project+'/path_dense.csv'
    trajectory_file_polynomial = project+'/polynomial_path.csv'
    trajectory_file_polynomial_reference = project+'/shortest_position_path.csv'
    roadmap_two_clusters_files = glob.glob(project+'/roadmap_'+c+'_cluster_*.csv')
    roadmap_clusters_files = glob.glob(project+'/roadmap_cluster_*.csv')
    roadmap_shortened_path_files = glob.glob(project+'/roadmap_shortened_path*.csv')
    roadmap_shortened_correct_dir_path_files = glob.glob(project+'/roadmap_shortened_correct_dir_path*.csv')


cpc_project='/home/robert/rpg_workspace/time_optimal_trajectory'
cpc_trajectory_file=cpc_project+'/results/arena_obst/final.csv'



#remove old generated paths
for model in bpy.data.objects:
    print(model)
    if name in model.name:
        bpy.data.objects.remove(model)

print("after removal")
# print("about to load files",roadmap_shortened_path_files)

#trajectory_pmm = load_trajectory_samples_pmm(trajectory_file_pmm)
#trajectory_sst = load_trajectory_samples_sst(trajectory_file_sst)
#trajectory_sst_dense = load_trajectory_samples_pmm(trajectory_file_sst_dense)
#trajectory_polynomial = load_trajectory_samples_pmm(trajectory_file_polynomial)
#trajectory_polynomial_reference = load_trajectory_samples_pmm(trajectory_file_polynomial_reference,False)
#trajectory_cpc = load_trajectory_samples_cpc(cpc_trajectory_file)


#print(trajectory_polynomial_reference)
#print(trajectory_sst)

if is_cl:
    roadmap_edges = {}
    roadmap_samples = {}
    for file in roadmap_files:
        print("loading ",file)
        print(file.split("/")[-1].replace("roadmap_all","").replace(".csv",""))
        id = int(file.split("/")[-1].replace("roadmap_all","").replace(".csv",""))
        samples,edges = load_roadmap(file)
        roadmap_edges[id] = edges
        roadmap_samples[id] = samples

    cluster_edges = {}
    cluster_samples = {}
    for file in roadmap_clusters_files:
        print("loading ",file)
        print(file.split("/")[-1].replace("roadmap_cluster_","").replace(".csv",""))
        id = int(file.split("/")[-1].replace("roadmap_cluster_","").replace(".csv",""))
        samples,edges = load_roadmap(file)
        cluster_edges[id] = edges
        cluster_samples[id] = samples
        
    two_edges = {}
    two_samples = {}
    for file in roadmap_two_clusters_files:
        print("loading ",file)
        print(file.split("/")[-1].replace("roadmap_"+c+"_cluster_","").replace(".csv",""))
        id = int(file.split("/")[-1].replace("roadmap_"+c+"_cluster_","").replace(".csv",""))
        samples,edges = load_roadmap(file)
        two_edges[id] = edges
        two_samples[id] = samples

    cluster_seed_samples, _ = load_roadmap(roadmap_seeds_cluster_file)
            
    path_shortened_edges = []
    for file in roadmap_shortened_path_files:
        #print("loading ",file)
        samples,edges = load_roadmap(file)
        path_shortened_edges.append(edges)
        
    path_shortened_correct_dir_edges = []
    for file in roadmap_shortened_correct_dir_path_files:
        samples,edges = load_roadmap(file)
        path_shortened_correct_dir_edges.append(edges)
        
    between_cluster_path_files =  glob.glob(project+'/roadmap_distinct_path*.csv')
    roadmap_path_files = glob.glob(project+'/roadmap_path*.csv')
    between_cluster_paths = []
    for file in between_cluster_path_files:
        samples,edges = load_roadmap(file)
        between_cluster_paths.append(edges)
        
if is_cl:

            
    paths = []
    for file in roadmap_path_files:
        #print("loading ",file)
        samples,edges = load_roadmap(file)
        paths.append(edges)
        
    path_min = []
    for file in roadmap_conn_files:
        #print("loading ",file)
        samples,edges = load_roadmap(file)
        path_min.append(edges)
    path_max = []
    for file in roadmap_con_files:
        #print("loading ",file)
        samples,edges = load_roadmap(file)
        path_max.append(edges)
        
path_shortened_unique_edges = []
for file in roadmap_shortened_unique_path_files:
    #print("loading ",file)
    samples,edges = load_roadmap(file)
    path_shortened_unique_edges.append(edges)
    


#for path_edges in roadmap_edges:
#    #print(["path_edges ",path_edges)
#    for edge in path_edges:
#        plot_curve([edge],name,color=(1.0,0,0,1.0))

if is_cl:
    all_edges = roadmap_edges[0]    
        
    if not to_plot['prm']:
        all_edges = []
    else:
        rand_color = (0.0,0.0, 0.0,1.0)
        for edge in all_edges:
            width=0.02
            plot_curve([edge],name,color=rand_color,width=width)
    
    if to_plot['plus1']:
        for id in range(len(cluster_edges)):
            if id > int(c):
                cluster_edges.pop(id, None)
    elif not to_plot['centers']:
        cluster_edges = {}
    if cluster_edges:
        step = 0.1
        value = 0
    for id in cluster_edges:
        path_edges = cluster_edges[id]
        print("value is ", value)

        #print(["path_edges ",path_edges)
        
        mat_name = 'colcl' + str(id)
        mat = bpy.data.materials.get(mat_name)
        if mat is None:
            mat = bpy.data.materials.new(name=mat_name)
            rand_color = (random.random(),random.random(), random.random(),1.0)
            mat.diffuse_color = rand_color
            
        
        rand_color = colorsys.hsv_to_rgb(value,1,1)
        if id == 0:
            rand_color = colorsys.hsv_to_rgb(0.25,1,1)
        if id == 1:
            rand_color = colorsys.hsv_to_rgb(0.75,1,1)
            
            
        
        rand_color = (rand_color[1], rand_color[0], rand_color[2], 1.0)
        
        if abs(value-0.75) < step/2:
            rand_color = (1.0, 0.0, 0.0, 1.0)
        value += step 
        
        rand_color = colors[id] #实现随即上色。有时候，对于connections和clusters需要将其注释，对于prm和shortened不需要
        
        print("color is ", rand_color)
        mat.diffuse_color = rand_color
        
        print("cl",id)    
        obj_copy = bpy.context.scene.objects['cluster'].copy()
        obj_copy.data = obj_copy.data.copy() # linked = False
        obj_copy.name = 'generated_curve_cl'+str(id)
        bpy.context.collection.objects.link(obj_copy)
        
        obj_copy = bpy.context.scene.objects['generated_curve_cl'+str(id)]  
        endpos = cluster_seed_samples[id]
        print(cluster_seed_samples[id])
        obj_copy.location = mathutils.Vector((endpos[0],endpos[1],endpos[2]))    
        
        obj_copy.data.materials.append(mat)
        
        
        if not to_plot['clusters']:
            path_edges = []
        for edge in path_edges:
            plot_curve([edge],name,material=mat,width=0.025)
            
        
    if not to_plot['two'] and not to_plot['shortened']:
        two_edges = {}
    if to_plot['shortened']:
        for i in range(2, len(two_edges)):
            two_edges.pop(i, None)
    value = 0
    for id in two_edges:
        path_edges = two_edges[id]
        print("value is ", value)

        #print(["path_edges ",path_edges)
        
        mat_name = 'colcl' + str(id)
        mat = bpy.data.materials.get(mat_name)
        if mat is None:
            mat = bpy.data.materials.new(name=mat_name)
            rand_color = (random.random(),random.random(), random.random(),1.0)
            mat.diffuse_color = rand_color
            
        
        rand_color = colorsys.hsv_to_rgb(value,1,1)
        if id == 0:
            rand_color = colorsys.hsv_to_rgb(0.25,1,1)
        if id == 1:
            rand_color = colorsys.hsv_to_rgb(0.75,1,1)
            
            
        
        rand_color = (rand_color[1], rand_color[0], rand_color[2], 1.0)
        
        if abs(value-0.75) < 0.05:
            rand_color = (1.0, 0.0, 0.0, 1.0)
        value += 0.1 
        rand_color = colors[id]
        
        print("color is ", rand_color)
        mat.diffuse_color = rand_color
        
        print("cl",id)    
        obj_copy = bpy.context.scene.objects['cluster'].copy()
        obj_copy.data = obj_copy.data.copy() # linked = False
        obj_copy.name = 'generated_curve_cl'+str(id)
        bpy.context.collection.objects.link(obj_copy)
        
        obj_copy = bpy.context.scene.objects['generated_curve_cl'+str(id)]  
        endpos = cluster_seed_samples[id]
        print(cluster_seed_samples[id])
        obj_copy.location = mathutils.Vector((endpos[0],endpos[1],endpos[2]))    
        
        obj_copy.data.materials.append(mat)
        
        
        if not to_plot['two']:
            path_edges = []
        for edge in path_edges:
            plot_curve([edge],name,material=mat,width=0.025)
            
    if to_plot['conn']:        
        color = (1.0, 0.0, 0.0, 1.0)
        for edge in path_max:
            plot_curve(edge,name,color=color,width=0.15)
        color = (0.0, 1.0, 0.0, 1.0)
        for edge in path_min:
            plot_curve(edge,name,color=color,width=0.15)
        
        
#for path_edges in between_cluster_paths:
#    for edge in path_edges:
#        plot_curve([edge],name,color = (1.0,0.0,1.0,1.0),width=0.08)
    
#tst = bpy.ops.mesh.primitive_ico_sphere_add(radius=0.5, location=(0, 0, 1.3)) 
#print("mesh",tst)
#pc = point_cloud("point-cloud", )
#bpy.context.collepction.objects.link(pc)

#for path_edges in paths:
#    for edge in path_edges:
#        plot_curve([edge],name)p


#for path_edges in path_shortened_edges:
#    for edge in path_edges:
#        plot_curve([edge],name)
        
#for path_edges in path_shortened_correct_dir_edges:
#    for edge in path_edges:
#        plot_curve([edge],name)


if not to_plot['connections']:
    paths = []
if paths:
    step = 1 / len(paths)
    step=0
    value = 0
for path_edges in paths:
    rand_color = (random.random(),random.random(), random.random(),1.0)
    rand_color = colorsys.hsv_to_rgb(value,1,1)
    rand_color = (rand_color[1], rand_color[0], rand_color[2], 1.0)
    rand_color = (0.0, 0.0, 0.0, 1.0)
    value += step
#    #print("path_edges ",path_edges)
#    #plot_curve(path_edges,name)
    for edge in path_edges:
        plot_curve([edge],name,color=rand_color,width=0.1)

if not to_plot['shortened']:
    path_shortened_unique_edges = []
if path_shortened_unique_edges:
    step = 1 / len(path_shortened_unique_edges)
    value = 0
for path_edges in path_shortened_unique_edges:
    rand_color = (random.random(),random.random(), random.random(),1.0)
    rand_color = colorsys.hsv_to_rgb(value,1,1)
    rand_color = (rand_color[1], rand_color[0], rand_color[2], 1.0)
    value += step
#    #print("path_edges ",path_edges)
#    #plot_curve(path_edges,name)
    for edge in path_edges:
        plot_curve([edge],name,color=rand_color,width=0.1)

if not to_plot['paths']:
    between_cluster_paths = []
if between_cluster_paths:
    step = 1 / len(between_cluster_paths)
    value = 0
for path_edges in between_cluster_paths:
    rand_color = (random.random(),random.random(), random.random(),1.0)
    rand_color = colorsys.hsv_to_rgb(value,1,1)
    rand_color = (rand_color[1], rand_color[0], rand_color[2], 1.0)
    value += step
#    #print("path_edges ",path_edges)
#    #plot_curve(path_edges,name)
    for edge in path_edges:
        plot_curve([edge],name,color=rand_color,width=0.025)

#plot_curve(trajectory_pmm,name+'pmm',color = (1.0,0.0,1.0,1.0),width=0.1)
#plot_curve(trajectory_sst,name+'sst',color = (0.0,1.0,0.0,1.0),width=0.1)
#plot_curve(trajectory_sst_dense,name+'sstdense',color = (0.0,1.0,0.0,1.0),width=0.1)

#plot_curve(trajectory_polynomial,name+'poly',color = (1.0,1.0,0,1.0),width=0.1)
#plot_curve(trajectory_polynomial_reference,name+'polyref',color = (1.0,1.0,1.0,1.0),width=0.1)

#plot_curve(trajectory_cpc,name+'cpc',color = (1.0,0.0,0,1.0),width=0.1)

    
if render:
    for i in range(90, 360, 360):
        ang = math.pi * i/180.0
        center = mathutils.Vector((2.0, -2.0, 0.0))
        camera_pos = center + mathutils.Vector((20*math.cos(ang), 20*math.sin(ang), 75))
        update_camera(bpy.data.objects['Camera'],camera_pos,focus_point=center)
#       bpy.context.scene.render.filepath =  '/home/novosma2/Pictures/topological_planning/'+folder+'/'+folder+str(i)+c+str(to_plot['conn'])+str(to_plot['plus1'])
        bpy.context.scene.render.filepath =  '/home/shczby/ROBOT/Course_Project/CTopPRM'+folder+'/'+folder+str(i)+c+str(to_plot['conn'])+str(to_plot['plus1'])
        bpy.ops.render.render(write_still = True)
    print("after")
```


### 3）复现结果

**仅以1-3-1场景为例，其他场景请读者根据上面的补充说明，自行完成：**

**① 终端输出结果：**

![img](https://img-blog.csdnimg.cn/direct/7bcc2c77dcb241eb88075ed320a90ded.png)

**② prm可视化结果**

![img](https://img-blog.csdnimg.cn/direct/40abe3e02f0343238cb46f7445c554a5.png)

**③ cl****usters可视化结果:** 

# ![img](https://img-blog.csdnimg.cn/direct/1dd1a31e960d418cadffee4eceb2f1e4.png)

**④ connections可视化结果**

# ![img](https://img-blog.csdnimg.cn/direct/3f670e8a957a47a9bee8933aa7978f5e.png)

**⑤ shortened可视化结果**

# ![img](https://img-blog.csdnimg.cn/direct/2d929605b6b245ceb342abac5a4983a8.png)

# 总结

> 本文总结了该项目的环境配置，代码运行以及结果的可视化，并提供了一些问题的解决方案。
