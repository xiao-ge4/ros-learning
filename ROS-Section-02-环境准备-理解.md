---
typora-copy-images-to: md-images
---

# Section 2 环境准备

## 参考文章

完整的 catkin 宏文档
https://docs.ros.org/en/groovy/api/catkin/html/dev_guide/ge

Cmake 
[CMake](https://cmake.org/)

## 文章逻辑

![1663573647435](.\md-images\1663573647435.png)

## 环境部署

**说明配置**

Ubuntu18.04 + ROS Melodic

如果是在树莓派上部署，可以参考这篇文章

[Ubuntu18.04 + 树莓派4B + wifi + 换源 +ssh + 防火墙相关 + mate桌面 + + vnc + ROS Melodic_XiaoGe4的博客-CSDN博客](https://blog.csdn.net/XiaoGe4/article/details/126598552?spm=1001.2014.3001.5502)

### ubuntu 安装

#### 下载镜像

现在 Ubuntu 官网只会放最新的版本，这里可以下载 18.04 的历史版本（其他版本第二个链接）

> 下载18.04 或其他版本的链接
>
> http://cdimage.ubuntu.com/releases/18.04/release/
>
> http://cdimage.ubuntu.com/releases/

#### 将镜像写入U盘

使用 Ventoy 

[超级好用的装机神器——Ventoy - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/137477151)

### 初始化设置

参考博客

> [Ubuntu18.04安装教程_寥廓长空的博客-CSDN博客_ubuntu18.04](https://blog.csdn.net/baidu_36602427/article/details/86548203)

### 换源

直接上指令

> 1. 编辑文件 #你要是熟悉其他编辑器 vi 等都可以
>
> ```shell
> sudo vim /etc/apt/sources.list
> ```
>
> 2. 点击键盘 i 进入 Insert 模式
> 3. 应该有很多个 deb 开头的一行，其他的被 # 注释掉了，删掉未被注释的 deb 下面的 #deb-src 前面的#
> 4. 将现在没被注释的每行  https://到/Ubuntu-ports之间的网址替换成mirrors.aliyun.com  换完之后长这样
>
> ```shell
> #注意：每个 sources.list 文件内容不一定完全一样，根据上面步骤更改，如下只是 样式 示例
> 
> deb http://mirrors.aliyun.com/ubuntu-ports/  bionic main restricted universe multiverse
> deb-src http://mirrors.aliyun.com/ubuntu-ports/ bionic main restricted universe multiverse
> 
> 当然，实际上有很多行
> ```
>
> 5. 然后按键 `Esc` ，输入 `:wq`退出当前编辑
> 6. 输入如下命令，更新软件，同时验证配置成功
>
> ```shell
> sudo apt-get update
> sudo apt-get upgrade
> ```

### 安装ROS

> 1. 设置下载源
>
> ```shell
> #注意：以下三个源下一个即可
> 
> #设置中科大源
> sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
> 
> #设置清华源
> sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
> 
> #设置上海交大源
> sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.sjtug.sjtu.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
> 
> #设置公钥
> sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
> 
> ```
>
> 2. 安装 ROS
>
> ```shell
> #先更新一下
> sudo apt update
> sudo apt upgrade #没有更新可能会影响后面的安装
> 
> sudo apt install ros-melodic-desktop-full
> 包括ROS, rqt, rviz, 机器人通用库, 2D/3D simulators and 2D/3D perception
> 
> #另有如下其他版本
> sudo apt install ros-melodic-desktop
> 包括ROS, rqt, rviz, 和机器人通用库
> 
> sudo apt install ros-melodic-ros-base
> 包括ROS 包, build和通信库。没有GUI工具
> 
> ```
>
> 3. 设置环境变量
>
> ```shell
> echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc #编辑文本
> 
> source ~/.bashrc #执行
> 
> #这个步骤是让你的 bash 终端 以后可以识别 roscore，rosrun等命令
> 
> #如果你用的是 zsh 等其他终端更改一下命令，例如
> echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
> source ~/.zshrc
> 
> ```
>
> 4. 下载必要功能组件
>
> ```shell
> sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
> ```
>
> 5. rosdep init
>
> ```shell
> sudo rosdep init  #来了来了，几乎都会报错的QAQ
> ```
>
> 原因：访问外网 raw.githubusercontent.com ，很难的啦
> 解决：a. 梯子 b.解析服务其地址（访问外网时绕过去）
> 操作：
>
> ```shell
> 1.sudo chmod 777 /etc
> # 此指令为获取etc中的权限
>    2. 在etc中手动创建文件夹
> /etc/ros/rosdep/sources.list.d
>    3. 解析 raw.githubusercontent.com 的服务器地址，工具https://site.ip138.com，打开该网站，根据提示输入要解析的网址，下面就会有地址（选一个）
> 我选的是 185.199.111.133
>    4. sudo vim /etc/hosts
> 将选择的服务器地址、网址加进去，格式如下
> 185.199.111.133 raw.githubusercontent.com
> 保存、退出
>    5. sudo rosdep init 
> 
> 若有报错ERROR: default sources list file already exists: /etc/ros/rosdep/sources.list.d/20-default.list
> Please delete if you wish to re-initialize
> 原因：/etc/ros/rosdep/sources.list.d目录下已经有20-default.list
> 解决：删除它
> 碎碎念：但是根据我刚刚的步骤，应该不会有这个问题
> ```
>
> 6. rosdep update
>
> ```shell
> sudo rosdep update	#可能又又又会报错
> 
> ```
>
> 原因：仍然是 raw.githubusercontent.com 被墙
>
> 解决：
>
> a. 梯子 
>
> b. 多次执行/手机开热点 
>
> c.需要更新的文件下载到本地
>
> 其中：a/b方法 非常的易操作，b 方法不可小觑，网络好的话，就不需要 c 方法那么复杂了
>
> 详细讲讲 c 方法：将更新的文件下载到本地
>
> 1. 先去 github (https://github.com/ros/rosdistro)将需要的包 down 下来，这里有一个小 Trick,因为 github 的服务器都在国外，访问速度较慢，当遇到这种情况时，我会把它导入 Gitee ，这样 访问速度 会快很多（授人以渔，下面有一个章节专门用来介绍如何 Github 转 Gitee）
>    附：我的仓库地址(https://gitee.com/xiaoge4/rosdistro.git)
>
> ```
> 记录存放地址
> 
> ```
>
> 2. 修改 /etc/ros/rosdep/source.list.d/下的20-default.list，将文件中指向raw.githubusercontent.com的url地址全部修改为指向本地文件的地址
>
> ```shell
> sudo vim /etc/ros/rosdep/source.list.d/20-default.list
> 
> 更改后：
> # os-specific listings first
> yaml file://....../rosdep/osx-homebrew.yaml osx
> 
> # generic
> yaml file://....../rosdep/base.yaml
> yaml file://....../rosdep/python.yaml
> yaml file://....../rosdep/ruby.yaml
> gbpdistro file://....../releases/fuerte.yaml fuerte
> 
> # newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
> 
> ```
>
> ​	其中，......就是你的存放地址，例如我放在/home/leisp下，那么我	更改后就应该是
>
> ```shell
> # os-specific listings first
> yaml file:///home/leisp/rosdistro/rosdep/osx-homebrew.yaml osx
> 
> # generic
> yaml file:///home/leisp/rosdistro/rosdep/base.yaml
> yaml file:///home/leisp/rosdistro/rosdep/python.yaml
> yaml file:///home/leisp/rosdistro/rosdep/ruby.yaml
> gbpdistro file:///home/leisp/rosdistro/releases/fuerte.yaml fuerte
> 
> # newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
> ```
>
> 3. 修改` /usr/lib/python2.7/dist-packages/rosdep2/sources_list.py`中的默认的url的地址
>
> ```shell
> 修改后：
> 
> # default file to download with 'init' command in order to bootstrap
> # rosdep
> DEFAULT_SOURCES_LIST_URL = 'file:///home/leisp/rosdistro/rosdep/sources.list.d/20-default.list'
> 
> # seconds to wait before aborting download of rosdep data
> ```
>
> 4. 修改`/usr/lib/python2.7/dist-packages/rosdep2/rep3.py`
>
> ```shell
> 修改后：
> 
> # location of targets file for processing gbpdistro files
> REP3_TARGETS_URL = 'file:///home/xxx/rosdistro/releases/targets.yaml'
> 
> # seconds to wait before aborting download of gbpdistro data
> ```
>
> 5. 修改`/usr/lib/python2.7/dist-packages/rosdistro/__init__.py`
>
> ```shell
> 修改后：
> 
> # index information
> 
> DEFAULT_INDEX_URL = 'file:///home/xxx/rosdistro/index-v4.yaml'
> 
> def get_index_url():
> 
> ```
>
> 6. 最后
>
> ```shell
> sudo rosdep init 
> 
> ```
>
> 7. check --> 小海龟
>
> ```shell
> roscore 
> rosrun turtlesim turtlesim_node #启动节点
> rosrun turtlesim  turtle_teleop_key
> 
> #注意：分别在三个终端输入以上命令
> 
> ```
>
> 8. 给自己鼓个掌吧，接下来我们要开始和硬件打交道了
>    阶段性 胜利！
>    ![1661823097704](C:\Users\22627\AppData\Roaming\Typora\typora-user-images\1661823097704.png)

### Github 转 Gitee

因为 github 的服务器都在国外，访问速度较慢，当遇到这种情况时，我会把它导入 Gitee ，这样 访问速度 会快很多

> 1. 创建仓库 
> 2. 点击“导入已有仓库”
> 3. 添加要导入项目的网址
> 4. 点击创建，仓库生成
> 5. 注意：如果想要仓库开源（所有人可见）
>    在仓库建立之后，点击管理，设置中更改信息

## 工作空间初始化

### 创建

```shell
mkdir -p ~/melodic_ws/src
cd ~/melodic_ws/src
catkin_init_workspace	#初始化

#解释
#1.melodic_ws 工作空间名字可以自己命名 
#2.-p 是因为没有创建melodic_ws/文件夹，就不能创建子文件夹，-p就可以
```

### 编译

```shell
cd ~/melodic_ws
catkin_make
```

### 设置环境变量

```shell
#解释：就是将工作空间的环境变量配置到控制台，在控制台输入ROS中的命令时，Linux知道含义
#有以下几种
source devel/setup.bash #仅仅将变量申明到当前控制窗口，重开一个工作窗口，就不能使用了

vim ~/.bashrc
输入“~/melodic_ws/devel/setup.bash ”在最后一行
source ~/.bashrc #将“~/melodic_ws/devel/setup.bash ”环境变量位置写入变量文件，所有控制窗口都可使用

echo "source ~/melodic/setup.bash" >> ~/.bashrc 
source ~/.bashrc #执行
#快速方法，与上面一样，但是一行命令写入
```

### 检查环境变量

```shell
echo $ROS_PACKAGE_PATH
```

## 工作空间理解

图解

![1663573748883](.\md-images\1663573748883.png)

### 工作空间

#### 包含内容

1. src
   源空间
   软件包（源代码包）
2. build
   编译空间
   CMake 的缓存信息和中间文件
3. devel
   开发空间
   生成的目标文件（包括头文件，动态链接库，静态链接库，可执行文件等）、环境变量
4. install
   安装空间

#### 生成方式

1. 工作空间（可以任意命名）和 src：自己创建
2. build 和 devel 文件夹：catkin_make 命令自动创建
3. install 文件夹：catkin_make install 命令自动创建，几乎不怎么使用，一般不创建

### Package 功能包

1. 创建功能包

   ```shell
   cd ~/catkin_ws/src
   catkin_create_pkg my_pkg rospy rosmsg roscpp
   
   #rospy、rosmsg、roscpp是依赖库，根据业务需求添加
   #后续需要添加其他的可以再配置
   ```

2. 文件结构

   ```shell
   |-- CMakeLists.txt #当前package的编译规则，通常需要为c++代码添加编译时的依赖，执行等操作
   |—— package.xml #package 的描述信息，通常添加一些ros库的支持
   |—— include文件夹 # 存放c++ 头文件的
   |—— config文件夹 # 存放参数配置文件
   |—— launch文件夹 # 存放launch文件(.launch或.xml)
   |—— meshes文件夹 # 存放机器人或仿真场景的3D模型(.sda, .stl, .dae等) ；
   |—— urdf文件夹 # 存放机器人的模型描述(.urdf或.xacro) ；
   |—— rviz文件夹 # rviz文件
   |—— src文件夹 # c++源代码
   |—— scripts文件夹 # 可执行脚本；例如shell脚本(.sh)、Python脚本(.py) ；
   |—— srv文件夹 # 自定义service
   |—— msg文件夹 # 自定义topic
   |—— action文件夹 # 自定义action 动作格式文件
   ```

3. 只有 CMakeLists.txt 和 package.xml 是必须的
   以下重点介绍二者

#### CMakeLists.txt

##### 概述

CMake 的编译风格，但针对 ROS 工程添加了一些宏定义

规定了这个 package 的依赖、编译目标、编译流程

##### 格式

```shell
cmake_minimum_required() # 所需CMake版本
project() # 软件包名称
find_package() # 找到编译需要的其他CMake/Catkin package
catkin_python_setup() # 启用Python模块支持
add_message_files() # message生成器
add_service_files() # service生成器
add_action_files() # action生成器
generate_message() # 生成不同语言版本的msg/srv/action接口
catkin_package() # 生成当前package的cmake配置，供依赖本包的其他软件包调用
add_library() # 用来指定编译产生的库。默认的catkin编译产生共享库。
add_executable() # 生成可执行二进制文件
add_dependencies() # 定义目标文件依赖于其他目标文件，确保其他目标已被构建
target_link_libraries() # 指定可执行文件链接的库。这个要用在add_executable()后面。
catkin_add_gtest() # 测试建立
install() # 安装至本机
```

##### Boost

如果使用 Boost ，需要在Boost上调用 find_package() ，并使用的 Boost 组件

```shell
#eg：使用 Boost 线程
find_package（Boost REQUIRED COMPONENTS thread）
```

##### find_package()

如果CMake通过 `find_package()`查找到一个软件包，它就会创建几个CMake环境变量，以提供有关已查找到的软件包的信息。这些环境变量可以在后面的CMake脚本中使用，它们表示软件包导出的头文件所在的位置、源文件所在的位置、软件包依赖的库以及这些库的查找路径，环境变量的名字遵循`<PACKAGENAME>_<PROPERTY>`，即包名-属性：

- `<NAME>_FOUND`：当库被查找到时置为true，否则为false
- `<NAME>_INCLUDE_DIRS`或`<NAME>_INCLUDES`：软件包导出的头文件路径
- `<NAME>_LIBRARIES`或`<NAME>_LIBS`：软件包导出的库的路径
- `<NAME>_DEFINITIONS`

##### catkin_package()

是一个 catkin 提供的 CMake 宏

在使用 `add_library()` 或 `add_executable()` 声明任何目标之前，必须调用

1. 可选参数

   ```txt
   INCLUDE_DIRS - 包的导出包含路径
   LIBRARIES - 从项目导出的库
   CATKIN_DEPENDS - 该项目依赖的其他 catkin 项目
   DEPENDS - 该项目所依赖的非 catkin CMake项目。
   CFG_EXTRAS - 其他配置选项
   ```

2. 例子

   ```txt
   catkin_package(
   INCLUDE_DIRS include #文件夹“include”是导出头文件的地方
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet #构建/运行此程序包需要依赖 roscpp nodelet 软件包
   DEPENDS eigen opencv) #构建/运行此程序包的系统依赖是 eigen opencv
   ```

##### 包括路径和库路径

```txt
include_directories（<dir1>，<dir2>，...，<dirN>）
#设置头文件的相对路径
eg：使用catkin和Boost
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
```

```
link_directories（<dir1>，<dir2>，...，<dirN>）

#用于添加额外的库路径
#所有catkin和CMake软件包在find_packaged时都会自动添加链接信息，只需链接到 target_link_libraries()中的库
```

##### 可执行目标

eg：

```
add_executable(myProgram src/main.cpp src/some_file.cpp src/another_file.cpp)
```

这将构建一个名为myProgram的目标可执行文件，它由3个源文件构建：src / main.cpp，src / some_file.cpp和src / another_file.cpp

##### 库文件

用来指定库来构建

```
add_library($ {PROJECT_NAME} $ {$ {PROJECT_NAME} _SRCS})
```

##### target_link_libraries

指定可执行目标链接的库

```
target_link_libraries(<executableTargetName>，<lib1>，<lib2>，... <libN>)
```

eg：

```
add_executable(foo src/foo.cpp)
add_library(moo src/moo.cpp)
target_link_libraries(foo moo)
```

##### 消息、服务和动作

```
add_message_files
add_service_files
add_action_files

generate_messages()
```

一个完整的`CMakeLists.txt`

```txt
cmake_minimum_required(VERSION 3.0.2) #所需 cmake 版本
project(demo01_hello_vscode) #包名称，会被 ${PROJECT_NAME} 的方式调用

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#设置构建所需要的软件包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
#默认添加系统依赖
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# 启动 python 模块支持
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
## 声明 ROS 消息、服务、动作... ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# 生成消息、服务时的依赖包
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
## 声明 ROS 动态参数配置 ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
## catkin 特定配置##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# 运行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo01_hello_vscode
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# 添加头文件路径，当前程序包的头文件路径位于其他文件路径之前
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# 声明 C++ 库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/demo01_hello_vscode.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# 添加库的 cmake 目标依赖
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# 声明 C++ 可执行文件
add_executable(Hello_VSCode src/Hello_VSCode.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#重命名c++可执行文件
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#添加可执行文件的 cmake 目标依赖
add_dependencies(Hello_VSCode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#指定库、可执行文件的链接库
target_link_libraries(Hello_VSCode
  ${catkin_LIBRARIES}
)

#############
## Install ##
## 安装 ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#设置用于安装的可执行脚本
catkin_install_python(PROGRAMS
  scripts/Hi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_demo01_hello_vscode.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```

#### package.xml

##### 概述

包含了 package 的名称、版本号、内容描述、维护人员、软件许可、编译构建工具、编译依赖、运行依赖等信息

在较早的ROS版本中，叫做 `manifest.xml` 

##### 格式

```shell
<pacakge> # 根标记文件
<name> # 包名
<version> # 版本号
<description> # 内容描述
<maintainer> # 维护者
<license> # 软件许可证
<buildtool_depend> # 编译构建工具，通常为catkin
<depend> # 指定依赖项为编译、导出、运行需要的依赖，最常用
<build_depend> # 编译依赖项
<build_export_depend> # 导出依赖项
<exec_depend> # 运行依赖项
<test_depend> # 测试用例依赖项
<doc_depend> # 文档依赖项
```

##### 依赖关系

构建依赖关系<build_depend>

> 指定构建此包所需的包

构建导出依赖关系<build_export_depend>

> 根据此包构建库所需的包

执行依赖关系<exec_depend>

> 在此程序包中运行代码所需的软件包

测试依赖关系<test_depend>

> 指定单元测试的附加依赖项

构建工具依赖关系<buildtool_depend>

> 此软件包需要构建自身的构建系统工具

文档工具依赖关系<doc_depend>

> 此软件包需要生成文档的文档工具

##### 一个完整的`package.xml`

```xml
<?xml version="1.0"?>
<!-- 格式: 以前是 1，推荐使用格式 2 -->
<package format="2">
  <!-- 包名 -->
  <name>demo01_hello_vscode</name>
  <!-- 版本 -->
  <version>0.0.0</version>
  <!-- 描述信息 -->
  <description>The demo01_hello_vscode package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <!-- 维护人员 -->
  <maintainer email="xuzuo@todo.todo">xuzuo</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <!-- 许可证信息，ROS核心组件默认 BSD -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/demo01_hello_vscode</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <!-- 依赖的构建工具，这是必须的 -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- 指定构建此软件包所需的软件包 -->
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <!-- 指定根据这个包构建库所需要的包 -->
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <!-- 运行该程序包中的代码所需的程序包 -->  
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

