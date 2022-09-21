# Section 4 关键组件

## 常见命令

接下来常用到的指令

| 命令             | 作用                       |
| ---------------- | -------------------------- |
| catin_create_pkg | 创建功能包的信息           |
| rospack          | 获取功能包的信息           |
| catkin_make      | 编译工作空间中的功能包     |
| rosdep           | 自动安装功能包依赖的其他包 |
| roscd            | 功能包目录跳转             |
| roscp            | 拷贝功能包中的文件         |
| rosed            | 编辑功能包中的文件         |
| rosrun           | 运行功能包中的可执行文件   |
| roslaunch        | 运行启动文件               |

## 启动节点方式

### launch 文件

1. 借助 ros package 路径启动

   ```shell
   roslaunch package名称 launch文件名称
   ```

2. 直接给出 launch 文件的绝对路径

   ```shell
   roslaunch path_to_launchfile
   ```

添加参数

1. `--screen` 
   ros node 的信息输出到屏幕上，而不是保存在某个 log 文件中，方便调试

2. `arg:=value `
   给 launch 文件中待赋值的变量赋值

   ```shell
   roslaunch pkg_name launchfile_name model:='$(find urdf_pkg)/urdf/myfile.urdf' 
   
   #给 model 变量赋值
   #用 find 命令提供路径
   ```

注意

1. launch 文件不需要编译
2. roslaunch 运行时先检查 rosmaster 是否运行，未运行则先启动 rosmaster 

### rosrun

rosrun 方法每次只能运行一个节点

rosrun 会寻找 package 的名为 executable 的可执行程序，将可选参数ARGS传入

1. 先启动 rosmaster 

   ```shell
   roscore
   ```

2. 启动 node 

   ```shell
   rosrun [--prefix cmd] [--debug] pkg_name node_name [ARGS]
   ```

## Launch

### 参考来源

http://wiki.ros.org/roslaunch/XML

### 概述

目的：通过 launch文件以及 roslaunch 命令可以一次性启动多个 node

### 格式

1. 一种xml 文件

2. 主要的`tag`

   ```xml
   <launch> <!--根标签-->
   <node> <!--需要启动的node及其参数-->
   <include> <!--包含其他launch-->
   <machine> <!--指定运行的机器-->
   <env-loader> <!--设置环境变量-->
   <param> <!--定义参数到参数服务器-->
   <rosparam> <!--加载yaml文件中的参数到参数服务器-->
   <arg> <!--定义变量-->
   <remap> <!--设定topic映射-->
   <group> <!--设定分组-->
   </launch> <!--根标签-->
   ```

接下来详解每种`tag`

#### node

```xml
<launch>
	<node pkg="package_name" type="executable_file" name="node_name"/>
	<node pkg="another_package" type="another_executable" name="another_node"></node>
...
</launch
```

`pkg` 节点所在功能包的名字

`type`功能包中可执行文件的名字

`name`节点启动后的名字

还可以设置更多参数

```xml
<launch>
	<node
		pkg=""
		type=""
		name=""
		respawn="true"
		required="true"
		launch-prefix="xterm -e"
		output="screen"
		ns="namespace"
	/>
</launch>
```

`respawn`若该节点关闭，是否自动重新启动

`required`若该节点关闭，是否关闭其他所有节点

`launch-prefix`是否新开一个窗口执行（不希望与其他 node 信息混杂）

`output`默认：launch 启动 node 的信息会存入/.ros/log/ 的 log 文件中，此参数设置，令信息显示在屏幕上

`ns`将 node 归入不同的 namespace

#### remap

修改 topic，抽象化，方便同一个 node 文件被应用到不同的环境中

from:原命名 ；to:映射之后的命名

```xml
<node pkg="some" type="some" name="some">
	<remap from="origin" to="new" />
</node>
```

#### include

将另外一个 launch 文件添加到本 launch 文件中

```xml
<include file="path-to-launch-file" />
```

可移植性更高：

```xml
<include file="$(find package-name)/launch-file-name" />
```

将引入的 node 统一命名

```xml
<include file="$(find package-name)/launch-file-name " ns="my" />
```

#### arg

使参数重复使用，便于多处同时修改

launch文件内部的局部变量，仅限于launch使用

声明一个`arg`，暂未赋值，后可以通过`include` tag 赋值

```xml
<arg name="foo"> 
```
赋默认值
```xml
<arg name="foo" default="1">
```
赋固定值
```xml
<arg name="foo" value="1">
```

命令行赋值

```shell
roslaunch package_name file_name.launch arg1:=value1 arg2:=value2
```

#### 变量替换

1. 直接设置值

   ```xml
   $(find pkg) 
   ```

2. 先设置默认值，如果没有额外的赋值，就用这个默认值了

   ```xml
   $(arg arg_name)
   ```

   若有额外赋值，命令行

   ```shell
   roslaunch package_name file_name.launch arg:=value
   ```

#### param

与 arg 区别开 

param 是共享的（设置ROS系统运行中的参数，存储在参数服务器中），取值不仅限于value，还可以是文件、命令

1. 格式

   ```xml
   <param name="param_name" type="type1" value="val"/>
   # type可以省略，系统自动判断
   <param name="param_name" textfile="$(find pkg)/path/file"/>
   # 读取 file 存成 string
   <param name="param_name" command="$(find pkg)/exe '$(find pkg)/arg.txt'"/>
   实例：
   <param name="param" type="yaml" command="cat '$(find pkg)/*.yaml'"/>
   # command 的结果存在 param 中
   ```

2. 范围
   全局的：name 就是原本的 name

   ```xml
   <param name="publish_frequency" type="double" value="10.0" />
   ```

   局部的：node/param 

   ```xml
   <node name="node1" pkg="pkg1" type="exe1">
   	<param name="param1" value="False"/>
   </node>
   ```

3. 查看` rosparam list`
   以上定义，会显示如下

   ```shell
   /publish_frequency
   /node1/param1 # 自动加上了namespace 前缀
   ```

#### rosparam

加载参数文件多个参数

1. `load`从 YAML 文件中加载一批 param

   ```xml
   <rosparam command="load" file="$(find rosparam)/example.yaml" />
   ```

2. `delete`删除某个 param

   ```xml
   <rosparam command="delete" param="my_param" />
   ```

3. 赋值操作

   ```xml
   <rosparam param="my_param">[1,2,3,4]</rosparam>
   或者
   <rosparam>
   a: 1
   b: 2
   </rosparam>
   ```

#### group

要对多个node进行同样的设置

```xml
<group ns="rosbot">
<remap from="chatter" to="talker"/> # 对该 group 中后续所有 node 都有效
<node ... />
<node ... >
<remap from="chatter" to="talker1"/> # 各个 node 中可以重新设置 remap
</node>
</group>
```

## TF 坐标变换

实时缓冲的树结构中的坐标帧之间的关系

### 常见坐标系

世界坐标 map 

里程计坐标系 odom

基座标 base_link

### TF 常用工具

1. view_frames
   监听当前时刻所有ROS广播的tf坐标系，并绘制出树状图表示坐标系之间的连接关系，生成 frame.pdf 文件，保存到本地当前位置

   ```shell
   rosrun tf view_frames
   ```

2. .rqt_tf_tree
   实时刷新显示坐标系关系

   ```shell
   rosrun rqt_tf_tree rqt_tf_tree
   ```

3. tf_echo
   查看两个特定参考系之间的关系

   ```shell
   rosrun tf tf_echo <source_frame> <target_frame>
   ```

4. static_transform_publisher
   发布两个坐标系之间的静态坐标变换

   ```shell
   
   ```

5. .roswtf plugin
   分析你当前的tf配置并试图找出常见问题

   ```shell
   roswtf
   ```

### 广播/监听

#### 广播器

1. 定义TF广播器(TransformBroadcaster)
2. 创建坐标变换值
3. 发布坐标板换（sendTransform）

#### 监听器

1. 定义监听器（TransformListener）
2. 查找坐标变换( waitForTransform\lookupTransform)

#### 编译代码

## QT 工具箱

### 常用工具

1. rqt_graph 计算图可视化

   ```shell
   rosrun rqt_graph rqt_graph
   ```

2. rqt_topic 查看话题

   ```shell
   rosrun rqt_topic rqt_topic
   ```

3. rqt_publisher
   一个GUI插件，用于发布具有固定或计算字段值的消息

   ```shell
   rosrun rqt_publisher rqt_publisher
   ```

4. rqt_plot 数据绘图

   ```shell
   rosrun rqt_plot rqt_plot
   ```

5. rqt_console 日志输出
   日志消息按照严重性由低到高可以分为 5 级

   | 等级  | 解析                           |
   | ----- | ------------------------------ |
   | DEBUG | 调试日志，供开发测试使用       |
   | INFO  | 常规日志，用户可见级别的信息   |
   | WARN  | 警告信息                       |
   | ERROR | 错误信息，程序出错后打印的信息 |
   | FATAL | 致命错误，出现宕机的日志记录   |

   ```shell
   rosrun rqt_console rqt_console
   ```

6. rqt_reconfigure 动态参数配置

   ```shell
   rosrun rqt_reconfigure rqt_reconfigure
   ```

## Rviz 

ros自带图形化工具

1. 3D视图区，用于可视化显示数据，目前没有任何数据，所以显示黑色。
2. 工具栏，提供视角控制、目标设置、发布地点等工具。
3. 显示项列表，用于显示当前选择的显示插件，可以配置每个插件的属性。
4. 视角设置区，可以选择多种观测视角。
5. 时间显示区，显示当前的系统时间和ROS时间。







