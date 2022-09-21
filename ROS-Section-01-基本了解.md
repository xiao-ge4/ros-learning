---
typora-copy-images-to: md-images
---

## Section 1  基本理解

### 三个层面

#### 计算图

目的：描述程序如何运行的，进程与进程、进程与系统之间的通讯

解说：创建一个连接到所有进程的网络，在系统中的任何节点都可以访问此网络，并通过网络与其他节点交互，获得其他节点发布的信息，并将自身数据发布到网络。

基本概念：

> ![1663573368999](.\md-images\1663573368999.png)

##### 节点 Node

计算执行进程，类似一个函数，拥有特定功能

1. 查看节点

命令行输入`rosnode`
![1663573406120](.\md-images\1663573406120.png)

命令作用

| rosnode命令            | 作用                                 |
| ---------------------- | ------------------------------------ |
| rosnode list           | 查询当前运行的所有节点               |
| rosnode info node_name | 显示该节点的详细信息                 |
| rosnode kill node_name | 结束某个节点                         |
| rosnode ping           | 测试该节点是否存活                   |
| rosnode machine        | 列出在特定机器或列表机器上运行的节点 |
| rosnode cleanup        | 清除不可运行节点的注册信息           |

2. 编写节点

节点需要 roscpp 、rospy对既节点进行编写

##### 节点管理器 ROS Master

控制中心，提供参数管理（节点的名称注册、查找）

在分布式系统中，节点管理器仅运行在某一台电脑上，而其它节点通过与节点管理器通信来找到彼此

##### 消息 Message

目的：节点之间通过消息实现彼此数据交换。

格式：支持一些标准的数据类型（整数、浮点数、布尔类型等），还可以利用标准数据类型-->建造自己需要的数据类型

1. 查看消息

命令行输入`rosmsg`

![1663573439373](.\md-images\1663573439373.png)

| rosmsg命令      | 作用                       |
| --------------- | -------------------------- |
| rosmsg show     | 显示一条消息字段           |
| rosmsg list     | 列出所有消息               |
| rosmsg package  | 列出所有功能包消息         |
| rosmsg packages | 列出所有具有该消息的功能包 |
| rosmsg md5      | 显示一条消息的MD5检验值    |

##### 话题 Topic

目的：发布/订阅消息，异步通信

传输方式：TCP/IP(TCPROS) ；UDPROS 后者低延迟、非可靠

1. 查看话题

命令行输入`rostopic`

![1663573459883](.\md-images\1663573459883.png)

| rostopic命令                  | 作用                       |
| ----------------------------- | -------------------------- |
| rostopic bw /topic            | 显示主题所使用的带宽       |
| rostopic echo /topic          | 将主题对应的消息输出到屏幕 |
| rostopic find message_type    | 按照类型查找主题           |
| rostopic hz /topic            | 显示主题的发布频率         |
| rostopic info /topic          | 输出关于该主题的信息       |
| rostopic list                 | 输出活动主题列表           |
| rostopic pub /topic type args | 将数据发布到主题           |
| rostopic type /topic          | 输出主题类型               |

##### 服务 Service

目的：请求／应答式，同步通信

场景：快速的

1. 查看服务

命令行输入`rosservice`

![1663573480791](.\md-images\1663573480791.png)

| rosservice 命令          | 作用                 |
| ------------------------ | -------------------- |
| rosservice args /service | 显示服务参数         |
| rosservice call /servic  | 用输入的参数请求服务 |
| rosservice find /service | 按照类型查找主题     |
| rosservice info /service | 显示指定服务的信息   |
| rosservice list          | 显示活动的服务信息   |
| rosservice uri /service  | 显示ROSRPC URI服务   |
| rosservice type /service | 显示服务类型         |

##### 消息记录包

目的：保存和回放ROS消息数据的文件格式，保存在.bag文件中

1. 查看消息记录

命令行输入`rosbag`

![1663573496285](.\md-images\1663573496285.png)

| rosbag命令 | 作用                                                 |
| ---------- | ---------------------------------------------------- |
| check      | 确定一个包是否可以在当前系统中进行，或者是否可以迁移 |
| decompress | 压缩一个或多个包文件                                 |
| filter     | 解压一个或多个包文件                                 |
| fix        | 在包文件中修复消息，以便在当前系统中播放             |
| help       | 获取相关命令指引帮助信息                             |
| info       | 总结一个或多个包文件的内容                           |
| play       | 以一种时间同步的方式回放一个或多个包文件的内容       |
| record     | 用指定话题的内容记录一个包文件                       |
| reindex    | 重新索引一个或多个包文件                             |

##### 参数服务器 Parameter Server

目的：通过网络访问的、共享的多变量字典，通过关键字存储在节点管理器上

1. 查看消息记录

命令行输入`rosparam`

![1663573507601](.\md-images\1663573507601.png)

| rosparam命令                 | 作用                           |
| ---------------------------- | ------------------------------ |
| rosparam delete parameter    | 删除参数                       |
| rosparam dump file           | 将参数服务器中的参数写入到文件 |
| rosparam get parameter       | 获得参数值                     |
| rosparam list                | 列出参数服务器中的参数         |
| rosparam load file           | 从文件中加载参数到参数服务器   |
| rosparam set parameter value | 设置参数                       |

#### 文件系统

目的：程序文件是如何组织与构建的

上图

> ![1663573528800](.\md-images\1663573528800.png)

##### 综合功能包 Meta Packages

又名：元功能包
组织多个用于同一目的功能包

##### 元功能包清单 Meta Packages

包含：运行时的功能包依赖、引用的标签

##### 功能包 Package

ROS系统中软件组织的基本形式，包含：运行的节点、配置文件等

ROS package 相关命令

| rospack 命令              | 作用                      |
| ------------------------- | ------------------------- |
| rospack help              | 显示rospack的用法         |
| rospack list              | 列出本机所有package       |
| rospack depends [package] | 显示package的依赖包       |
| rospack find [package]    | 定位某个package           |
| rospack profile           | 刷新所有package的位置记录 |

##### 功能包清单 Package manifest

package.xml
功能包的基本信息：作者信息、许可信息、依赖关系、编译标志

##### 消息类型 Message

c文件夹 .msg文件
消息是ROS节点之间发布／订阅的通信信息数据类型，包括：自定义 + 系统自带

##### 服务类型 Service

srv 文件夹 .srv文件
ROS 服务器／客户端通信模型下的请求与应答数据类型，包括：自定义 + 系统自带

##### 代码 Code

src 文件夹 .src文件
放置功能包节点源代码的文件夹

#### 开源社区

ROS 资源是分布式管理，开发人员知识、代码、算法共享

1. 发行版（Distribution）：ROS发行版包括一系列 带有版本号、可以直接安装的功能包。
2. 软件源（Repository）：ROS依赖于共享网络上 的开源代码，不同的组织机构可以开发或者共享自己的机器人软件。
3. ROS wiki：记录ROS信息文档的主要论坛。
   http://wiki.ros.org
4. 邮件列表（Mailing list）：交流ROS更新的主要 渠道，同时也可以交流ROS开发的各种疑问。
5. Bug提交系统（Bug Ticket System）：如果你发现问题或者想提出一个新功能，ROS提供这个资源去做这些
6. ROS Answers：咨询ROS相关问题的网站。
   https://answers.ros.org
7. 博客（Blog）：发布ROS社区中的新闻、图片、视频
   [古月居 - ROS机器人知识分享社区 (guyuehome.com)](https://www.guyuehome.com/author/guyue)
   [ CSDN博客-专业IT技术发表平台](https://blog.csdn.net/)等

### 特点

1. 一个目标：提高机器人研发的软件复用率
2. 五个特点：点对点、多语言、集成化、组件化、开源
3. 四位一体：通讯机制、开发工具、应用功能、生态系统

