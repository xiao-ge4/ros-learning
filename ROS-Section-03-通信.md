---
typora-copy-images-to: md-images
---

# Section 3 通信

## 通信编程

### 话题编程

#### 流程

创建发布者

创建订阅者

添加编译选项

运行可执行程序

#### 编程实现

1. 创建功能包

在 工作空间/src 下创建 talker/ （名字可自取）

再在 talker/ 下创建 src/ 这个文件夹下会放 talker/ package 的所有源码

`catkin_create_pkg talker std_msgs rospy roscpp geometry_msgs turtlesim`

2. 编译功能包

在 工作空间下/ `catkin_make` 

##### 发布者

1. 导包（包括自定义 msg ）
2. 初始化ROS节点,命名(唯一)
3. 向ROS Master注册节点信息，包括发布的话题名和话题中的消息类型
4. 组织被发布的数据
5. 按照一定的频率循环发布消息

###### C++ 实现

在 工作空间/src/talker/src 下创建 talker.cpp （也可以用其他语言）

```c++
#include <sstream> 		//加载字符串流
#include "ros/ros.h"	//实用头文件，引用了ROS系统中大部分常用的头文件
#include "std_msgs/String.h" //引用了std_msgs/String 消息存放在std_msgs package里，是由String.msg文件自动生成的头文件

int main(int argc,char **argv)
{
	//R0s节点初始化，命名
	ros::init (argc,argv,"talker");
	//创节点句柄
    //第一个创建的 NodeHandle 会为节点进行初始化，最后一个销毁的会清理节点使用的所有资源
	ros::NodeHandle n;
	//创建一个Pub1isher,发布名为chatter的topic,消息类型为std_msgs:String，第二个参数是发布序列大小（发布数据太快,缓冲区中的消息在大于1000个的时候就会开始丢弃先前发布的消息）
    //advertise()发布函数：告诉 master 我将在chatter topic上发布一个std_msgs/String的消息
	ros::Publisher chatter_pub=n.advertise<std_msgs::String>("chatter",1000);
	//设置自循环的频率
    //它会追踪记录自上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间
	ros::Rate loop_rate(10);

	int count =0
	while (ros::ok())
	{
		//初始化std_msgs::String类型的消息
		std_msgs::String msg;
		std::stringstream ss;
		ss<<"hello world"<<count;
		msg.data=ss.str();

		//打印消息
		ROS_INFO("%s",msg.data.c_str());
		//发布小徐
        chatter_pub.publish (msg);
		
        //循环等待回调函数
		ros:spinOnce();
		//按照循环颜率延时
		loop_rate.sleep();
		++count;
	}

return 0;
    
//ros::ok()返回false，如果下列条件之一发生
//SIGINT接收到( Ctrl+C )
//被另一同名节点踢出ROS网络
//ros::shutdown()被程序的另一部分调用
//所有的ros::NodeHandles都已经被销毁
```

###### python 实现

```python
#! /usr/bin/env python
#这里我常常用环境的绝对路径，实际使用会设计到多个 python 环境，懒得一直更换
"""
    消息发布方:
        循环发布信息:HelloWorld 后缀数字编号
"""
#1.导包 
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("talker_p")
    #3.实例化 发布者 对象
    pub = rospy.Publisher("chatter",String,queue_size=10)
    #4.组织被发布的数据，并编写逻辑发布数据
    msg = String()  #创建 msg 对象
    msg_front = "hello 你好"
    count = 0  #计数器 
    # 设置循环频率
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        #拼接字符串
        msg.data = msg_front + str(count)

        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("写出的数据:%s",msg.data)
        count += 1
```

##### 订阅者

1. 导包 
2. 初始化ROS节点,命名(唯一)
3. 订阅需要的话题
4. 循环等待话题消息，接收到消息后进入回调函数
5. 在回调函数中完成消息处理
6. 再次进入循环等待消息

###### C++实现

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

//接收到订阅的消息后，会进入消息回调函数
void chatterCallback(const std_msgs::String:ConstPtr&msg)
{
	//将接收到的消息打印出来
	ROS INFO("I heard:[s]",msg->data.c str());
}
int main(int argc,char **argv)
{
	//初始化ROs节点
	ros::init (argc,argv,"listener");
	//创建节点句柄
	ros::NodeHandle n;
	//创建一个Subscriber,订阅名为chatter的topic,注册回调函数chatterCallback
	ros:Subscriber sub=n.subscribe("chatter",1000,chatterCallback);
	//循环等待回调函数
	ros::spin();
	return 0;
}
```

###### python实现

```python
#! /usr/bin/env python
"""
    消息订阅方:
        订阅话题并打印接收到的消息
"""
#1.导包 
import rospy
from std_msgs.msg import String

def doMsg(msg):
    rospy.loginfo("I heard:%s",msg.data)

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("listener_p")
    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("chatter",String,doMsg,queue_size=10)
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    rospy.spin()
```

##### 编译

1. 设置需要编译的代码和生成的可执行文件-->add_executable

在 `CMakeLists.txt`文件末尾添加如下内容

```txt
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
```

在工作空间下 `catkin_make`

生成两个可执行文件， talker 和 listener, 默认存储到 devel/（`~/工作空间/devel/lib/`）

2. 设置链接库
3. 设置依赖



在 工作空间/ 下`catkin_make`

##### 运行可执行文件

1. roscore
2. rosrun <learning_communication> talker
3. rosrun <learning_communication> listener

##### 自定义话题消息

在 工作空间/src/package/ 目录下，新建文件夹 msg

```shell
eg：

cd ~/melodic_ws/src/talker/
mkdir msg
```

1. 添加 msg 文件
   在 msg 文件夹下，添加文件`test.msg`

   ```
   string name
   uint16 age
   float64 height
   ```

2. 在package.xml中添加功能包依赖

   ```xml
     <build_depend>message_generation</build_depend>
     <exec_depend>message_runtime</exec_depend>
     <!-- 
     exce_depend 以前对应的是 run_depend 现在非法
     -->
   ```

3. 在CMakeLists.txt添加编译选项

   ```
   find_package(catkin REQUIRED COMPONENTS
     roscpp
     rospy
     std_msgs
     message_generation
   )
   # 需要加入 message_generation,必须有 std_msgs
   
   ## 配置 msg 源文件
   add_message_files(
     FILES
     Person.msg
   )
   
   # 生成消息时依赖于 std_msgs
   generate_messages(
     DEPENDENCIES
     std_msgs
   )
   
   #执行时依赖
   catkin_package(
   #  INCLUDE_DIRS include
   #  LIBRARIES demo02_talker_listener
     CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
   #  DEPENDS system_lib
   )
   ```

4. 编译生成语言相关文件

   

###### C++ 实现

1. 发布方

   ```cpp
   #include "ros/ros.h"
   #include "demo02_talker_listener/Person.h"
   
   int main(int argc, char *argv[])
   {
       setlocale(LC_ALL,"");
   
       //1.初始化 ROS 节点
       ros::init(argc,argv,"talker_person");
   
       //2.创建 ROS 句柄
       ros::NodeHandle nh;
   
       //3.创建发布者对象
       ros::Publisher pub = nh.advertise<demo02_talker_listener::Person>("chatter_person",1000);
   
       //4.组织被发布的消息，编写发布逻辑并发布消息
       demo02_talker_listener::Person p;
       p.name = "sunwukong";
       p.age = 2000;
       p.height = 1.45;
   
       ros::Rate r(1);
       while (ros::ok())
       {
           pub.publish(p);
           p.age += 1;
           ROS_INFO("我叫:%s,今年%d岁,高%.2f米", p.name.c_str(), p.age, p.height);
   
           r.sleep();
           ros::spinOnce();
       }
   
   
   
       return 0;
   }
   ```

2. 订阅方

   ```cpp
   #include "ros/ros.h"
   #include "demo02_talker_listener/Person.h"
   
   void doPerson(const demo02_talker_listener::Person::ConstPtr& person_p){
       ROS_INFO("订阅的人信息:%s, %d, %.2f", person_p->name.c_str(), person_p->age, person_p->height);
   }
   
   int main(int argc, char *argv[])
   {   
       setlocale(LC_ALL,"");
   
       //1.初始化 ROS 节点
       ros::init(argc,argv,"listener_person");
       //2.创建 ROS 句柄
       ros::NodeHandle nh;
       //3.创建订阅对象
       ros::Subscriber sub = nh.subscribe<demo02_talker_listener::Person>("chatter_person",10,doPerson);
   
       //4.回调函数中处理 person
   
       //5.ros::spin();
       ros::spin();    
       return 0;
   }
   ```

3. 配置 CMakeLists.txt

   ```
   add_executable(person_talker src/person_talker.cpp)
   add_executable(person_listener src/person_listener.cpp)
   
   
   
   add_dependencies(person_talker ${PROJECT_NAME}_generate_messages_cpp)
   add_dependencies(person_listener ${PROJECT_NAME}_generate_messages_cpp)
   
   
   target_link_libraries(person_talker
     ${catkin_LIBRARIES}
   )
   target_link_libraries(person_listener
     ${catkin_LIBRARIES}
   )
   ```

4. 执行

   > 1. 启动 roscore;
   >
   > 2. 启动发布节点;
   >
   > 3. 启动订阅节点。

###### python 实现

1. 发布方

   ```python
   #! /usr/bin/env python
   
   import rospy
   from demo02_talker_listener.msg import Person
   
   
   if __name__ == "__main__":
       #1.初始化 ROS 节点
       rospy.init_node("talker_person_p")
       #2.创建发布者对象
       pub = rospy.Publisher("chatter_person",Person,queue_size=10)
       #3.组织消息
       p = Person()
       p.name = "葫芦瓦"
       p.age = 18
       p.height = 0.75
   
       #4.编写消息发布逻辑
       rate = rospy.Rate(1)
       while not rospy.is_shutdown():
           pub.publish(p)  #发布消息
           rate.sleep()  #休眠
           rospy.loginfo("姓名:%s, 年龄:%d, 身高:%.2f",p.name, p.age, p.height)
   ```

2. 订阅方

   ```python
   #! /usr/bin/env python
   
   import rospy
   from demo02_talker_listener.msg import Person
   
   def doPerson(p):
       rospy.loginfo("接收到的人的信息:%s, %d, %.2f",p.name, p.age, p.height)
   
   
   if __name__ == "__main__":
       #1.初始化节点
       rospy.init_node("listener_person_p")
       #2.创建订阅者对象
       sub = rospy.Subscriber("chatter_person",Person,doPerson,queue_size=10)
       rospy.spin() #4.循环
   
   ```

3. 增加权限

   ```shell
   chmod +x *.py
   ```

4. 配置 CMakeLists.txt

   ```
   catkin_install_python(PROGRAMS
     scripts/talker_p.py
     scripts/listener_p.py
     scripts/person_talker.py
     scripts/person_listener.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

5. 执行

### 服务编程

#### 模型

1. Server注册
2. Client注册
3. ROS Master实现信息匹配
4. Client发送请求
5. Server发送响应

#### 编程实现

1. 创建功能包
   在`~/工作空间/src/`下创建功能包

   ```shell
   catkin_create_pkg learning_server std_msgs rospy roscpp geometry_msgs turtlesim
   ```

2. 编译功能包

   ```shell
   catkin_make
   ```

##### 服务通信自定义srv

1. 按照固定格式创建srv文件
   数据分成两部分，请求与响应，在 srv 文件中请求和响应使用`---`分割

   ```
   # 客户端请求时发送的两个数字
   int32 num1
   int32 num2
   ---
   # 服务器响应发送的数据
   int32 sum
   ```

2. 编辑配置文件

   1. package.xml

      ```xml
        <build_depend>message_generation</build_depend>
        <exec_depend>message_runtime</exec_depend>
        <!-- 
        exce_depend 以前对应的是 run_depend 现在非法
        -->
      ```

   2. CMakeLists.txt

      ```
      find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
      )
      # 需要加入 message_generation,必须有 std_msgs
      
      add_service_files(
        FILES
        AddInts.srv
      )
      
      generate_messages(
        DEPENDENCIES
        std_msgs
      )
      ```

3. 编译

##### 创建服务端

###### c++

在 `~/工作空间/src/learning_server/src`下新建`server.cpp`

1. 包含头文件
2. 初始化ROS节点
3. 创建句柄
4. 创建 服务 对象
5. 回调函数处理请求并产生响应
6. 由于请求有多个，需要调用 ros::spin() 进入循环

```cpp
#include "ros/ros.h"
#include "demo03_server_client/AddInts.h"

// bool 返回值由于标志是否处理成功
bool doReq(demo03_server_client::AddInts::Request& req,
          demo03_server_client::AddInts::Response& resp){
    int num1 = req.num1;
    int num2 = req.num2;

    ROS_INFO("服务器接收到的请求数据为:num1 = %d, num2 = %d",num1, num2);

    //逻辑处理
    if (num1 < 0 || num2 < 0)
    {
        ROS_ERROR("提交的数据异常:数据不可以为负数");
        return false;
    }

    //如果没有异常，那么相加并将结果赋值给 resp
    resp.sum = num1 + num2;
    return true;


}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"AddInts_Server");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 服务 对象
    ros::ServiceServer server = nh.advertiseService("AddInts",doReq);
    ROS_INFO("服务已经启动....");
    //     5.回调函数处理请求并产生响应
    //     6.由于请求有多个，需要调用 ros::spin()
    ros::spin();
    return 0;
}
```

###### python

1. 导包
2. 初始化 ROS 节点
3. 创建服务对象
4. 回调函数处理请求并产生响应
5. spin 函数

```python
# 1.导包
import rospy
from demo03_server_client.srv import AddInts,AddIntsRequest,AddIntsResponse
# 回调函数的参数是请求对象，返回值是响应对象
def doReq(req):
    # 解析提交的数据
    sum = req.num1 + req.num2
    rospy.loginfo("提交的数据:num1 = %d, num2 = %d, sum = %d",req.num1, req.num2, sum)

    # 创建响应对象，赋值并返回
    # resp = AddIntsResponse()
    # resp.sum = sum
    resp = AddIntsResponse(sum)
    return resp


if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("addints_server_p")
    # 3.创建服务对象
    server = rospy.Service("AddInts",AddInts,doReq)
    # 4.回调函数处理请求并产生响应
    # 5.spin 函数
    rospy.spin()
```

##### 创建客户端

1. 包含头文件
2. 初始化 ROS 节点
3. 创建 ROS 句柄
4. 创建 客户端 对象
5. 请求服务接收响应

```c++
#include "ros/ros.h"
#include "learning communication/AddTwoInts.h"

// 1.包含头文件
#include "ros/ros.h"
#include "demo03_server_client/AddInts.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    // 调用时动态传值,如果通过 launch 的 args 传参，需要传递的参数个数 +3
    if (argc != 3)
    // if (argc != 5)//launch 传参(0-文件路径 1传入的参数 2传入的参数 3节点名称 4日志路径)
    {
        ROS_ERROR("请提交两个整数");
        return 1;
    }


    // 2.初始化 ROS 节点
    ros::init(argc,argv,"AddInts_Client");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 客户端 对象
    ros::ServiceClient client = nh.serviceClient<demo03_server_client::AddInts>("AddInts");
    //等待服务启动成功
    //方式1
    ros::service::waitForService("AddInts");
    //方式2
    // client.waitForExistence();
    // 5.组织请求数据
    demo03_server_client::AddInts ai;
    ai.request.num1 = atoi(argv[1]);
    ai.request.num2 = atoi(argv[2]);
    // 6.发送请求,返回 bool 值，标记是否成功
    bool flag = client.call(ai);
    // 7.处理响应
    if (flag)
    {
        ROS_INFO("请求正常处理,响应结果:%d",ai.response.sum);
    }
    else
    {
        ROS_ERROR("请求处理失败....");
        return 1;
    }

    return 0;
}
```

###### python

1. 导包
2. 初始化 ROS 节点
3. 创建请求对象
4. 发送请求
5. 接收并处理响应

```python
#! /usr/bin/env python


#1.导包
import rospy
from demo03_server_client.srv import *
import sys

if __name__ == "__main__":

    #优化实现
    if len(sys.argv) != 3:
        rospy.logerr("请正确提交参数")
        sys.exit(1)


    # 2.初始化 ROS 节点
    rospy.init_node("AddInts_Client_p")
    # 3.创建请求对象
    client = rospy.ServiceProxy("AddInts",AddInts)
    # 请求前，等待服务已经就绪
    # 方式1:
    # rospy.wait_for_service("AddInts")
    # 方式2
    client.wait_for_service()
    # 4.发送请求,接收并处理响应
    # 方式1
    # resp = client(3,4)
    # 方式2
    # resp = client(AddIntsRequest(1,5))
    # 方式3
    req = AddIntsRequest()
    # req.num1 = 100
    # req.num2 = 200 

    #优化
    req.num1 = int(sys.argv[1])
    req.num2 = int(sys.argv[2]) 

    resp = client.call(req)
    rospy.loginfo("响应结果:%d",resp.sum)

```

设置权限

```shell
chmod +x *.py
```

##### 配置文件

CMakeLists.txt

###### C++

```
add_executable(AddInts_Server src/AddInts_Server.cpp)
add_executable(AddInts_Client src/AddInts_Client.cpp)


add_dependencies(AddInts_Server ${PROJECT_NAME}_gencpp)
add_dependencies(AddInts_Client ${PROJECT_NAME}_gencpp)


target_link_libraries(AddInts_Server
  ${catkin_LIBRARIES}
)
target_link_libraries(AddInts_Client
  ${catkin_LIBRARIES}
)
```

###### python

```
catkin_install_python(PROGRAMS
  scripts/AddInts_Server_p.py 
  scripts/AddInts_Client_p.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

##### 运行

###### C++

1. rosrun 包名 服务

2. rosrun 包名 客户端 参数1 参数2

3. 优化

   >在客户端发送请求前添加:`client.waitForExistence();`
   >
   >或:`ros::service::waitForService("AddInts");`
   >
   >这是一个阻塞式函数，只有服务启动成功后才会继续执行

###### python

1. rosrun 包名 服务
2. rosrun 包名 客户端 参数1 参数2

### 动作编程

## 分布式通信

ROS是一种**分布式**框架，节点之间通过**松耦合**的方式进行组合

1. 设置ip地址，确保底层链路的联通
   查询ip地址 ifconfig
     	设置用户     sudo vim /etc/hosts
     	分别检测网络是否联通 ping <name>
2. 在从机端设置ROS_MASTER_URI,让从机找到ROS MASTER

```shell
export ROS_MASTER_URI=http://<name：hcx>-pc:11311（当前终端）
#或者
echo “export ROS_MASTER_URI=http://<name：hcx>-pc:11311”>>~/.bash（所有终端）
```

## 参数服务器

### 模型

1. Talker 设置参数
2. Listener 获取参数
3. ROS Master 向 Listener 发送参数

图解

![1663636957943](.\补充图片\1663636957943.png)

### 流程

参数服务器的增删改查

两个 API 可实现

- ros::NodeHandle
- ros::param

#### C++

##### 增加

```cpp
/*
    ros::NodeHandle
        setParam("键",值)
    ros::param
        set("键","值")
*/
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"set_update_param");

    std::vector<std::string> stus;
    stus.push_back("zhangsan");
    stus.push_back("李四");
    stus.push_back("王五");
    stus.push_back("孙大脑袋");

    std::map<std::string,std::string> friends;
    friends["guo"] = "huang";
    friends["yuang"] = "xiao";

    //NodeHandle--------------------------------------------------------
    ros::NodeHandle nh;
    nh.setParam("nh_int",10); //整型
    nh.setParam("nh_double",3.14); //浮点型
    nh.setParam("nh_bool",true); //bool
    nh.setParam("nh_string","hello NodeHandle"); //字符串
    nh.setParam("nh_vector",stus); // vector
    nh.setParam("nh_map",friends); // map

    //修改演示(相同的键，不同的值)
    nh.setParam("nh_int",10000);

    //param--------------------------------------------------------
    ros::param::set("param_int",20);
    ros::param::set("param_double",3.14);
    ros::param::set("param_string","Hello Param");
    ros::param::set("param_bool",false);
    ros::param::set("param_vector",stus);
    ros::param::set("param_map",friends);

    //修改演示(相同的键，不同的值)
    ros::param::set("param_int",20000);

    return 0;
}
```

##### 获得

```cpp
/*
    ros::NodeHandle

        param(键,默认值) 
            存在，返回对应结果，否则返回默认值

        getParam(键,存储结果的变量)
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        getParamCached键,存储结果的变量)--提高变量获取效率
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        getParamNames(std::vector<std::string>)
            获取所有的键,并存储在参数 vector 中 

        hasParam(键)
            是否包含某个键，存在返回 true，否则返回 false

        searchParam(参数1，参数2)
            搜索键，参数1是被搜索的键，参数2存储搜索结果的变量*/

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"get_param");

    //NodeHandle--------------------------------------------------------
    
    ros::NodeHandle nh;
    // param 函数
    int res1 = nh.param("nh_int",100); // 键存在
    int res2 = nh.param("nh_int2",100); // 键不存在
    ROS_INFO("param获取结果:%d,%d",res1,res2);

    // getParam 函数
    int nh_int_value;
    double nh_double_value;
    bool nh_bool_value;
    std::string nh_string_value;
    std::vector<std::string> stus;
    std::map<std::string, std::string> friends;

    nh.getParam("nh_int",nh_int_value);
    nh.getParam("nh_double",nh_double_value);
    nh.getParam("nh_bool",nh_bool_value);
    nh.getParam("nh_string",nh_string_value);
    nh.getParam("nh_vector",stus);
    nh.getParam("nh_map",friends);

    ROS_INFO("getParam获取的结果:%d,%.2f,%s,%d",
            nh_int_value,
            nh_double_value,
            nh_string_value.c_str(),
            nh_bool_value
            );
    for (auto &&stu : stus)
    {
        ROS_INFO("stus 元素:%s",stu.c_str());        
    }

    for (auto &&f : friends)
    {
        ROS_INFO("map 元素:%s = %s",f.first.c_str(), f.second.c_str());
    }

    // getParamCached()
    nh.getParamCached("nh_int",nh_int_value);
    ROS_INFO("通过缓存获取数据:%d",nh_int_value);

    //getParamNames()
    std::vector<std::string> param_names1;
    nh.getParamNames(param_names1);
    for (auto &&name : param_names1)
    {
        ROS_INFO("名称解析name = %s",name.c_str());        
    }
    ROS_INFO("----------------------------");

    ROS_INFO("存在 nh_int 吗? %d",nh.hasParam("nh_int"));
    ROS_INFO("存在 nh_intttt 吗? %d",nh.hasParam("nh_intttt"));

    std::string key;
    nh.searchParam("nh_int",key);
    ROS_INFO("搜索键:%s",key.c_str());
    
    //param--------------------------------------------------------
    ROS_INFO("++++++++++++++++++++++++++++++++++++++++");
    int res3 = ros::param::param("param_int",20); //存在
    int res4 = ros::param::param("param_int2",20); // 不存在返回默认
    ROS_INFO("param获取结果:%d,%d",res3,res4);

    // getParam 函数
    int param_int_value;
    double param_double_value;
    bool param_bool_value;
    std::string param_string_value;
    std::vector<std::string> param_stus;
    std::map<std::string, std::string> param_friends;

    ros::param::get("param_int",param_int_value);
    ros::param::get("param_double",param_double_value);
    ros::param::get("param_bool",param_bool_value);
    ros::param::get("param_string",param_string_value);
    ros::param::get("param_vector",param_stus);
    ros::param::get("param_map",param_friends);

    ROS_INFO("getParam获取的结果:%d,%.2f,%s,%d",
            param_int_value,
            param_double_value,
            param_string_value.c_str(),
            param_bool_value
            );
    for (auto &&stu : param_stus)
    {
        ROS_INFO("stus 元素:%s",stu.c_str());        
    }

    for (auto &&f : param_friends)
    {
        ROS_INFO("map 元素:%s = %s",f.first.c_str(), f.second.c_str());
    }

    // getParamCached()
    ros::param::getCached("param_int",param_int_value);
    ROS_INFO("通过缓存获取数据:%d",param_int_value);

    //getParamNames()
    std::vector<std::string> param_names2;
    ros::param::getParamNames(param_names2);
    for (auto &&name : param_names2)
    {
        ROS_INFO("名称解析name = %s",name.c_str());        
    }
    ROS_INFO("----------------------------");

    ROS_INFO("存在 param_int 吗? %d",ros::param::has("param_int"));
    ROS_INFO("存在 param_intttt 吗? %d",ros::param::has("param_intttt"));

    std::string key;
    ros::param::search("param_int",key);
    ROS_INFO("搜索键:%s",key.c_str());

    return 0;
}
```

##### 删除

```cpp
/* 
    ros::NodeHandle
        deleteParam("键")
        根据键删除参数，删除成功，返回 true，否则(参数不存在)，返回 false

    ros::param
        del("键")
        根据键删除参数，删除成功，返回 true，否则(参数不存在)，返回 false
*/
#include "ros/ros.h"


int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"delete_param");

    ros::NodeHandle nh;
    bool r1 = nh.deleteParam("nh_int");
    ROS_INFO("nh 删除结果:%d",r1);

    bool r2 = ros::param::del("param_int");
    ROS_INFO("param 删除结果:%d",r2);

    return 0;
}
```

#### python

##### 增加

```python
#! /usr/bin/env python


import rospy

if __name__ == "__main__":
    rospy.init_node("set_update_paramter_p")

    # 设置各种类型参数
    rospy.set_param("p_int",10)
    rospy.set_param("p_double",3.14)
    rospy.set_param("p_bool",True)
    rospy.set_param("p_string","hello python")
    rospy.set_param("p_list",["hello","haha","xixi"])
    rospy.set_param("p_dict",{"name":"hulu","age":8})

    # 修改
    rospy.set_param("p_int",100)
```

##### 获得

```python
#! /usr/bin/env python

"""
    参数服务器操作之查询_Python实现:    
        get_param(键,默认值)
            当键存在时，返回对应的值，如果不存在返回默认值
        get_param_cached
        get_param_names
        has_param
        search_param
"""

import rospy

if __name__ == "__main__":
    rospy.init_node("get_param_p")

    #获取参数
    int_value = rospy.get_param("p_int",10000)
    double_value = rospy.get_param("p_double")
    bool_value = rospy.get_param("p_bool")
    string_value = rospy.get_param("p_string")
    p_list = rospy.get_param("p_list")
    p_dict = rospy.get_param("p_dict")

    rospy.loginfo("获取的数据:%d,%.2f,%d,%s",
                int_value,
                double_value,
                bool_value,
                string_value)
    for ele in p_list:
        rospy.loginfo("ele = %s", ele)

    rospy.loginfo("name = %s, age = %d",p_dict["name"],p_dict["age"])

    # get_param_cached
    int_cached = rospy.get_param_cached("p_int")
    rospy.loginfo("缓存数据:%d",int_cached)

    # get_param_names
    names = rospy.get_param_names()
    for name in names:
        rospy.loginfo("name = %s",name)

    rospy.loginfo("-"*80)

    # has_param
    flag = rospy.has_param("p_int")
    rospy.loginfo("包含p_int吗？%d",flag)

    # search_param
    key = rospy.search_param("p_int")
    rospy.loginfo("搜索的键 = %s",key)
```

##### 删除

```python
#! /usr/bin/env python
"""
    参数服务器操作之删除_Python实现:
    rospy.delete_param("键")
    键存在时，可以删除成功，键不存在时，会抛出异常
"""
import rospy

if __name__ == "__main__":
    rospy.init_node("delete_param_p")

    try:
        rospy.delete_param("p_int")
    except Exception as e:
        rospy.loginfo("删除失败")
```

## 相关函数

### C++

#### init()

```cpp
/*
 * 该函数可以解析并使用节点启动时传入的参数(通过参数设置节点名称、命名空间...) 
 *
 * 该函数有多个重载版本，如果使用NodeHandle建议调用该版本。 
 *
 * \param argc 参数个数
 * \param argv 参数列表
 * \param name 节点名称，需要保证其唯一性，不允许包含命名空间
 * \param options 节点启动选项，被封装进了ros::init_options
 *
 */
void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);
```

#### advertise()

```cpp
/**
* \brief 根据话题生成发布对象
*
* 在 ROS master 注册并返回一个发布者对象，该对象可以发布消息
*
* 使用示例如下:
*
*   ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);
*
* \param topic 发布消息使用的话题
*
* \param queue_size 等待发送给订阅者的最大消息数量
*
* \param latch (optional) 如果为 true,该话题发布的最后一条消息将被保存，并且后期当有订阅者连接时会将该消息发送给订阅者
*
* \return 调用成功时，会返回一个发布对象
*
*
*/
template <class M>
Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)

```

#### publish()

```cpp
/**
* 发布消息          
*/
template <typename M>
void publish(const M& message) const
```

#### subscribe()

```cpp
/**
   * \brief 生成某个话题的订阅对象
   *
   * 该函数将根据给定的话题在ROS master 注册，并自动连接相同主题的发布方，每接收到一条消息，都会调用回调
   * 函数，并且传入该消息的共享指针，该消息不能被修改，因为可能其他订阅对象也会使用该消息。
   * 
   * 使用示例如下:

void callback(const std_msgs::Empty::ConstPtr& message)
{
}

ros::Subscriber sub = handle.subscribe("my_topic", 1, callback);

   *
* \param M [template] M 是指消息类型
* \param topic 订阅的话题
* \param queue_size 消息队列长度，超出长度时，头部的消息将被弃用
* \param fp 当订阅到一条消息时，需要执行的回调函数
* \return 调用成功时，返回一个订阅者对象，失败时，返回空对象
* 

void callback(const std_msgs::Empty::ConstPtr& message){...}
ros::NodeHandle nodeHandle;
ros::Subscriber sub = nodeHandle.subscribe("my_topic", 1, callback);
if (sub) // Enter if subscriber is valid
{
...
}

*/
template<class M>
Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&), const TransportHints& transport_hints = TransportHints())
```

#### advertiseService()

```cpp
/**
* \brief 生成服务端对象
*
* 该函数可以连接到 ROS master，并提供一个具有给定名称的服务对象。
*
* 使用示例如下:
\verbatim
bool callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
return true;
}

ros::ServiceServer service = handle.advertiseService("my_service", callback);
\endverbatim
*
* \param service 服务的主题名称
* \param srv_func 接收到请求时，需要处理请求的回调函数
* \return 请求成功时返回服务对象，否则返回空对象:
\verbatim
bool Foo::callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
return true;
}
ros::NodeHandle nodeHandle;
Foo foo_object;
ros::ServiceServer service = nodeHandle.advertiseService("my_service", callback);
if (service) // Enter if advertised service is valid
{
...
}
\endverbatim

*/
template<class MReq, class MRes>
ServiceServer advertiseService(const std::string& service, bool(*srv_func)(MReq&, MRes&))
```

#### serviceClient()

```cpp
/** 
  * @brief 创建一个服务客户端对象
  *
  * 当清除最后一个连接的引用句柄时，连接将被关闭。
  *
  * @param service_name 服务主题名称
  */
 template<class Service>
 ServiceClient serviceClient(const std::string& service_name, bool persistent = false, 
                             const M_string& header_values = M_string())
```

#### 发送请求函数

```cpp
/**
   * @brief 发送请求
   * 返回值为 bool 类型，true，请求处理成功，false，处理失败。
   */
  template<class Service>
  bool call(Service& service)
```

#### 等待服务函数

```cpp
/**
 * ros::service::waitForService("addInts");
 * \brief 等待服务可用，否则一致处于阻塞状态
 * \param service_name 被"等待"的服务的话题名称
 * \param timeout 等待最大时常，默认为 -1，可以永久等待直至节点关闭
 * \return 成功返回 true，否则返回 false。
 */
ROSCPP_DECL bool waitForService(const std::string& service_name, ros::Duration timeout = ros::Duration(-1));

/**
* client.waitForExistence();
* \brief 等待服务可用，否则一致处于阻塞状态
* \param timeout 等待最大时常，默认为 -1，可以永久等待直至节点关闭
* \return 成功返回 true，否则返回 false。
*/
bool waitForExistence(ros::Duration timeout = ros::Duration(-1));
```

#### 回旋函数

```cpp
/**
 * \brief 处理一轮回调
 *
 * 一般应用场景:
 *     在循环体内，处理所有可用的回调函数
 * 
 */
ROSCPP_DECL void spinOnce();
```

```cpp
/** 
 * \brief 进入循环处理回调 
 */
ROSCPP_DECL void spin();
```

**相同点:**二者都用于处理回调函数；

**不同点:**ros::spin() 是进入了循环执行回调函数，而 ros::spinOnce() 只会执行一次回调函数(没有循环)，在 ros::spin() 后的语句不会执行到，而 ros::spinOnce() 后的语句可以执行。

#### 时间相关

##### 获取时刻

```cpp
ros::init(argc,argv,"hello_time");
ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败
ros::Time right_now = ros::Time::now();//将当前时刻封装成对象
ROS_INFO("当前时刻:%.2f",right_now.toSec());//获取距离 1970年01月01日 00:00:00 的秒数

ros::Time someTime(100,100000000);// 参数1:秒数  参数2:纳秒
ROS_INFO("时刻:%.2f",someTime.toSec()); //100.10
```

##### 持续时间

```python
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
ros::Duration du(10);//持续10秒钟,参数是double类型的，以秒为单位
du.sleep();//按照指定的持续时间休眠
ROS_INFO("持续时间:%.2f",du.toSec());//将持续时间换算成秒
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
```

##### 持续时间与时刻运算



##### 运行频率

##### 定时器

### PYTHON

#### init()

```python
def init_node(name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False, disable_signals=False, xmlrpc_port=0, tcpros_port=0):
    """
    在ROS msater中注册节点

    @param name: 节点名称，必须保证节点名称唯一，节点名称中不能使用命名空间(不能包含 '/')
    @type  name: str

    @param anonymous: 取值为 true 时，为节点名称后缀随机编号
    @type anonymous: bool
    """
```

#### 发布对象

```python
class Publisher(Topic):
    """
    在ROS master注册为相关话题的发布方
    """

    def __init__(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
        """
        Constructor
        @param name: 话题名称 
        @type  name: str
        @param data_class: 消息类型

        @param latch: 如果为 true,该话题发布的最后一条消息将被保存，并且后期当有订阅者连接时会将该消息发送给订阅者
        @type  latch: bool

        @param queue_size: 等待发送给订阅者的最大消息数量
        @type  queue_size: int

        """
```

#### publish()

```python
def publish(self, *args, **kwds):
        """
        发布消息
        """
```

#### 订阅对象

```python
class Subscriber(Topic):
    """
   类注册为指定主题的订阅者，其中消息是给定类型的。
    """
    def __init__(self, name, data_class, callback=None, callback_args=None,
                 queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):
        """
        Constructor.

        @param name: 话题名称
        @type  name: str
        @param data_class: 消息类型
        @type  data_class: L{Message} class
        @param callback: 处理订阅到的消息的回调函数
        @type  callback: fn(msg, cb_args)

        @param queue_size: 消息队列长度，超出长度时，头部的消息将被弃用

        """
```

#### advertiseService()

```python
class Service(ServiceImpl):
    """
     声明一个ROS服务

    使用示例::
      s = Service('getmapservice', GetMap, get_map_handler)
    """

    def __init__(self, name, service_class, handler,
                 buff_size=DEFAULT_BUFF_SIZE, error_handler=None):
        """

        @param name: 服务主题名称 ``str``
        @param service_class:服务消息类型

        @param handler: 回调函数，处理请求数据，并返回响应数据

        @type  handler: fn(req)->resp

        """
```

#### serviceClient()

```python
class ServiceProxy(_Service):
    """
   创建一个ROS服务的句柄

    示例用法::
      add_two_ints = ServiceProxy('add_two_ints', AddTwoInts)
      resp = add_two_ints(1, 2)
    """

    def __init__(self, name, service_class, persistent=False, headers=None):
        """
        ctor.
        @param name: 服务主题名称
        @type  name: str
        @param service_class: 服务消息类型
        @type  service_class: Service class
        """
```

#### 发送请求函数

```python
def call(self, *args, **kwds):
        """
        发送请求，返回值为响应数据
        """
```

#### 等待服务函数

```python
def wait_for_service(service, timeout=None):
    """
    调用该函数时，程序会处于阻塞状态直到服务可用
    @param service: 被等待的服务话题名称
    @type  service: str
    @param timeout: 超时时间
    @type  timeout: double|rospy.Duration
    """
```

#### 回旋函数

```python
def spin():
    """
    进入循环处理回调 
    """
```

#### 时间相关

##### 获取时刻

##### 持续时间

##### 持续时间与时刻运算

##### 运行频率

##### 定时器

