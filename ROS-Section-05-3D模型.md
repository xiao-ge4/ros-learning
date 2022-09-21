# Section 5 3D模型

- URDF 用于创建机器人模型
- Gzebo 用于搭建仿真环境
- Rviz 图形化的显示机器人各种传感器感知到的环境信息

## 机器人组成

1. **传感系统**
2. **控制系统**：输出控制命令信号
3. **驱动系统**：肌肉和筋络，负责驱动执行机构
4. **执行机构**

## URDF

Unified Robot Description Format（统一机器人描述格式）

XML 文件

### 分块解析

1. <robot>

   >1. 完整机器人，最顶层标签
   >2. 属性：name 设置机器人模型名称
   
2. <link>

   > 1. 描述机器人某个刚体部分：外观、物理属性
   >   尺寸、颜色、形状、惯性矩阵、碰撞参数
   >
   > 2. 属性：name 连杆命名
   >
   > 3. 子标签
   >
   >   1. <inertial>：惯性矩阵
   >   2. <collision>：碰撞属性
   >   3. <visual>：外观参数
   >      1. geometry 设置连杆的形状
   >         1. box(盒状) | 属性：size=长x 宽y 高z
   >         2. cylinder(圆柱) | 属性：半径radius 高度length
   >         3.  sphere(球体) | 属性：半径radius
   >         4. mesh(为连杆添加皮肤)| 属性： filename=资源路径(格式:**package://<packagename>/<path>/文件**)
   >      2. origin 设置偏移量与倾斜弧度
   >         属性 ：xyz=x偏移 y偏移 z偏移
   >          rpy=x翻滚 y俯仰 z偏航 (单位是弧度)
   >      3. metrial 设置材料属性(颜色)
   >         属性：name
   >         1. 标签: color
   >            属性: rgba=红绿蓝权重值与透明度 (每个权重值以及透明度取值[0,1])
   >
   > 4. 案例
   >
   >   ```xml
   >       <link name="base_link">
   >           <visual>
   >               <!-- 形状 -->
   >               <geometry>
   >                   <!-- 长方体的长宽高 -->
   >                   <!-- <box size="0.5 0.3 0.1" /> -->
   >                   <!-- 圆柱，半径和长度 -->
   >                   <!-- <cylinder radius="0.5" length="0.1" /> -->
   >                   <!-- 球体，半径-->
   >                   <!-- <sphere radius="0.3" /> -->
   >   
   >               </geometry>
   >               <!-- xyz坐标 rpy翻滚俯仰与偏航角度(3.14=180度 1.57=90度) -->
   >               <origin xyz="0 0 0" rpy="0 0 0" />
   >               <!-- 颜色: r=red g=green b=blue a=alpha -->
   >               <material name="black">
   >                   <color rgba="0.7 0.5 0 0.5" />
   >               </material>
   >           </visual>
   >       </link>
   >   ```

3. <joint>

   > 1. 描述机器人 关节 的 运动学和动力学属性
   > 关节运动位置、速度限制
   > 2. 属性
   > name 关节命名
   > type 关节运动形式
   >
   > | 关节类型   | 描述                                          |
   > | ---------- | --------------------------------------------- |
   > | continuous | 旋转关节，围绕单轴无限旋转                    |
   > | revolute   | 旋转关节，类似于continuous,但有旋转的角度极限 |
   > | prismatic  | 滑动关节，沿某一轴线移动的关节，有位置极限    |
   > | planar     | 平面关节，允许在平面正交方向上平移或者旋转    |
   > | floating   | 浮动关节，允许进行平移、旋转运动              |
   > | fixed      | 固定关节，不允许运动的特殊关节                |
   >
   > 3. 子标签
   >
   > <parent>：必需的、强制的属性
   >
   > <child>：必需的、强制的属性
   >
   > <origin>：属性: xyz=各轴线上的偏移量 rpy=各轴线上的偏移弧度
   >
   > <axis>：属性: xyz用于设置围绕哪个关节轴运动
   >
   > <calibration>：关节的参考位置，用来校准关节的绝对位置；
   > <dynamics>：关节的物理属性，阻尼值、物理静摩擦力等
   > <limit>：运动的极限值，关节运动的上下限位置、速度限制、力矩限制等
   > <mimic>：关节与已有关节的关系；
   > <safety_controller>：描述安全控制器参数
   >
 4. 案例
   >    创建机器人模型，底盘为长方体，在长方体的前面添加一摄像头，摄像头可以沿着 Z 轴 360 度旋转
   >    URDF
   >
   > ```xml
   > <!-- 
   >    需求: 创建机器人模型，底盘为长方体，
   >         在长方体的前面添加一摄像头，
   >         摄像头可以沿着 Z 轴 360 度旋转
   > 
   > -->
   > <robot name="mycar">
   >    <!-- 底盘 -->
   >    <link name="base_link">
   >        <visual>
   >            <geometry>
   >                <box size="0.5 0.2 0.1" />
   >            </geometry>
   >            <origin xyz="0 0 0" rpy="0 0 0" />
   >            <material name="blue">
   >                <color rgba="0 0 1.0 0.5" />
   >            </material>
   >        </visual>
   >    </link>
   > 
   >    <!-- 摄像头 -->
   >    <link name="camera">
   >        <visual>
   >            <geometry>
   >                <box size="0.02 0.05 0.05" />
   >            </geometry>
   >            <origin xyz="0 0 0" rpy="0 0 0" />
   >            <material name="red">
   >                <color rgba="1 0 0 0.5" />
   >            </material>
   >        </visual>
   >    </link>
   > 
   >    <!-- 关节 -->
   >    <joint name="camera2baselink" type="continuous">
   >        <parent link="base_link"/>
   >        <child link="camera" />
   >        <!-- 需要计算两个 link 的物理中心之间的偏移量 -->
   >        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
   >        <axis xyz="0 0 1" />
   >    </joint>
   > 
   > </robot>
   > ```
   >
   > ​		launch
   >
   > ```xml
   > <launch>
   > 
   >     <param name="robot_description" textfile="$(find urdf_rviz_demo)/urdf/urdf/urdf03_joint.urdf" />
   >     <node pkg="rviz" type="rviz" name="rviz" args="-d $(find urdf_rviz_demo)/config/helloworld.rviz" /> 
   > 
   >     <!-- 添加关节状态发布节点 -->
   >     <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
   >     <!-- 添加机器人状态发布节点 -->
   >     <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
   >     <!-- 可选:用于控制关节运动的节点 -->
   >     <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
   > 
   > </launch>
   > 
   > <!--关节运动控制节点(可选)，会生成关节控制的UI，用于测试关节运动是否正常-->
   > ```
   >
 5. 优化 base_footprint 阻止下沉
   >
   >    ```xml
   >    <!--
   >    
   >        使用 base_footprint 优化
   >    
   >    -->
   >    <robot name="mycar">
   >        <!-- 设置一个原点(机器人中心点的投影) -->
   >        <link name="base_footprint">
   >            <visual>
   >                <geometry>
   >                    <sphere radius="0.001" />
   >                </geometry>
   >            </visual>
   >        </link>
   >    
   >        <!-- 添加底盘 -->
   >        <link name="base_link">
   >            <visual>
   >                <geometry>
   >                    <box size="0.5 0.2 0.1" />
   >                </geometry>
   >                <origin xyz="0 0 0" rpy="0 0 0" />
   >                <material name="blue">
   >                    <color rgba="0 0 1.0 0.5" />
   >                </material>
   >            </visual>
   >        </link>
   >    
   >        <!-- 底盘与原点连接的关节 -->
   >        <joint name="base_link2base_footprint" type="fixed">
   >            <parent link="base_footprint" />
   >            <child link="base_link" />
   >            <origin xyz="0 0 0.05" />
   >        </joint>
   >    
   >        <!-- 添加摄像头 -->
   >        <link name="camera">
   >            <visual>
   >                <geometry>
   >                    <box size="0.02 0.05 0.05" />
   >                </geometry>
   >                <origin xyz="0 0 0" rpy="0 0 0" />
   >                <material name="red">
   >                    <color rgba="1 0 0 0.5" />
   >                </material>
   >            </visual>
   >        </link>
   >        <!-- 关节 -->
   >        <joint name="camera2baselink" type="continuous">
   >            <parent link="base_link"/>
   >            <child link="camera" />
   >            <origin xyz="0.2 0 0.075" rpy="0 0 0" />
   >            <axis xyz="0 0 1" />
   >        </joint>
   >    
   >    </robot>
   >    ```

6. URDF工具

   > 1. check_urdf 语法检查
   >    进入urdf文件所属目录，调用:`check_urdf urdf 文件`，如果不抛出异常，说明文件合法,否则非法
   > 2. urdf_to_graphiz 结构查看
   >    进入urdf文件所属目录，调用:`urdf_to_graphiz urdf文件`，当前目录下会生成 pdf 文件

## URDF + RVIZ--静态

### 流程

1. 准备:新建功能包，导入依赖
2. 核心:编写 urdf 文件
3. 核心:在 launch 文件集成 URDF 与 Rviz
4. 在 Rviz 中显示机器人模型

### 实现

1. 创建一个机器人建模的功能包（添加依赖）

   ```shell
   catkin_create_pkg mbot_description urdf xacro
   ```

2. 在功能包下，新建以下目录，说明：

   > urdf：存放机器人模型的URDF或xacro文件
   > meshes：放置URDF中引用的模型渲染文件（暂时不用）
   > launch：保存相关启动文件
   > config：保存rviz的配置文件

3. 在 urdf 文件夹下，新建一个 .urdf 文件

   ```xml
   <robot name="mycar">
       <link name="base_link">
           <visual>
               <geometry>
                   <box size="0.5 0.2 0.1" />
               </geometry>
           </visual>
       </link>
   </robot>
   ```

4. 在 launch 文件下，新建一个 .launch 文件

   ```xml
   <launch>
   
       <!-- 设置参数 -->
       <param name="robot_description" textfile="$(find 包名)/urdf/urdf/urdf01_HelloWorld.urdf" />
   
       <!-- 启动 rviz -->
       <node pkg="rviz" type="rviz" name="rviz" />
   
   </launch>
   ```
   
5. 启动，添加机器人显示
   RobotModel

6. 优化 rviz 启动

   > 1. rviz 中将当前配置保存进`config`目录
   > 2. `launch`文件中 Rviz 的启动配置添加参数:`args`,值设置为`-d 配置文件路径`

## XACRO

目的：优化 URDF

优点：可以声明变量、通过数学运算求解、流程控制控制执行顺序、类似函数的实现

语言：XML Macros

### 示例--快速体验

注意：以下并不合法，只是一个概览

新建 XACRO 文件，编写 .xacro 文件

上面 URDF 文件改写

```xml
<robot name="mycar" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- 属性封装 -->
    <xacro:property name="wheel_radius" value="0.0325" />
    <xacro:property name="wheel_length" value="0.0015" />
    <xacro:property name="PI" value="3.1415927" />
    <xacro:property name="base_link_length" value="0.08" />
    <xacro:property name="lidi_space" value="0.015" />

    <!-- 宏 -->
    <xacro:macro name="wheel_func" params="wheel_name flag" >
        <link name="${wheel_name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}" />
                </geometry>

                <origin xyz="0 0 0" rpy="${PI / 2} 0 0" />

                <material name="wheel_color">
                    <color rgba="0 0 0 0.3" />
                </material>
            </visual>
        </link>

        <!-- 3-2.joint -->
        <joint name="${wheel_name}2link" type="continuous">
            <parent link="base_link"  />
            <child link="${wheel_name}_wheel" />
            <!-- 
                x 无偏移
                y 车体半径
                z z= 车体高度 / 2 + 离地间距 - 车轮半径

            -->
            <origin xyz="0 ${0.1 * flag} ${(base_link_length / 2 + lidi_space - wheel_radius) * -1}" rpy="0 0 0" />
            <axis xyz="0 1 0" />
        </joint>

    </xacro:macro>
    <xacro:wheel_func wheel_name="left" flag="1" />
    <xacro:wheel_func wheel_name="right" flag="-1" />
</robot>

```

在 xacro文件 所属目录，执行:`rosrun xacro xacro xxx.xacro > xxx.urdf`, 会将 xacro 文件解析为 urdf 文件，内容如下:

```xml
<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from test.xacro                     | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="mycar">
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.0015" radius="0.0325"/>
      </geometry>
      <origin rpy="1.57079635 0 0" xyz="0 0 0"/>
      <material name="wheel_color">
        <color rgba="0 0 0 0.3"/>
      </material>
    </visual>
  </link>
  <!-- 3-2.joint -->
  <joint name="left2link" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <!-- 
                x 无偏移
                y 车体半径
                z z= 车体高度 / 2 + 离地间距 - 车轮半径

            -->
    <origin rpy="0 0 0" xyz="0 0.1 -0.0225"/>
    <axis xyz="0 1 0"/>
  </joint>
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.0015" radius="0.0325"/>
      </geometry>
      <origin rpy="1.57079635 0 0" xyz="0 0 0"/>
      <material name="wheel_color">
        <color rgba="0 0 0 0.3"/>
      </material>
    </visual>
  </link>
  <!-- 3-2.joint -->
  <joint name="right2link" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <!-- 
                x 无偏移
                y 车体半径
                z z= 车体高度 / 2 + 离地间距 - 车轮半径

            -->
    <origin rpy="0 0 0" xyz="0 -0.1 -0.0225"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### 语法详解

在使用 xacro 生成 urdf 时，根标签`robot`中必须包含命名空间声明:`xmlns:xacro="http://wiki.ros.org/xacro"`

#### 属性与算数运算

属性定义

```xml
<xacro:property name="xxxx" value="yyyy" />
```

属性调用

```
${属性名称}
```

算数运算

```
${数学表达式}
```

#### 宏

类似于函数实现

宏定义

```xml
<xacro:macro name="宏名称" params="参数列表(多参数之间使用空格分隔)">

    .....

    参数调用格式: ${参数名}

</xacro:macro>
```

宏调用

```xml
<xacro:宏名称 参数1=xxx 参数2=xxx/>
```

#### 文件包含

机器人由多部件组成，不同部件可封装为单独的 xacro 文件，再将不同的文件集成，使用文件包含实现

```xml
<robot name="xxx" xmlns:xacro="http://wiki.ros.org/xacro">
      <xacro:include filename="my_base.xacro" />
      <xacro:include filename="my_camera.xacro" />
      <xacro:include filename="my_laser.xacro" />
      ....
</robot>
```

### 示例--完整流程

#### XARCO

```xml
<!--
    使用 xacro 优化 URDF 版的小车底盘实现：

    实现思路:
    1.将一些常量、变量封装为 xacro:property
      比如:PI 值、小车底盘半径、离地间距、车轮半径、宽度 ....
    2.使用 宏 封装驱动轮以及支撑轮实现，调用相关宏生成驱动轮与支撑轮

-->
<!-- 根标签，必须声明 xmlns:xacro -->
<robot name="my_base" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 封装变量、常量 -->
    <xacro:property name="PI" value="3.141"/>
    <!-- 宏:黑色设置 -->
    <material name="black">
        <color rgba="0.0 0.0 0.0 1.0" />
    </material>
    <!-- 底盘属性 -->
    <xacro:property name="base_footprint_radius" value="0.001" /> <!-- base_footprint 半径  -->
    <xacro:property name="base_link_radius" value="0.1" /> <!-- base_link 半径 -->
    <xacro:property name="base_link_length" value="0.08" /> <!-- base_link 长 -->
    <xacro:property name="earth_space" value="0.015" /> <!-- 离地间距 -->

    <!-- 底盘 -->
    <link name="base_footprint">
      <visual>
        <geometry>
          <sphere radius="${base_footprint_radius}" />
        </geometry>
      </visual>
    </link>

    <link name="base_link">
      <visual>
        <geometry>
          <cylinder radius="${base_link_radius}" length="${base_link_length}" />
        </geometry>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <material name="yellow">
          <color rgba="0.5 0.3 0.0 0.5" />
        </material>
      </visual>
    </link>

    <joint name="base_link2base_footprint" type="fixed">
      <parent link="base_footprint" />
      <child link="base_link" />
      <origin xyz="0 0 ${earth_space + base_link_length / 2 }" />
    </joint>

    <!-- 驱动轮 -->
    <!-- 驱动轮属性 -->
    <xacro:property name="wheel_radius" value="0.0325" /><!-- 半径 -->
    <xacro:property name="wheel_length" value="0.015" /><!-- 宽度 -->
    <!-- 驱动轮宏实现 -->
    <xacro:macro name="add_wheels" params="name flag">
      <link name="${name}_wheel">
        <visual>
          <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_length}" />
          </geometry>
          <origin xyz="0.0 0.0 0.0" rpy="${PI / 2} 0.0 0.0" />
          <material name="black" />
        </visual>
      </link>

      <joint name="${name}_wheel2base_link" type="continuous">
        <parent link="base_link" />
        <child link="${name}_wheel" />
        <origin xyz="0 ${flag * base_link_radius} ${-(earth_space + base_link_length / 2 - wheel_radius) }" />
        <axis xyz="0 1 0" />
      </joint>
    </xacro:macro>
    <xacro:add_wheels name="left" flag="1" />
    <xacro:add_wheels name="right" flag="-1" />
    <!-- 支撑轮 -->
    <!-- 支撑轮属性 -->
    <xacro:property name="support_wheel_radius" value="0.0075" /> <!-- 支撑轮半径 -->

    <!-- 支撑轮宏 -->
    <xacro:macro name="add_support_wheel" params="name flag" >
      <link name="${name}_wheel">
        <visual>
            <geometry>
                <sphere radius="${support_wheel_radius}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black" />
        </visual>
      </link>

      <joint name="${name}_wheel2base_link" type="continuous">
          <parent link="base_link" />
          <child link="${name}_wheel" />
          <origin xyz="${flag * (base_link_radius - support_wheel_radius)} 0 ${-(base_link_length / 2 + earth_space / 2)}" />
          <axis xyz="1 1 1" />
      </joint>
    </xacro:macro>

    <xacro:add_support_wheel name="front" flag="1" />
    <xacro:add_support_wheel name="back" flag="-1" />

</robot>
```

#### launch文件

两个方法

1. 先将 xacro 文件转换出 urdf 文件，然后集成

   ```xml
   <launch>
       <param name="robot_description" textfile="$(find demo01_urdf_helloworld)/urdf/xacro/my_base.urdf" />
   
       <node pkg="rviz" type="rviz" name="rviz" args="-d $(find demo01_urdf_helloworld)/config/helloworld.rviz" />
       <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" output="screen" />
       <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen" />
       <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" output="screen" />
   
   </launch>
   ```

2. 在 launch 文件中直接加载 xacro
   区别：加载`robot_description`时使用`command`属性，属性值就是调用 xacro 功能包的 xacro 程序直接解析 xacro 文件

   ```xml
   <launch>
       <param name="robot_description" command="$(find xacro)/xacro $(find demo01_urdf_helloworld)/urdf/xacro/my_base.urdf.xacro" />
   
       <node pkg="rviz" type="rviz" name="rviz" args="-d $(find demo01_urdf_helloworld)/config/helloworld.rviz" />
       <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" output="screen" />
       <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen" />
       <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" output="screen" />
   
   </launch>
   ```

## URDF + RVIZ--动态

目的

> 通过 URDF 结合 rviz 可以创建并显示机器人模型 --> 静态模型
>
> 调用 Arbotix --> 动态

简介

> Arbotix 是一款控制电机、舵机的控制板，并提供相应的 ros 功能包
>
> 可以驱动真实的 Arbotix 控制板
>
> 提供一个差速控制器：接受速度控制指令更新机器人的 joint 状态
>
> 这个差速控制器在 arbotix_python 程序包中
>
> 完整的 arbotix 程序包还包括多种控制器，分别对应 dynamixel 电机、多关节机械臂以及不同形状的夹持器。

### 流程

1. 安装 Arbotix
2. 创建新功能包，准备机器人 urdf、xacro 文件
3. 添加 Arbotix 配置文件
4. 编写 launch 文件配置 Arbotix
5. 启动 launch 文件并控制机器人模型运动

### 实现

1. 安装 Arbotix

   > 两个方法
   >
   > 1. sudo apt-get install ros-<<VersionName()>>-arbotix
   > 2. git clone https://github.com/vanadiumlabs/arbotix_ros.git
   >    然后 catkin_make 编译

2. 创建新功能包，准备机器人 urdf、xacro 文件
3. 添加 Arbotix 配置文件
   参考来源http://wiki.ros.org/arbotix_python/diff_controller

```yaml
# 该文件是控制器配置,一个机器人模型可能有多个控制器，比如: 底盘、机械臂、夹持器(机械手)....
# 因此，根 name 是 controller
controllers: {
   # 单控制器设置
   base_controller: {
          #类型: 差速控制器
       type: diff_controller,
       #参考坐标
       base_frame_id: base_footprint, 
       #两个轮子之间的间距
       base_width: 0.2,
       #控制频率
       ticks_meter: 2000, 
       #PID控制参数，使机器人车轮快速达到预期速度
       Kp: 12, 
       Kd: 12, 
       Ki: 0, 
       Ko: 50, 
       #加速限制
       accel_limit: 1.0 
    }
}
```

4. launch 文件中配置 arbotix 节点

```xml
<node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
     <rosparam file="$(find my_urdf05_rviz)/config/hello.yaml" command="load" />
     <param name="sim" value="true" />
</node>

<!--解释-->
<rosparam> arbotix 驱动机器人运行时，需要获取机器人信息，可以通过 file 加载配置文件
<param> 在仿真环境下，需要配置 sim 为 true
```

5. 启动 launch 文件并控制机器人模型运动

```shell
roslaunch xxxx ....launch
```

配置 rviz

Fixed Frame 选择 odom

Odometry Topic 设置为 odom

查看话题-->有/cmd_vel

控制小车移动

```
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
```

## URDF + Gazebo

三维物理仿真平台

### 特点

1. 具备强大的**物理引擎**
2. 高质量的**图形渲染**
3. 方便的**编程与图形接口**
4. **开源**免费

### 常用工具

1. gazebo_ros 
   gazebo接口封装、gazebo服务端和客户端的启动、URDF模型生成等
2. gazebo_msgs 
   是gazebo的Msg和Srv数据结构
3. gazebo_plugins 
   gazebo的通用传感器插件
4. gazebo_ros_api_plugin 和gazebo_ros_path_plugin 
   接口封装

## URDF + Gazebo + Rviz