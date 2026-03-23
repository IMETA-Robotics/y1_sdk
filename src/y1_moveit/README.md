# y1_Moveit2

# Software Dependency

  - Ubuntu 22.04 LTS
  - ROS Humble

## 1 安装Moveit2

1）二进制安装

```bash
sudo apt install ros-humble-moveit*
```

2）安装依赖库

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

## 3 moveit控制真实机械臂

### 3.1 编译相关功能模块，如需编译特定功能模块，请编辑~/build.sh文件

  ```sh
  cd ~/y1_sdk
  bash build.sh
  ```

### 3.3 开启控制节点

```bash
source install/setup.bash
ros2 launch y1_controller single_arm_control.launch.py
```

### 3.4 moveit2控制

开启moveit2

```bash
source install/setup.bash
```

#### 3.4.1 无夹爪运行

```bash
ros2 launch y1_no_gripper_moveit demo.launch.py
```

#### 3.4.2 有夹爪运行

```bash
ros2 launch y1_with_gripper_moveit demo.launch.py
```

可以直接拖动机械臂末端的箭头控制机械臂

调整好位置后点击左侧MotionPlanning中Planning的Plan&Execute即可开始规划并运动


## 4 moveit控制仿真机械臂

### 4.1 gazebo

#### 4.1.1 运行gazebo

见 [y1_gazebo](../y1_gazebo/README.md)

#### 4.1.2 moveit控制

```bash
source install/setup.bash
```

注!!!: **下面的launch不是控制真实机械臂的demo.launch.py,且需要在gazebo之后运行,否则会没有机械臂模型**

有夹爪运行

```bash
ros2 launch y1_with_gripper_moveit y1_moveit.launch.py
```

无夹爪运行

```bash
ros2 launch y1_no_gripper_moveit y1_moveit.launch.py
```

### 4.1.3 提示

若使用ctrl + c 无法退出gazebo界面，请另启动一个终端，使用 

```bash
pkill -9 -f gzclient
pkill -9 -f gzserver
```