# y1_gazebo

# Software Dependency

  - Ubuntu 22.04 LTS
  - ROS Humble

## 1 gazebo仿真

### 0 环境配置

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
```
  
### 1.1 y1 gazebo仿真(有夹爪)

gazebo仿真运行

```bash
cd ~/y1_sdk
source install/setup.bash
```

```bash
ros2 launch y1_gazebo y1_with_gripper_gazebo.launch.py
```

### 1.2 y1 gazebo仿真(无夹爪)

```bash
ros2 launch y1_gazebo y1_no_gripper_gazebo.launch.py
```

注：**若通过moveit控制时需要先启动gazebo，再启动moveit，并且使用y1_moveit.launch.py而不是demo.launch.py**
