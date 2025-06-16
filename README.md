### 依赖项
ubuntu 20.04  ros-noetic版本

### build
bash build.sh

### run
source devel/setup.bash
roslaunch imeta_y1_controller single_arm_controller.launch

问题:
1. 夹爪是否需要支持 力钜控制？

2. 是否将末端重量传进SDK里，一起做动力学建模？

3. 只给电机下发一帧位置控制，看看效果？

4. 示教器或末端夹具装上后，需要对J6进行零位标定，因此开放J6零位标定和所有关节使能和失能的接口。

5. rviz上可视化关节位置信息，end pose信息。

6. 增加通过/joint_state_publisher_gui控制机械臂。

7. 增加位置控制 和 end pose控制的两个demo。

8. 增加机械臂回零位的接口。

9. 增加停止机械臂，让机械臂会以一个恒定阻尼落下

10. 驱动被杀死后，是不是不需要对电机失能，这样会更安全？