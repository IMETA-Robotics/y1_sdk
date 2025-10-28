# 依赖项
  - Ubuntu 20.04 LTS
  - ROS Noetic

# Mujoco Simulation

## 1. Installing Mujoco210 and mujoco-py

### 1.1 Install Mujoco

1. [Download Mujoco210](https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz)

2. Extract the files

    ```bash
    mkdir ~/.mujoco
    cd (Directory containing the package)
    tar -zxvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco
    ```

3. Add the environment variable

    ```bash
    echo "export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:\$LD_LIBRARY_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Test the installation

    ```bash
    cd ~/.mujoco/mujoco210/bin
    ./simulate ../model/humanoid.xml
    ```

### 1.2 Install mujoco-py

1. Install dependencies
    ```bash
    pip install mujoco_py
    sudo apt-get install libglew-dev
    ```
2. Add the environment variable

    ```bash
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
    echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so" >> ~/.bashrc
    source ~/.bashrc
    ```

## Y1 Mujoco Simulation (Without Gripper)

### 1. Run Mujoco simulation

    ```bash
    source devel/setup.bash
    roslaunch y1_mujoco y1_no_gripper_mujoco.launch
    ```

### 2. Control Y1 arm without the gripper via RViz GUI (Run in a new terminal)

    ```bash
    source devel/setup.bash
    roslaunch y1_description display_y1_no_gripper.launch
    ```