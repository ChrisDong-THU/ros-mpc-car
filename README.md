# ros-mpc-car
MPC实现ROS内仿真小车控制，based on [Nonlinear Model Predictive Control](https://github.com/Geonhee-LEE/mpc_ros)

## 运行环境
1. Ubuntu 20.04

2. Install [ROS](http://wiki.ros.org/) Noetic.

3. Install ROS dependencies.

   ```
   sudo apt install ros-noetic-costmap-2d \
   ros-noetic-move-base \
   ros-noetic-global-planner \
   ros-noetic-amcl
   ```

4. Install Ipopt: Please refer the tutorial in ["document/ipopt_install"](https://github.com/Geonhee-LEE/mpc_ros/tree/melodic/assets/document/ipopt_install).
   在安装过程中，会出现部分脚本无法连接上的情况，将自动化安装脚本中的下载部分注释，手动下载`environment/`的离线源码包，移动到相应文件夹即可。

5. Create your own catkin_ws and clone the repositories.

   ```
   git clone https://github.com/Geonhee-LEE/mpc_ros.git 
   ```

   - _(optional)_ If you already have the urdf model, you don't need to clone below  

     ```
     git clone https://github.com/CzJaewan/servingbot.git
     ```

   > NOTE: you can also refer other models such as ackermann model, holonomic model. you can see it [mpc_ros_description](https://github.com/Geonhee-LEE/mpc_ros_description)

6. Install turtlebot3_msgs (已补全功能包，可省略)

7. Build (_catkin_make_) and Try it.

## 运行
`roslaunch mpc_ros ref_trajectory_tracking_gazebo.launch`
