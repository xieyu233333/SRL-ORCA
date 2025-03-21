# SRL-ORCA

**SRL-ORCA** is an open-source program designed to ensure safe and efficient navigation for robot swarms in dynamic and complex environments without the need for a map. It employs a safety-reinforced learning approach, addressing the safety deficiencies inherent in traditional DRL methods, and theoretically guarantees collision avoidance safety in homogeneous scenarios. All technical documentation is available in both Chinese and English.

## Author

 **Jianmin Qin** (Code Contributor), Jiahu Qin, Jiaxin Qiu, Qingchen Liu, Man Li and Qichao Ma

Contact: qjm@mail.ustc.edu.cn

## Citation

If this project helps you, we will open source more! You can star this repository and cite our paper:

```
@article{qin2023srl,
  title={Srl-orca: A socially aware multi-agent mapless navigation algorithm in complex dynamic scenes},
  author={Qin, Jianmin and Qin, Jiahu and Qiu, Jiaxin and Liu, Qingchen and Li, Man and Ma, Qichao},
  journal={IEEE Robotics and Automation Letters},
  volume={9},
  number={1},
  pages={143--150},
  year={2023},
  publisher={IEEE}
}
```

## Table of Contents

* [Installation](#1-installation)
* [Run Simulations](#2-run-simulations)


## 1.installation
Compilation commands and preparatory steps:
编译命令和辅助准备工作
```bash
cd /home/qjm/orca/turtlebot3/
qjm@qjm-Legion-Y9000P-IAH7H:~/orca/turtlebot3$ catkin_make
```
If you are copying an entire project and making improvements to create a new project, you need to modify the CMake and package files (as of 2024-01-29):
    The CMake file needs to be updated, specifically the project name:
其中，如果复制以前整个文件，再做改进成新工程。需要修改cmake文件和package文件-24.01.29
   其中cmake文件需要改，其中的文件名
   project(hsrln_robot_drl_6robots)
   
   The package file needs to be updated, specifically the package name:
   其中package文件需要改，其中的文件名
  <name>hsrln_robot_drl_6robots</name>
  
  
Additionally, if you add new C++ files, you need to modify the corresponding .cpp and .h files, as well as the CMake files in the respective directories (as of 2024-01-30).
  另外，如果新增C++文件，还需要对应修改.cpp和.h文件，以及相应文件夹下的cmake文件-24.01.30
  
 

## 2.Run-Simulations
To run the SRL-ORCA simulation training (core functionality):
正式使用，以下为SRL-ORCA仿真训练命令（核心工作）

Open three terminals to run commands individually and sequentially.

(Launch all necessary launch files)
(调用所有launch文件列表)
Terminal 1:
```bash
roslaunch turtlebot3_gazebo
roslaunch turtlebot3_gazebo ral2310_1.launch
```
(Use 0 for real robot operation, 1 for simulation)
(要带数字0-实物运行， 1-仿真运行)

Terminal 2:
```bash
qjm@qjm-Legion-Y9000P-IAH7H:~/桌面$ rosrun collvoid_local_planner OrcaLocalPlanner 1
```


运行训练程序
Terminal 3:
```bash
qjm@qjm-Legion-Y9000P-IAH7H:~$ cd /home/qjm/orca/turtlebot3/src/multi_robot_drl_6robots/src
qjm@qjm-Legion-Y9000P-IAH7H:~/orca/turtlebot3/src/multi_robot_drl_6robots/src$ python3 ./sac.py
```

Set the training mode in sac.py:
```bash
is_training = True  # Set to True for training, False for testing
agent.load_models(31320)  # Keep this line to continue training with an old model, new models will start from 0 and save incrementally to save memory.
```

训练模式和测试模式，sac.py程序中需要设置
is_training = True #训练的时候要改成True，不训练改成False
agent.load_models(31320)---老模型继续训练时要开着,新模型会从0开始重新存储，以少占用内存

## 3.Run on Real Robots

To run SRL-ORCA on real robots (core functionality):

(Use 0 for real robot operation, 1 for simulation)


以下为SRL-ORCA实物机器人运行命令--以下不同，其余相同（核心工作）
(要带数字0-实物运行， 1-仿真运行)
```bash
qjm@qjm-Legion-Y9000P-IAH7H:~/桌面$ rosrun collvoid_local_planner OrcaLocalPlanner 0
```

The robot launch file is different for real robots:
机器人启动文件不一样，应为如下
```bash
qjm@qjm-Legion-Y9000P-IAH7H:~/orca/turtlebot3/src/multi_robot_drl_6robots/src$ python3 ./sac_on_real.py
```

Important steps for real robot experiments:

    (1) Check the IP addresses in the bashrc file for the host machine and each TB3 robot (see sections 18 and 15). Ensure the TB3 robot IP is set to http://192.168.1.6:11311.

    (2) Start roscore, vrpn, and the 6 TB3 robots.

    (3) Start orca, and finally run the robot with: python3 ./sac_on_real.py.

其中，要核对实物实验重要步骤，
（1）检查本机IP和各TB3机器人IP的bashc文件（见18和15）--注意TB3机器人IP是否为 'http://192.168.1.6:11311'

（2）开启roscore，开启vrpn，开启6个TB3机器人
（3）开启orca,最后开启机器人运行：python3 ./sac_on_real.py


## 4.Auxiliary Tasks

To open TensorBoard for viewing training curves:

    Method 1: Open TensorBoard in VS Code by selecting the folder containing the training logs. This method is less convenient as it only allows opening one folder at a time.

    Method 2: Copy all tfevents training files from different folders into a single folder, e.g.:

tensorboard打开曲线图，
方法1--在vs code里打开对应sac.py的tensorboard选取存训练工具的文件夹，但只能打开一个，不太好用

方法2--先把其他文件夹的tfevents训练文件，复制到同一个文件夹中，例如以下文件
```
home/qjm/orca/turtlebot3/src/idsi_multi_robot_drl_6robots/src/runs
```

Then, in a new terminal, run:
在新terminal界面打开以下启动代码
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ tensorboard --logdir=/home/qjm/orca/turtlebot3/src/idsi_multi_robot_drl_6robots/src/runs

qjm@qjm-Legion-Y9000P-IAH7H:~$ tensorboard --logdir=/home/qjm/orca/turtlebot3/src/multi_robot_drl_6robots/src/runs
```
    Copy the URL http://localhost:6006/ to your browser to view the training curves.
再复制http://localhost:6006/地址到浏览器中，打开即可看到

To clear ROS logs:
清理内存
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ rosclean check
qjm@qjm-Legion-Y9000P-IAH7H:~$ rosclean purge
```

To open RViz:
打开rviz
```
qjm@qjm-Legion-Y9000P-IAH7H:~/桌面$ rviz
```

建图需要gmapping用激光雷达在rviz上显示--230723

To start Gazebo separately (e.g., for drawing):
单独启动gazebo，例如画图
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ gazebo
```

To view laser scan data:
调用scan雷达数据
```
qjm@qjm-Legion-Y9000P-IAH7H:~/桌面$ rostopic echo /tb3_0/scan
```

To view the TF tree:
看tf树的内容
```
qjm@qjm-Legion-Y9000P-IAH7H:~/桌面$ rosrun tf
qjm@qjm-Legion-Y9000P-IAH7H:~/桌面$ rosrun tf2
jm@qjm-Legion-Y9000P-IAH7H:~/桌面$ rosrun rqt_tf_tree rqt_tf_tree
```

To view published messages:
查看发布者的消息--可用
```
rostopic --help
rostopic list
qjm@qjm-Legion-Y9000P-IAH7H:~$ rostopic echo /visual_flied/idsi_topic
```

To install PyTorch:
安装pytorch
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ sudo apt install python3-pip
qjm@qjm-Legion-Y9000P-IAH7H:~$ pip3 install torch
qjm@qjm-Legion-Y9000P-IAH7H:~$ pip3 install tensorboard
```

To set CPU frequency:
设置cpu频率
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ cpupower-gui
qjm@qjm-Legion-Y9000P-IAH7H:~$ cpufreq-info
```
## 5.Robot Starting Positions

Modify the starting positions of the robots in:
机器人出发位置在environment_stage_5.py,处修改
```
/home/qjm/orca/turtlebot3/src/multi_robot_drl_6robots/src/environment_stage_5.py
def reset_vars(self):
```

## 6.Modifying the Number of Robots
修改机器人数量的方法

For example, to change from 6 robots to 10 robots:

    (1) Modify the launch files.

    (2) Copy the ids_multi_robot_drl_6robots folder and update the CMake and package files with the new project name.

    (3) Modify the number of robots in environment_stage_5.py.

    (4) Modify the number of robots in sac.py.

    (5) Modify the number of robots in orca.

    (6) Adjust the Gazebo Physics solver settings to speed up training by changing max_step_size from 0.003 to 0.004.
例如将6个机器人改为10个，修改流程如下：
(1)修改launch文件
(2)复制ids_multi_robot_drl_6robots文件夹，并对应修改cmake，package文件中的文件名称。
(3)修改environment_stage_5.py程序中对应数量
(4)修改sac.py程序中对应数量
(5)修改orca中对应数量
(6)修改gazebo窗口中的Physics求解器，将max_step_size由0.003改为0.004，可以加快训练速度


    (7) Check GPU usage to ensure it is fully utilized:
(7)查看gpu显卡使用功率，命令行，看是否充分使用
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ cpupower-gui 
```


## 7.VRPN Installation

To install VRPN for real robot experiments:
本机安装vrpn方法--用于跑实物实验
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ sudo apt install ros-noetic-vrpn
```
Install the necessary files:
安装相应文件
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ sudo apt install ros-noetic-vrpn-client-ros

qjm@qjm-Legion-Y9000P-IAH7H:~$ roslaunch vrpn_client_ros sample.launch
```
Install the necessary files:
安装相应文件
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ roscd vrpn_client_ros/launch/

qjm@qjm-Legion-Y9000P-IAH7H:/opt/ros/noetic/share/vrpn_client_ros/launch$ sudo apt install vim
```

Configure the IP address:
配置相关IP
```
qjm@qjm-Legion-Y9000P-IAH7H:/opt/ros/noetic/share/vrpn_client_ros/launch$ roslaunch vrpn_client_ros sample.launch server:=192.168.1.29
```

useless!
（以下没有用--这些是编辑模式）
```
qjm@qjm-Legion-Y9000P-IAH7H:/opt/ros/noetic/share/vrpn_client_ros/launch$ sudo vim ./sample.launch 

qjm@qjm-Legion-Y9000P-IAH7H:/opt/ros/noetic/share/vrpn_client_ros/launch$ sudo vim ./sample.launch server:=192.168.1.29
```

## 8.Network Issues with TB3 Robots

To resolve network connection issues with TB3 robots:

Access the Huawei router web interface (password: qiujiaxin):
    
TB3机器人连不上解决网络问题

华为路由器，网页打开（密码为qiujiaxin）
```
http://192.168.1.1/html/index.html#/login
```

Check the 2.4G devices under "Terminal Management."
Ensure the host machine's IP is correctly set:
终端管理——>2.4G设备
本电脑的网络IP为
```
cat: 'http://192.168.1.6:11311': No such file or directory
```

Check the IP address on the TB3 Raspberry Pi:
查找TB3树莓派上的IP地址
```
turtlebot@T4:~$ cat $ROS_MASTER_URI 
```

It should display (important, easy to forget! If incorrect, the connection will fail):
应显示为（重点，容易遗忘！！！！如果不对连不上）
```
cat: 'http://192.168.1.6:11311': No such file or directory
```

Ping a robot to check connectivity:
ping某个机器人，看是否能通信？？
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ ping 192.168.1.126
```

## 9.Changing IP Addresses

To prevent static IP changes in WiFi from causing LiDAR connection issues:
Edit the bashrc file on the TB3 Raspberry Pi:

修改IP的方法--（防止WIFI中静态IP经常变，导致激光雷达无法连上）
输入--打开TB3树莓派的bashrc设定，对应改IP
```
turtlebot@T4:~$ vim ~/.bashrc
```
Press i to enter insert mode, make changes, then press Esc and type :wq to save and exit.
Refresh the Raspberry Pi:
输入i键盘,出现光标,改好后按esc键退出
先按esc，键盘打一个“：”,输入wq（意思是保存退出）
最后刷新下树莓派
```
turtlebot@T4:~$ source ~/.bashrc
```
Check the list of topics:
查看有哪些话题list--
```
rostopic list 
```
Check if the LiDAR is connected:
查看rostopic有无数据，看雷达等是否连上？(1号直接为burger)
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ rostopic echo /burger3/scan 
qjm@qjm-Legion-Y9000P-IAH7H:~$ rostopic echo /burger/odom
```


## 10.Detailed Steps for Real Robot Experiments

Ensure the host machine's IP is correct by running roscore:

本机电脑启动实物实验--详细版
用roscore查看本机IP是否正确？？---实物实验运行时候，roscore一定要开
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ roscore
```

Note: Do not run roscore when launching new launch files.
Start VRPN with the correct IP:
注意！！其中开新的launch文件不要开任何roscore
启动vrpn链接地址（安装文件）
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ roslaunch vrpn
qjm@qjm-Legion-Y9000P-IAH7H:~$ roslaunch vrpn_client_ros sample.launch
qjm@qjm-Legion-Y9000P-IAH7H:/opt/ros/noetic/share/vrpn_client_ros/launch$ sudo apt install vim
qjm@qjm-Legion-Y9000P-IAH7H:/opt/ros/noetic/share/vrpn_client_ros/launch$ sudo vim ./sample.launch 
qjm@qjm-Legion-Y9000P-IAH7H:/opt/ros/noetic/share/vrpn_client_ros/launch$ sudo vim ./sample.launch server:=192.168.1.29
qjm@qjm-Legion-Y9000P-IAH7H:/opt/ros/noetic/share/vrpn_client_ros/launch$ roslaunch vrpn_client_ros sample.launch server:=192.168.1.29
```

The above are all installation files. If the installation is already completed, VRPN only requires the following single line of code.
以上都是安装文件，如果已安装好，vrpn只需要以下一行
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ roslaunch vrpn_client_ros sample.launch server:=192.168.1.29
```

其中，刚体是从编号1-6，Rigid1-Rigid6

为了能运行TB3实验,改了本机的bashrc文件，网络只能连到tiger_exp
注意运行所有程序，都是这样！！！


Connect to each robot's LiDAR (from 1 to 6, password: 8888):
链接各机器人的激光雷达（从1到6逐个链接，逐个启动,m密码是8888）
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ ssh turtlebot@192.168.1.121
turtlebot@T1:~$ roslaunch turtlebot3_bringup turtlebot3_robot_rplidar.launch
```
If the LiDAR performance is poor, restart the robot, check the connections, or clean the SD cards.
For experiments without LiDAR (e.g., Fang Ziyi's experiment):
效果不好要重新启动机器人板子，或者检查接线，擦擦SD卡
(如果不用激光雷达，只用TB3的运动功能，用该命令，比如方子怡的实验--230925)
```
turtlebot@T1:~$ roslaunch turtlebot3_bringup turtlebot3_remote_robot.launch
```

Run the robot with:
此外，机器人启动文件也不一样，应为如下
```
qjm@qjm-Legion-Y9000P-IAH7H:~/orca/turtlebot3/src/idsi_multi_robot_drl_6robots/src$ python3 ./sac_on_real.py
```

To test speed, send velocity commands to a specific robot:
实验用测试速度，给某个指定机器人速度
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ rostopic pub /burger5/cmd_vel geometry_msgs/Twist "linear:
```
Press Tab to autofill the velocity and angular velocity settings.
再按tab键调出速度角速度设置




## 11.Gmapping for Map Building

First, launch the environment:

gmapping建图
要先启动环境场景
```
roslaunch turtlebot3_gazebo turtlebot3_house.launch 
```

Modify the world file for the desired SLAM map:
对应的在此处修改world，为要slam建图的world场景
```
/home/qjm/orca/turtlebot3/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
```

Open RViz for mapping:
打开rviz对应模块
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

Control the robot with the keyboard:
键盘控制前进
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

## 12.Modifying World Files for Faster Training
To speed up training, modify the world file's update rate:
修改world文件里面的更新频率，让训练频率更快--以下该段复制粘贴过去
```
      <real_time_update_rate>0.0</real_time_update_rate>
      <max_step_size>0.003</max_step_size>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
```

To modify read-only files (errors may occur, but changes will be applied):
其中，修改只读文件的权限--虽然会报错无法访问，但确实可成功修改--240213
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ cd /home/qjm/orca/turtlebot3/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
qjm@qjm-Legion-Y9000P-IAH7H:~/orca/turtlebot3/src/n/turtlebot3_gazebo/worlds$ chown --help
qjm@qjm-Legion-Y9000P-IAH7H:~/orca/turtlebot3/src/turtlebot3_simulations/turtlebot3_gazebo/worlds$ sudo chown qjm qjm ./*
```

## 13.Configuring bashrc for Simulation/Real Experiments

Edit the bashrc file to switch between simulation and real robot experiments:
配置本机的bashrc，用于更改仿真/实物试验时的IP
命令行打开bashrc
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ gedit ~/.bashrc
```

Comment out one of the following sections based on the experiment type:
For real robot experiments:
修改bashrc里的文件，对应注释掉其中一个
实物实验用IP，230728
```
export ROS_HOSTNAME=192.168.1.6
export ROS_MASTER_URI=http://192.168.1.6:11311
```

For local simulations:
平时的本机IP，230928
```
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```

Update the configuration:
更新下相关地址文件
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ source ~/.bashrc
```

Check the host machine's IP with roscore (required for real robot experiments):
用roscore查看本机IP是否正确？？---实物实验运行时候，roscore一定要开
```
qjm@qjm-Legion-Y9000P-IAH7H:~$ roscore
```

Note: Do not run roscore when launching new launch files.
注意！！其中开新的launch文件不要开任何roscore

