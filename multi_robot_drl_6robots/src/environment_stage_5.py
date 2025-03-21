#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from std_msgs.msg import Float32MultiArray #测试用，230811
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
import random
import copy
target_not_movable = False

class Env():
    def __init__(self, ext_shutdown, action_dim=2):
        self.reset_vars()

        self.pub_cmd_vel = [
            rospy.Publisher('/tb3_0/sac_cmd_vel', Twist, queue_size=5),#sac_cmd_vel
            rospy.Publisher('/tb3_1/sac_cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/tb3_2/sac_cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/tb3_3/sac_cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/tb3_4/sac_cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/tb3_5/sac_cmd_vel', Twist, queue_size=5)
        ]

        # 创建计数器和循环频率控制器--用于打印位置
        self.counter = 0
        self.rate = rospy.Rate(1 / 10)  # 指定循环频率为10次/秒
        self.sub_odom = [
            rospy.Subscriber('/tb3_0/odom', Odometry, self.odometryCallback, 0),
            rospy.Subscriber('/tb3_1/odom', Odometry, self.odometryCallback, 1),
            rospy.Subscriber('/tb3_2/odom', Odometry, self.odometryCallback, 2),
            rospy.Subscriber('/tb3_3/odom', Odometry, self.odometryCallback, 3),
            rospy.Subscriber('/tb3_4/odom', Odometry, self.odometryCallback, 4),
            rospy.Subscriber('/tb3_5/odom', Odometry, self.odometryCallback, 5)
        ]
        #测试用，不起到实际意义，230811
        self.sub_idsi = [
            rospy.Subscriber('/tb3_0/idsi_topic', Float32MultiArray, self.idsiCallback, 0),
            rospy.Subscriber('/tb3_1/idsi_topic', Float32MultiArray, self.idsiCallback, 1),
            rospy.Subscriber('/tb3_2/idsi_topic', Float32MultiArray, self.idsiCallback, 2),
            rospy.Subscriber('/tb3_3/idsi_topic', Float32MultiArray, self.idsiCallback, 3),
            rospy.Subscriber('/tb3_4/idsi_topic', Float32MultiArray, self.idsiCallback, 4),
            rospy.Subscriber('/tb3_5/idsi_topic', Float32MultiArray, self.idsiCallback, 5)
        ]

        self.set_model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=32)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.ext_shutdown = ext_shutdown

        rospy.on_shutdown(self.shutdown)
    
    def reset_vars(self):
        self.goal = [

            [2.45, 1.95],  #240213--IROS24-4?实验论文--目标位置
            [3.19, 0.14],
            [-0.75, -1.45],
            [-2.12, -0.67],
            [-0.86, 1.12],
            [-0.88, 1.94]

            # [3.02, 0.49],  #240213--IROS24-3实验论文--目标位置
            # [2.25, -1.52],
            # [-0.86, -1.75],
            # [-2.14, -0.39],
            # [-2.07, 1.21],
            # [0.83, 1.40]

            # [2.6, 0.62],  #231108--TIE2310-2实验论文--目标位置
            # [2.52, -0.96],
            # [-2.35, -0.84],
            # [-2.15, 1.3],
            # [2.66, 1.13],
            # [-0.25, 1.48]

            # [1.08, -1.53],  #231031----RAL2310-2实验论文--目标位置
            # [-1.07, -1.69],
            # [-2.20, -1.52],
            # [-1.75, 0.50],
            # [-1.81, 1.26],
            # [2.72, 0.65]

            # [1.08, -1.53],  #231027----RAL2310-1实验论文--目标位置
            # [-1.07, -1.69],
            # [-1.86, -1.16],
            # [-2.20, 1.04],
            # [-1.81, 1.26],
            # [1.58, 0.45]

            # [1.47, 0.85],  #230831--RAL论文--泛化实验，目标位置
            # [1.86, -0.39],
            # [1.40, -0.68],
            # [-0.46, -0.79],
            # [-1.92, 0.79],
            # [-1.06, 1.19]

            # [3.11, -1.87],  #230830--TIE论文目标位置
            # [-3.08, -1.77],
            # [-0.94, -1.67],
            # [-2.93, 1.46],
            # [-0.47, -1.97],
            # [2.62, -1.48]
        ] #[x,y]
        self.start_range = [

        ]
        self.start_pose = [
            # [-2.76, 0.10, 0.50],  #230831--RAL论文--泛化实验，出发位置 -2.40
            # [-2.22, -0.12, -0.11],#-2.76
            # [-0.35, 1.65, -1.46],#-0.41, 1.60, -1.46
            # [0.05, 1.60, -2.05],#0.14, 1.66, -2.05
            # [2.00, 0.50, -2.97],
            # [0.83, -1.06, -0.67]

            [-1.64, -1.52, -0.01], #240213--IROS24-4?实验论文--出发位置
            [-1.90, 0.67, -1.46],
            [2.05, 1.50, -2.78],
            [3.12, 1.26, 1.8],
            [3.28, -0.73, 1.84],
            [1.63, -1.81, 0.02]#1.44, -1.81, 0.02

            # [-1.93, -1.53, -0.01], #240213--IROS24-3实验论文--出发位置
            # [-1.93, 0.95, -1.46],
            # [1.98, 1.72, -2.78],
            # [2.70, -0.19, 1.8],
            # [2.83, -1.72, 1.84],
            # [-0.41, -1.36, 1.14]

            # [-2.51, 0.21, -0.23], #231108--TIE2310-2实验论文--出发位置
            # [-1.25, 1.6, -1.46],
            # [2.26, 0.88, -2.78],
            # [2.12, -1.15, 1.8],
            # [0.05, -1.60, 1.84],
            # [-1.37, -1.49, 1.14]

            # [-2.68, -0.06, -0.23], #231101--RAL2310-2实验论文--出发位置
            # [-0.80, 1.6, -1.46],
            # [2.45, 1.28, -2.78],
            # [2.58, -1.43, 0.0],
            # [0.11, -1.60, 1.84],
            # [-1.43, -1.49, 1.14]


            # [-2.65, 0.14, -0.23], #231027--RAL2310-1实验论文--出发位置
            # [-0.80, 1.6, -1.46],
            # [2.70, -0.20, -2.78],
            # [2.58, -1.43, 0.0],
            # [0.11, -1.60, 1.84],
            # [-1.43, -1.49, 1.14]

            # [-0.71, 0.24, 1.59], #230830--TIE论文出发位置
            # [-0.36, -0.34, 1.60],
            # [-2.79, -0.76, 1.64],
            # [2.98, 0.36, 1.68],
            # [2.84, -0.57, 1.65],
            # [-2.86, 1.00, -0.09]
        ] #[x,y,yaw]
        self.pose = self.start_pose.copy()
        self.last_pose = self.start_pose.copy()
        self.vel = [[0.0, 0.0] for i in range(6)] #linear.x angular.z
        self.idsi = [[10.0 for j in range(6)] for i in range(6)]   
        self.distance_to_goal = [0.0 for i in range(6)]
        self.last_distance_to_goal = self.distance_to_goal.copy()
        self.arrive = [False for i in range(6)]
        self.spend_yaw = [0.0 for i in range(6)]
        self.stopped_cnt = [0 for i in range(6)]
        self.reset_time = rospy.Time.now()
        self.done = [False for i in range(6)]
        self.hit_obs = [False for i in range(6)]
        self.min_range = [9999 for i in range(6)]
        self.min_range_thr = 0.136
        self.min_range_thr2 = 0.226
        self.steps = [0 for i in range(6)]

    def shutdown(self):
        print("Stopping TurtleBot")
        # for i in range(4):
        #     self.pub_cmd_vel[i].publish(Twist())
        self.ext_shutdown()
        # rospy.sleep(1)

    def odometryCallback(self, odom, arg):
        self.last_pose[arg] = copy.deepcopy(self.pose[arg])
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.pose[arg] = [odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]
        self.vel[arg] = [odom.twist.twist.linear.x, odom.twist.twist.angular.z]

        # # 在回调函数中添加计数器并打印--用来论文画图，重要，IROS--230226
        # self.counter += 1
        # if self.counter % 10 == 0:
        #     # print('Robot = ' + str(arg) + ', Pos_X = ' + str(self.pose[arg][0]) +', Pos_Y = ' + str(self.pose[arg][1]) + ', linear_X = ' + str(self.vel[arg][0]) +', angular_Z = ' + str(self.vel[arg][1]) )
        #     print( str(arg) + ',' + str(self.pose[arg][0]) +',' + str(self.pose[arg][1]) + ',' + str(self.vel[arg][0]) +',' + str(self.vel[arg][1]) )  #+ "!!!!!!!" 
        #     #+6
        # # # 控制循环频率--这个和上面的打印无关--230811
        # # self.rate.sleep()

        if abs(self.pose[arg][2] - self.last_pose[arg][2]) < pi / 2:
            self.spend_yaw[arg] += (self.pose[arg][2] - self.last_pose[arg][2])
        elif self.pose[arg][2] < self.last_pose[arg][2]:
            self.spend_yaw[arg] += (self.pose[arg][2] + 2 * pi - self.last_pose[arg][2])
        elif self.pose[arg][2] > self.last_pose[arg][2]:
            self.spend_yaw[arg] -= (-self.pose[arg][2] + 2 * pi + self.last_pose[arg][2])
    
    #测试用，不起到实际意义，230811
    def idsiCallback(self, idsi, arg):
        for i in range(6):
            if idsi.data[i] == float('nan'):
                self.idsi[arg][i] = 0.1
                print("There is a nan!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif idsi.data[i] == float('inf'):
                if idsi.data[i] > 0:
                    idsi.data[i] = 40
                else:
                    idsi.data[i] = -40
                print("There is a inf!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            else:
                self.idsi[arg][i] = idsi.data[i]
        # print(" i = " + str(arg) + ", " )
        print( str(arg) + ", " + str(self.idsi[arg])) #" i = " + str(arg) + ", " +

    def angle(self, p1, p2):
        goal_angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        heading = goal_angle - p1[2]
        if heading > pi:
            heading -= 2 * pi
        elif heading < -pi:
            heading += 2 * pi
        heading = round(heading, 3)
        return heading

    def getState(self, scan, past_action):
        state = [[] for i in range(6)]
        for n in range(6):
            goal_angle = math.atan2(self.goal[n][1] - self.pose[n][1], self.goal[n][0] - self.pose[n][0])
            heading = goal_angle - self.pose[n][2]
            if heading > pi:
                heading -= 2 * pi
            elif heading < -pi:
                heading += 2 * pi
            heading = round(heading, 3)

            scan_range = []
            for i in range(len(scan[n].ranges)):
                if scan[n].ranges[i] == float('Inf') or scan[n].ranges[i] == float('inf'):
                    scan_range.append(3.5)
                elif np.isnan(scan[n].ranges[i]) or scan[n].ranges[i] == float('nan'):
                    scan_range.append(0)
                else:
                    scan_range.append(scan[n].ranges[i])

            self.min_range[n] = min(scan_range)
            if self.min_range_thr > self.min_range[n] > 0:
                self.hit_obs[n] = True

            for pa in past_action[n]:
                scan_range.append(pa)

            current_distance = round(math.hypot(self.goal[n][0] - self.pose[n][0], self.goal[n][1] - self.pose[n][1]), 2)
            self.last_distance_to_goal[n] = self.distance_to_goal[n]
            self.distance_to_goal[n] = current_distance
            if current_distance < 0.10:
                self.arrive[n] = True
            
            me = r1 = r2 = r3 = r4 = r5 = 0
            if n == 0:
                me = 0
                r1 = 1
                r2 = 2
                r3 = 3
                r4 = 4
                r5 = 5
            elif n == 1:
                me = 1
                r1 = 0
                r2 = 2
                r3 = 3
                r4 = 4
                r5 = 5
            elif n == 2:
                me = 2
                r1 = 0
                r2 = 1
                r3 = 3
                r4 = 4
                r5 = 5
            elif n == 3:
                me = 3
                r1 = 0
                r2 = 1
                r3 = 2
                r4 = 4
                r5 = 5
            elif n == 4:
                me = 4
                r1 = 0
                r2 = 1
                r3 = 2
                r4 = 3
                r5 = 5
            elif n == 5:
                me = 5
                r1 = 0
                r2 = 1
                r3 = 2
                r4 = 3
                r5 = 4
            obs = [
                round(math.hypot(self.pose[me][0] - self.pose[r1][0], self.pose[me][1] - self.pose[r1][1]), 2),
                round(math.hypot(self.pose[me][0] - self.pose[r2][0], self.pose[me][1] - self.pose[r2][1]), 2),
                round(math.hypot(self.pose[me][0] - self.pose[r3][0], self.pose[me][1] - self.pose[r3][1]), 2),
                round(math.hypot(self.pose[me][0] - self.pose[r4][0], self.pose[me][1] - self.pose[r4][1]), 2),
                round(math.hypot(self.pose[me][0] - self.pose[r5][0], self.pose[me][1] - self.pose[r5][1]), 2),
                self.angle(self.pose[me], self.pose[r1]),
                self.angle(self.pose[me], self.pose[r2]),
                self.angle(self.pose[me], self.pose[r3]),
                self.angle(self.pose[me], self.pose[r4]),
                self.angle(self.pose[me], self.pose[r5]),
                self.pose[me][2],
                self.pose[r1][2],
                self.pose[r2][2],
                self.pose[r3][2],
                self.pose[r4][2],
                self.pose[r5][2],
            ]
            state[n] = scan_range + \
                [heading, current_distance] + \
                self.vel[me] + self.vel[r1] + self.vel[r2] + self.vel[r3] + self.vel[r4] + self.vel[r5] + obs
            # elif n == 1:
            #     state[n] = scan_range + \
            #         [heading, current_distance] + \
            #         self.vel[1] + self.vel[0] + self.vel[2] + self.vel[3] +\
            #         self.pose[1] + self.pose[0] + self.pose[2] + self.pose[3]
            # elif n == 2:
            #     state[n] = scan_range + \
            #         [heading, current_distance] + \
            #         self.vel[2] + self.vel[0] + self.vel[1] + self.vel[3] +\
            #         self.pose[2] + self.pose[0] + self.pose[1] + self.pose[3]
            # elif n == 3:
            #     state[n] = scan_range + \
            #         [heading, current_distance] + \
            #         self.vel[3] + self.vel[0] + self.vel[1] + self.vel[2] +\
            #         self.pose[3] + self.pose[0] + self.pose[1] + self.pose[2]

        return state

    def setReward(self, state, action, past_action):
        reward = [0.0 for i in range(6)]
        for n in range(6):
            if self.done[n] == False:
                distance_rate = (self.last_distance_to_goal[n] - self.distance_to_goal[n]) 
                if distance_rate > 0:
                    reward[n] = distance_rate * 10

                if distance_rate <= 0:
                    reward[n] = distance_rate * 11
                
                # if self.min_range[n] < self.min_range_thr2 and self.min_range[n] >= self.min_range_thr:
                #     reward[n] -= 4.0 * (((self.min_range_thr2 - self.min_range[n]) / (self.min_range_thr2 - self.min_range_thr)) ** 2)
                # elif self.min_range[n] < self.min_range_thr:
                #     reward[n] -= 5.0
                
                # action_dt = [
                #     abs(action[n][0] - past_action[n][0]),
                #     abs(action[n][1] - past_action[n][1])
                # ]
                # reward[n] -= (action_dt[0] * 0.1 / 0.28 + action_dt[1] * 0.1 / 2.0)
                
                if self.hit_obs[n]:
                    self.done[n] = True

                if self.done[n] == False:
                    a, b, c, d = float('{0:.3f}'.format(self.pose[n][0])), \
                        float('{0:.3f}'.format(self.last_pose[n][0])), \
                        float('{0:.3f}'.format(self.pose[n][1])), \
                        float('{0:.3f}'.format(self.last_pose[n][1]))
                    if a == b and c == d:
                        self.stopped_cnt[n] += 1
                        if self.stopped_cnt[n] == 100:
                            rospy.loginfo('Robot' + str(n) + ' is in the same 100 times in a row')
                            self.stopped_cnt[n] = 0
                            self.done[n] = True
                            reward[n] = -10
                    elif abs(self.spend_yaw[n]) > 2 * pi * 2.0: #原为8.0
                        rospy.loginfo('Robot' + str(n) + ' spins too much')
                        self.stopped_cnt[n] = 0
                        self.done[n] = True
                        reward[n] = -10
                    elif self.steps[n] > 150:
                        rospy.loginfo('Robot' + str(n) + ' spend too much time')
                        self.stopped_cnt[n] = 0
                        self.done[n] = True
                        reward[n] = -10
                    else:
                        self.stopped_cnt[n] = 0
                else:
                    rospy.loginfo('Robot' + str(n) + ' hit an obstacle')
                    reward[n] = -10
                    self.done[n] = True
                    self.pub_cmd_vel[n].publish(Twist())
            
                if self.arrive[n]:
                    #rospy.loginfo('*******Robot' + str(n) + ' reaches the goal*******')
                    reward[n] = 200 - self.steps[n] * 0.1
                    self.done[n] = True
                    self.pub_cmd_vel[n].publish(Twist())

        return reward, self.done

    def step(self, action, past_action):
        for i in range(6):
            linear_vel = action[i][0]
            ang_vel = action[i][1]
            vel_cmd = Twist()
            vel_cmd.linear.x = linear_vel + np.random.normal(0, 0.02) #在此处增加了高斯噪声，让学习的更鲁棒
            vel_cmd.angular.z = ang_vel+ np.random.normal(0, 0.2)
            self.pub_cmd_vel[i].publish(vel_cmd)
            self.steps[i] += 1

        data = [None for i in range(6)]
        for i in range(6):
            while data[i] is None and not rospy.is_shutdown():
                try:
                    data[i] = rospy.wait_for_message('/tb3_'+str(i)+'/scan', LaserScan, timeout=5)
                except:
                    pass
        if rospy.is_shutdown():
            exit()

        state = self.getState(data, past_action)
        reward, done = self.setReward(state, action, past_action)

        return [np.asarray(state[n]) for n in range(6)], reward, done, self.arrive

    def reset(self):
        # rospy.wait_for_service('gazebo/reset_world', timeout=5)
        # try:
        #     self.reset_proxy()
        # except (rospy.ServiceException) as e:
        #     print("gazebo/reset_simulation service call failed")
        for i in range(6):
            model = ModelState()
            model.model_name = 'tb3_' + str(i)
            # x = random.uniform(self.start_range[i][0][0], self.start_range[i][1][0])
            # y = random.uniform(self.start_range[i][0][1], self.start_range[i][1][1])
            x = self.start_pose[i][0] + random.uniform(-0.15,0.15) #231027新增随机数噪声--增强泛化性能
            y = self.start_pose[i][1] + random.uniform(-0.15,0.15) #231027新增随机数噪声--增强泛化性能
            model.pose.position = Point(x, y, 0)
            # orien = quaternion_from_euler(0, 0, random.uniform(60.0, 120.0) * pi / 180.0)
            orien = quaternion_from_euler(0, 0, self.start_pose[i][2])
            model.pose.orientation = Quaternion(orien[0], orien[1], orien[2], orien[3])
            self.set_model_pub.publish(model)

        data = [None for i in range(6)]
        for i in range(6):
            while data[i] is None and not rospy.is_shutdown():
                try:
                    data[i] = rospy.wait_for_message('/tb3_'+str(i)+'/scan', LaserScan, timeout=5)
                except:
                    pass
        print("Episode is over !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        if rospy.is_shutdown():
            exit()

        self.reset_vars()
        state = self.getState(data, [[0.0, 0.0] for i in range(6)])
        
        return [np.asarray(state[n]) for n in range(6)]
