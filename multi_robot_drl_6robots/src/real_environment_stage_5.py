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
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
import copy
import math
target_not_movable = False

class Env():
    def __init__(self, action_dim=2):
        self.reset_vars()

        self.pub_cmd_vel = [
            rospy.Publisher('/burger/cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/burger2/cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/burger3/cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/burger4/cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/burger5/cmd_vel', Twist, queue_size=5),
            rospy.Publisher('/burger6/cmd_vel', Twist, queue_size=5)
        ]
        self.sub_odom = [
            rospy.Subscriber('/burger/odom', Odometry, self.odometryCallback, 0),
            rospy.Subscriber('/burger2/odom', Odometry, self.odometryCallback, 1),
            rospy.Subscriber('/burger3/odom', Odometry, self.odometryCallback, 2),
            rospy.Subscriber('/burger4/odom', Odometry, self.odometryCallback, 3),
            rospy.Subscriber('/burger5/odom', Odometry, self.odometryCallback, 4),
            rospy.Subscriber('/burger6/odom', Odometry, self.odometryCallback, 5)
        ]
        self.sub_pose = [
            rospy.Subscriber('/vrpn_client_node/Rigid1/pose', PoseStamped, self.poseCallback, 0),
            rospy.Subscriber('/vrpn_client_node/Rigid2/pose', PoseStamped, self.poseCallback, 1),
            rospy.Subscriber('/vrpn_client_node/Rigid3/pose', PoseStamped, self.poseCallback, 2),
            rospy.Subscriber('/vrpn_client_node/Rigid4/pose', PoseStamped, self.poseCallback, 3),
            rospy.Subscriber('/vrpn_client_node/Rigid5/pose', PoseStamped, self.poseCallback, 4),
            rospy.Subscriber('/vrpn_client_node/Rigid6/pose', PoseStamped, self.poseCallback, 5)
        ]
        self.sub_scan = [
            rospy.Subscriber('/burger/scan', LaserScan, self.scanCallback, 0),
            rospy.Subscriber('/burger2/scan', LaserScan, self.scanCallback, 1),
            rospy.Subscriber('/burger3/scan', LaserScan, self.scanCallback, 2),
            rospy.Subscriber('/burger4/scan', LaserScan, self.scanCallback, 3),
            rospy.Subscriber('/burger5/scan', LaserScan, self.scanCallback, 4),
            rospy.Subscriber('/burger6/scan', LaserScan, self.scanCallback, 5)
        ]
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        rospy.on_shutdown(self.shutdown)
    
    def reset_vars(self):
        self.goal = [
            # [-0.28, -1.17],
            # [0.74, -1.60],
            # [-1.10, -0.14],
            # [-2.60, -0.22],
            # [0.18, -1.63],
            # [0.28, 0.61]
            # [-0.2, 1.11],
            # [0.15, 1.55],
            # [-1.5, -0.2],
            # [-1.0, 0.2],
            # [0.0, -1.54],
            # [1.51, -0.22]
            
            # [-1.5, -0.17],
            # [-1.2, 0.21],
            # [1.3, -0.13],
            # [1.12, 0.18],
            # [0.2, -1.3],
            # [0.0, 1.5]

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

            # [-1.0433609619140625, -0.2237098541259765],
            # [-2.4360283203125, -0.27485211181640623],
            # [-0.01739626121520996, -1.196630615234375],
            # [0.45964913940429686, -1.451390869140625],
            # [0.873778564453125, -1.773697021484375],
            # [0.22241242218017578, 0.5779932250976563]

        ] #[x,y]
        self.start_pose = [
            # [-2.65, -0.18, 0.0],
            # [-2.40, -0.63, 0.0],
            # [2.80, -0.21, -3.14],
            # [2.96, -0.63, -3.14],
            # [-0.26, 1.79, -1.57],
            # [0.52, -1.62, 1.57]

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

            # [-2.76, 0.10, 0.50],  #2300907--RAL论文--泛化实验，出发位置 -2.40
            # [-2.22, -0.12, -0.11],#-2.76
            # [-0.35, 1.65, -1.46],#-0.41, 1.60, -1.46
            # [0.05, 1.60, -2.05],#0.14, 1.66, -2.05
            # [2.00, 0.50, -2.97],
            # [0.83, -1.06, -0.67]

            # [-0.17, -1.5, 1.57], #230907--之前的旧实验
            # [0.21, -1.2, 1.57],
            # [1.3, -0.2, -3.0],
            # [0.18, 1.12, -1.57],
            # [0.13, 1.5, -1.57],
            # [-1.5, -0.0, 0.0]
        ] #[x,y,yaw]
        self.pose = self.start_pose.copy()
        self.last_pose = self.start_pose.copy()
        self.vel = [[0.0, 0.0] for i in range(6)] #linear.x angular.z
        self.distance_to_goal = [0.0 for i in range(6)]
        self.last_distance_to_goal = self.distance_to_goal.copy()
        self.arrive = [False for i in range(6)]
        self.spend_yaw = [0.0 for i in range(6)]
        self.stopped_cnt = [0 for i in range(6)]
        self.reset_time = rospy.Time.now()
        self.done = [False for i in range(6)]
        self.hit_obs = [False for i in range(6)]
        self.min_range = [9999 for i in range(6)]
        self.min_range_thr = 0.14
        self.min_range_thr2 = 0.236
        self.steps = [0 for i in range(6)]
        self.scan_data = [LaserScan() for i in range(6)]
        self.scan_data_init = False

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        for i in range(6):
            self.pub_cmd_vel[i].publish(Twist())
        rospy.sleep(1)
        # exit()
    
    def scanCallback(self, msg, arg):
        # rospy.loginfo("new_scan%d", arg)
        self.scan_data_init = True
        self.scan_data[arg] = msg

    def odometryCallback(self, odom, arg):
        # rospy.loginfo("new_odom%d", arg)
        self.vel[arg] = [odom.twist.twist.linear.x, odom.twist.twist.angular.z]
    
    def poseCallback(self, pose, arg):
        # print("new_pose%d", arg)
        self.last_pose[arg] = copy.deepcopy(self.pose[arg])
        orientation = pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        yaw += pi / 2.0
        if yaw > pi:
            yaw -= 2 * pi
        self.pose[arg] = [-pose.pose.position.x, -pose.pose.position.y, yaw]

        if abs(self.pose[arg][2] - self.last_pose[arg][2]) < pi / 2:
            self.spend_yaw[arg] += (self.pose[arg][2] - self.last_pose[arg][2])
        elif self.pose[arg][2] < self.last_pose[arg][2]:
            self.spend_yaw[arg] += (self.pose[arg][2] + 2 * pi - self.last_pose[arg][2])
        elif self.pose[arg][2] > self.last_pose[arg][2]:
            self.spend_yaw[arg] -= (-self.pose[arg][2] + 2 * pi + self.last_pose[arg][2])
        # if arg == 3:
        # print(arg, " ", self.pose[arg])
    
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
            for j in range(360):
                i = int((j / 362.0) * len(scan[n].ranges))
                # print(n, " ", i, " ", len(scan[n].ranges))
                if scan[n].ranges[i] == float('Inf') or scan[n].ranges[i] == float('inf'):
                    scan_range.append(0.13)
                elif np.isnan(scan[n].ranges[i]) or scan[n].ranges[i] == float('nan'):
                    scan_range.append(0.13)
                else:
                    scan_range.append(scan[n].ranges[i])

            self.min_range[n] = min(scan_range)
            # if self.min_range_thr > self.min_range[n] > 0:
            #     self.hit_obs[n] = True

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
                
                if self.min_range[n] < self.min_range_thr2 and self.min_range[n] >= self.min_range_thr:
                    reward[n] -= 4.0 * (((self.min_range_thr2 - self.min_range[n]) / (self.min_range_thr2 - self.min_range_thr)) ** 2)
                elif self.min_range[n] < self.min_range_thr:
                    reward[n] -= 5.0
                
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
                        if self.stopped_cnt[n] == 50:
                            rospy.loginfo('Robot' + str(n) + ' is in the same 100 times in a row')
                            self.stopped_cnt[n] = 0
                            self.done[n] = True
                            reward[n] = -10
                    elif abs(self.spend_yaw[n]) > 2 * pi * 8.0:
                        rospy.loginfo('Robot' + str(n) + ' spins too much')
                        self.stopped_cnt[n] = 0
                        self.done[n] = True
                        reward[n] = -10
                    # elif self.steps[n] > 250:
                    #     rospy.loginfo('Robot' + str(n) + ' spend too much time')
                    #     self.stopped_cnt[n] = 0
                    #     self.done[n] = True
                    #     reward[n] = -20
                    else:
                        self.stopped_cnt[n] = 0
                else:
                    rospy.loginfo('Robot' + str(n) + ' hit an obstacle')
                    reward[n] = -20
                    self.done[n] = True
                    self.pub_cmd_vel[n].publish(Twist())
            
                if self.arrive[n]:
                    rospy.loginfo('*******Robot' + str(n) + ' reaches the goal*******')
                    reward[n] = 100
                    self.done[n] = True
                    self.pub_cmd_vel[n].publish(Twist())

        return reward, self.done

    def step(self, action, past_action):
        for i in range(6):
            linear_vel = action[i][0]
            ang_vel = action[i][1]
            vel_cmd = Twist()
            vel_cmd.linear.x = linear_vel
            vel_cmd.angular.z = ang_vel
            # if i == 3:
            self.pub_cmd_vel[i].publish(vel_cmd)
            self.steps[i] += 1

        # data = [None for i in range(6)]
        # for i in range(1,2):
        #     while data[i] is None and not rospy.is_shutdown():
        #         try:
        #             data[i] = rospy.wait_for_message('/burger'+str(i+1)+'/scan', LaserScan, timeout=5)
        #         except:
        #             pass
        # data[0] = data[2] = data[3] = data[1]
        while not self.scan_data_init and not rospy.is_shutdown():
            rospy.sleep(1)
        # self.scan_data[0] = self.scan_data[2] = self.scan_data[3] = self.scan_data[1]
        state = self.getState(self.scan_data, past_action)
        reward, done = self.setReward(state, action, past_action)
        rospy.rostime.wallsleep(0.05)

        return [np.asarray(state[n]) for n in range(6)], reward, done

    def reset(self):
        # data = [None for i in range(4)]
        # for i in range(1,2):
        #     while data[i] is None and not rospy.is_shutdown():
        #         try:
        #             data[i] = rospy.wait_for_message('/burger'+str(i+1)+'/scan', LaserScan, timeout=5)
        #         except:
        #             pass
        self.reset_vars()
        while not self.scan_data_init and not rospy.is_shutdown():
            rospy.sleep(1)
        # self.scan_data[0] = self.scan_data[2] = self.scan_data[3] = self.scan_data[1]
        state = self.getState(self.scan_data, [[0.0, 0.0] for i in range(6)])
        
        return [np.asarray(state[n]) for n in range(6)]
