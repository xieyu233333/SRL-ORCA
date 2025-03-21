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
import random
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import copy
import std_msgs
target_not_movable = False

class Env():
    def __init__(self, action_dim=2):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.yaw = 0
        self.last_yaw = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('reset_simulation', Empty)
        self.pub_goal = rospy.Publisher('nav_goal', Marker, queue_size=15)
        self.past_distance = 0.
        self.stopped = 0
        self.action_dim = action_dim
        self.reset_time = rospy.Time.now()
        self.spend_distance = 0
        self.spend_yaw = 0
        self.init_yaw = False
        #Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        #you can stop turtlebot by publishing an empty Twist
        #message
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.past_distance = goal_distance

        return goal_distance

    def getOdometry(self, odom):
        self.past_position = copy.deepcopy(self.position)
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw = yaw
        if self.init_yaw:
            self.last_yaw = self.yaw
            self.init_yaw = False
        if abs(self.yaw - self.last_yaw) < pi / 2:
            self.spend_yaw += (self.yaw - self.last_yaw)
        elif self.yaw < self.last_yaw:
            self.spend_yaw += (self.yaw + 2 * pi - self.last_yaw)
        elif self.yaw > self.last_yaw:
            self.spend_yaw -= (-self.yaw + 2 * pi + self.last_yaw)
        self.last_yaw = self.yaw

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        #print 'yaw', yaw
        #print 'gA', goal_angle

        heading = goal_angle - yaw
        #print 'heading', heading
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)

    def getState(self, scan, past_action):
        scan_range = []
        heading = self.heading
        min_range = 0.165
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf') or scan.ranges[i] == float('inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]) or scan.ranges[i] == float('nan'):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
    
        if min_range > min(scan_range) > 0:
            done = True

        for pa in past_action:
            scan_range.append(pa)

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        # current_distance = self.getGoalDistace()
        if current_distance < 0.20:
            self.get_goalbox = True
        
        # print(heading, " ", current_distance)

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done):
        current_distance = state[-1]
        heading = state[-2]
        timeout = False
        #print('cur:', current_distance, self.past_distance)


        distance_rate = (self.past_distance - current_distance) 
        if distance_rate > 0:
            # reward = 200.*distance_rate
            reward = 0.

        # if distance_rate == 0:
        #     reward = 0.

        if distance_rate <= 0:
            # reward = -8.
            reward = 0.

        #angle_reward = math.pi - abs(heading)
        #print('d', 500*distance_rate)
        #reward = 500.*distance_rate #+ 3.*angle_reward
        self.past_distance = current_distance

        a, b, c, d = float('{0:.3f}'.format(self.position.x)), float('{0:.3f}'.format(self.past_position.x)), float('{0:.3f}'.format(self.position.y)), float('{0:.3f}'.format(self.past_position.y))
        if a == b and c == d and False:
            # rospy.loginfo('\n<<<<<Stopped>>>>>\n')
            # print('\n' + str(a) + ' ' + str(b) + ' ' + str(c) + ' ' + str(d) + '\n')
            self.stopped += 1
            if self.stopped == 20:
                rospy.loginfo('Robot is in the same 20 times in a row')
                self.stopped = 0
                done = True
        # elif (rospy.Time.now() - self.reset_time).to_sec() > 50.0:
        #     rospy.loginfo('Timeout')
        #     self.stopped = 0
        #     done = True
        #     timeout = True
        elif abs(self.spend_yaw) > 2 * pi * 1.5:
            self.stopped = 0
            done = True
        else:
            # rospy.loginfo('\n>>>>> not stopped>>>>>\n')
            self.stopped = 0
        
        # print((rospy.Time.now() - self.reset_time).to_sec())
        self.spend_distance += math.sqrt((a-b)**2+(c-b)**2)

        if done:
            rospy.loginfo("Collision!!")
            # reward = -500.
            reward = -10
            self.pub_cmd_vel.publish(Twist())
        
        if done and abs(self.spend_yaw) > 2 * pi * 1.5:
            reward = -50.0

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            # reward = 500.
            reward = 100
            # print(self.spend_distance)
            self.pub_cmd_vel.publish(Twist())

            self.generateGoal()
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False
            self.spend_yaw = 0
            self.init_yaw = True

        return reward, done
    
    def generateGoal(self):
        goal_x_list = [0.6, 1.9, 0.5, 0.2, -0.8, -1, -1.9, 0.5, 2, 0.5, 0, -0.1, -2]
        goal_y_list = [0, -0.5, -1.9, 1.5, -0.9, 1, 1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8]

        index = random.randrange(0, 13)

        self.goal_x = goal_x_list[index]
        self.goal_y = goal_y_list[index]

    def get_color(self, r, g, b, a = 255):
        return std_msgs.msg.ColorRGBA(r / 255.0, g / 255.0, b / 255.0, a / 255.0)
    
    def publishGoal(self):
        goal = Marker()
        goal.header.frame_id = 'real_map'
        goal.header.stamp = rospy.Time.now()
        goal.ns = 'goal'
        goal.pose.orientation.w = 1.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.action = Marker.ADD
        goal.id = 0
        goal.type = Marker.SPHERE_LIST
        goal.scale.x = 0.1
        goal.scale.y = 0.1
        goal.scale.z = 0.1
        goal.color = self.get_color(255, 255, 255)
        goal.points.clear()
        goal.points.append(Point(self.goal_x, self.goal_y, 0))
        self.pub_goal.publish(goal)

    def step(self, action, past_action):
        linear_vel = action[0]
        ang_vel = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)
        # self.spend_yaw += ang_vel * 1.0
        # print(self.spend_yaw / pi * 180.0)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data, past_action)
        reward, done = self.setReward(state, done)

        self.publishGoal()

        return np.asarray(state), reward, done

    def reset(self):
        #print('aqui2_____________---')
        rospy.wait_for_service('reset_simulation')
        print("sss")
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("reset_simulation service call failed")

        data = None
        while data is None:
            try:
                self.pub_cmd_vel.publish(Twist())
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass
        print("sss")

        if self.initGoal:
            self.generateGoal()
            self.initGoal = False
        else:
            self.generateGoal()

        self.goal_distance = self.getGoalDistace()
        state, _ = self.getState(data, [0]*self.action_dim)
        self.reset_time = rospy.Time.now()
        self.spend_distance = 0
        self.spend_yaw = 0
        self.init_yaw = True
        self.publishGoal()
        rospy.rostime.wallsleep(0.01)

        return np.asarray(state)
