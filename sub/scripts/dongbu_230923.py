#动作捕捉订阅程序--230923

#头文件--缺的头文件从里面找
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

#订阅动捕程序--注意编号开始的数字
        self.sub_pose = [
            rospy.Subscriber('/vrpn_client_node/Rigid1/pose', PoseStamped, self.poseCallback, 0),
            rospy.Subscriber('/vrpn_client_node/Rigid2/pose', PoseStamped, self.poseCallback, 1),
            rospy.Subscriber('/vrpn_client_node/Rigid3/pose', PoseStamped, self.poseCallback, 2),
            rospy.Subscriber('/vrpn_client_node/Rigid4/pose', PoseStamped, self.poseCallback, 3),
            rospy.Subscriber('/vrpn_client_node/Rigid5/pose', PoseStamped, self.poseCallback, 4),
            rospy.Subscriber('/vrpn_client_node/Rigid6/pose', PoseStamped, self.poseCallback, 5)
        ]

#调用程序
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


#引用程序数据的参考---位置信息存储在一个二维数组里
        self.pose[arg] = [-pose.pose.position.x, -pose.pose.position.y, yaw]

        if abs(self.pose[arg][2] - self.last_pose[arg][2]) < pi / 2:
            self.spend_yaw[arg] += (self.pose[arg][2] - self.last_pose[arg][2])
        elif self.pose[arg][2] < self.last_pose[arg][2]:
            self.spend_yaw[arg] += (self.pose[arg][2] + 2 * pi - self.last_pose[arg][2])
        elif self.pose[arg][2] > self.last_pose[arg][2]:
            self.spend_yaw[arg] -= (-self.pose[arg][2] + 2 * pi + self.last_pose[arg][2])
        # if arg == 3:
        # print(arg, " ", self.pose[arg])