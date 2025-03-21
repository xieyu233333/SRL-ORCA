#!/usr/bin/env python3
import rospy,math,tf
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
import numpy as np
from nav_msgs.msg import Odometry
# def euler_form_quaternion(quaternion):
#     x, y, z, w = [getattr(quaternion, attr) for attr in ['x', 'y', 'z', 'w']]
#     # print("quaternion: ", quaternion)
#     # 计算滚转角（绕x轴旋转）roll
#     sinr_cosp = 2.0 * (w * x + y * z)
#     cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
#     roll = math.atan2(sinr_cosp, cosr_cosp)
#     # 计算俯仰角（绕y轴旋转）pitch
#     sinp = 2.0 * (w * y - z * x)
#     if abs(sinp) >= 1:
#         pitch = math.copysign(math.pi / 2, sinp)  # 如果超出范围，使用90度
#     else:
#         pitch = math.asin(sinp)
#     # 计算偏航角（绕z轴旋转）yaw
#     siny_cosp = 2.0 * (w * z + x * y)
#     cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
#     yaw = math.atan2(siny_cosp, cosy_cosp)
#     if yaw<0:
#         yaw+=2 * math.pi
#     # print("euler_form_quaternion")
#     return roll, pitch, yaw
def euler_form_quaternion(orientation):
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler
# def yaw_callback(data, args):
#     tbnum = args[0]
#     yaw = args[1]
#     yaw[tbnum] = euler_form_quaternion(data.pose.pose.orientation)[2]

def pos_callback(data, args):
    tbnum = args[0]
    x_in = args[1]
    y_in = args[2]
    yaw = args[3]
    x_in[tbnum] = round(data.pose.position.x-1.5375,3)
    y_in[tbnum] = round(data.pose.position.y+4.1426,3)
    yaw[tbnum] = euler_form_quaternion(data.pose.orientation)[2]
    # if len(x_in)==5 and len(y_in)==5:
    #     print(f'x_in:{x_in}\n')
    #     print(f'y_in:{y_in}\n')

def listener(x_in, y_in,yaw):
    # 初始化ROS节点
    rospy.init_node('odom_subscriber', anonymous=True)
    # 创建一个订阅者，订阅里程计信息
    rospy.Subscriber('/vrpn_client_node/Rigid2/pose', PoseStamped, pos_callback, [1, x_in, y_in , yaw])
    rospy.Subscriber('/vrpn_client_node/Rigid3/pose', PoseStamped, pos_callback, [2, x_in, y_in , yaw])
    rospy.Subscriber('/vrpn_client_node/Rigid4/pose', PoseStamped, pos_callback, [3, x_in, y_in , yaw])
    rospy.Subscriber('/vrpn_client_node/Rigid5/pose', PoseStamped, pos_callback, [4, x_in, y_in , yaw])
    rospy.Subscriber('/vrpn_client_node/Rigid1/pose', PoseStamped, pos_callback, [0, x_in, y_in , yaw])





