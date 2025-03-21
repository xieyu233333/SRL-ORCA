#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
rospy.init_node("stop")
cmd_vel_pub1 = rospy.Publisher('/burger/cmd_vel', Twist, queue_size=1)
cmd_vel_pub2 = rospy.Publisher('/burger2/cmd_vel', Twist, queue_size=1)
cmd_vel_pub3 = rospy.Publisher('/burger3/cmd_vel', Twist, queue_size=1)
cmd_vel_pub4 = rospy.Publisher('/burger4/cmd_vel', Twist, queue_size=1)
cmd_vel_pub5 = rospy.Publisher('/burger5/cmd_vel', Twist, queue_size=1)
twist_cmd = Twist()
twist_cmd.linear.x = 0
twist_cmd.linear.y = 0
twist_cmd.angular.z = 0
# 发布控制命令
cmd_vel_pub1.publish(twist_cmd)
cmd_vel_pub2.publish(twist_cmd)
cmd_vel_pub3.publish(twist_cmd)
cmd_vel_pub4.publish(twist_cmd)
cmd_vel_pub5.publish(twist_cmd)