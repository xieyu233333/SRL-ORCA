import rospy,math
from geometry_msgs.msg import Twist
rospy.init_node("publisher")
cmd_vel_pub1 = rospy.Publisher('/burger/cmd_vel', Twist, queue_size=1)

twist_cmd = Twist()
twist_cmd.linear.x = 0.1
twist_cmd.linear.y = 0
twist_cmd.angular.z = 0.01
# 发布控制命令
cmd_vel_pub1.publish(twist_cmd)
