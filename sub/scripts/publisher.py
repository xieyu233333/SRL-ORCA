import rospy,math
import numpy as np
from geometry_msgs.msg import Twist
# def publisher():
#     # rospy.init_node("publisher")
#     cmd_vel_pub1 = rospy.Publisher('/burger/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub2 = rospy.Publisher('/burger2/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub3 = rospy.Publisher('/burger3/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub4 = rospy.Publisher('/burger4/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub5 = rospy.Publisher('/burger5/cmd_vel', Twist, queue_size=1)
#
#     cmd_vel_pub=[cmd_vel_pub1,cmd_vel_pub2,cmd_vel_pub3,cmd_vel_pub4,cmd_vel_pub5]
#     return cmd_vel_pub

def publisher():
    # rospy.init_node("publisher")
    sac_cmd_vel_pub1 = rospy.Publisher('/tb3_1/sac_cmd_vel', Twist, queue_size=1)
    sac_cmd_vel_pub2 = rospy.Publisher('/tb3_2/sac_cmd_vel', Twist, queue_size=1)
    sac_cmd_vel_pub3 = rospy.Publisher('/tb3_3/sac_cmd_vel', Twist, queue_size=1)
    sac_cmd_vel_pub4 = rospy.Publisher('/tb3_4/sac_cmd_vel', Twist, queue_size=1)
    sac_cmd_vel_pub5 = rospy.Publisher('/tb3_5/sac_cmd_vel', Twist, queue_size=1)
    sac_cmd_vel_pub=[sac_cmd_vel_pub1,sac_cmd_vel_pub2,sac_cmd_vel_pub3,sac_cmd_vel_pub4,sac_cmd_vel_pub5]
    return sac_cmd_vel_pub

# def publisher():
#     # rospy.init_node("publisher")
#     cmd_vel_pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub4 = rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub5 = rospy.Publisher('/tb3_5/cmd_vel', Twist, queue_size=1)
#     cmd_vel_pub=[cmd_vel_pub1,cmd_vel_pub2,cmd_vel_pub3,cmd_vel_pub4,cmd_vel_pub5]
#     return cmd_vel_pub

# ---------------------------------------------------------------------------#
# 改pub
# ---------------------------------------------------------------------------#
def pub(ui,yaw_ori,tbname,sac_cmd_vel_pub):
    twist_cmd = Twist()
    norm_ui = np.linalg.norm(ui)
    w = np.arctan2(ui[1],ui[0]) - yaw_ori
    if w <= -math.pi:
        w += 2 * math.pi
    elif w > math.pi:
        w -= 2 * math.pi
    # print('w:',w)
    # print('norm_ui:',norm_ui)
    twist_cmd.linear.x = (norm_ui<0.10)*((abs(w)<1.5)*norm_ui*2+(abs(w)>=1.5)*norm_ui*0.2) + (norm_ui>=0.10)*(norm_ui<0.2)*((abs(w)<1.5)*norm_ui*2+(abs(w)>=1.5)*norm_ui*0.5) + (norm_ui>0.2)*((abs(w)<1.5)*norm_ui*2+(abs(w)>=1.5)*norm_ui*0.2)
    twist_cmd.angular.z = (abs(w)<1.5) * w * 0.5 + (abs(w)>=1.5) * w * 3*twist_cmd.linear.x
    # twist_cmd.linear.x = (abs(w) < 1.5) * norm_ui * 1.5 + (abs(w) >= 1.5) * norm_ui * 0.3
    # twist_cmd.angular.z = (twist_cmd.linear.x > 0.15) * w * 0.1 + (twist_cmd.linear.x <= 0.15) * w * 2.
    # print('linear.x:',twist_cmd.linear.x)
    # print('angular.z:',twist_cmd.angular.z)
    # 发布控制命令
    sac_cmd_vel_pub[tbname].publish(twist_cmd)
    # rospy.sleep(0.02)
    # twist_cmd.angular.z = twist_cmd.angular.z * 0.5
    # twist_cmd.linear.x = twist_cmd.linear.x * 0.5
    # cmd_vel_pub[tbname].publish(twist_cmd)