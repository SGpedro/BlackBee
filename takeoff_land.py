'''
Code used by the Black Bee Drones team.
(This code was not developed by us and it can be easily found on google.)
'''

#!/usr/bin/env python
# license removed for brevity

#import do ROS
import rospy
import sys
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
global cmd_vel_msg
cmd_vel_msg = Twist()

pub_cmd_vel = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size = 1)
takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size = 1)
land_pub = rospy.Publisher("/bebop/land", Empty, queue_size = 1)
rospy.init_node("drive_takeoff_land", anonymous=True)

def main():
    while 1:
        callback()

def callback():
    print('_______________________________________________________\n')
    a = input('Takeoff: (1) ||| Land (2) ||| Subir (3) ||| DESCER (4)\n_______________________________________________________\n')
    if(a == 1 ):
        print(a)
        takeoff_pub.publish(Empty())
    elif(a == 2):
        print(a)
        land_pub.publish(Empty())
    elif(a == 3):
        print(a)
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.6
        pub_cmd_vel.publish(cmd_vel_msg)
    elif(a == 4):
        print(a)
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = -0.6
        pub_cmd_vel.publish(cmd_vel_msg)
    elif(a == None):
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        pub_cmd_vel.publish(cmd_vel_msg)

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
