'''
Code used by the Black Bee Drones team.
(This code was not developed by us and it can be easily found on google.)
'''

#!/usr/bin/env python
# license removed for brevity

#import do ROS
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#pub global
global pub_cmd_vel
global cmd_vel_msg
cmd_vel_msg = Twist()
pub_cmd_vel = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size = 1)

def callback(msg):
    if(msg.data == "esq"):
        #rospy.loginfo("Indo para a direita")
        rospy.loginfo("Rotacionando para a esquerda")
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y= 0.0
        cmd_vel_msg.angular.z = 0.05
    elif(msg.data == "dir"):
        #rospy.loginfo("Indo para a esquerda")
        rospy.loginfo("Rotacionando para a direita")
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y= 0.0
        cmd_vel_msg.angular.z = -0.05
    pub_cmd_vel.publish(cmd_vel_msg)

def main():
    sub_face = rospy.Subscriber("movimento_face", String, callback)
    #takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size = 1)
    rospy.init_node("drive_face", anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
