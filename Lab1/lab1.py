# Zach Schwark -- 2434346
# Lisa Godwin -- 2437980


#!/usr/bin/python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


def takeoff_Move_Land():
    rospy.init_node('control_drone', anonymous=True)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff',Empty, queue_size=1)
    forward_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=5)
    land_pub = rospy.Publisher('/ardrone/land',Empty, queue_size=1)
    
   

    rate = rospy.Rate(10) # 10hz
    
    rate.sleep()
    

    move_msg = Twist()
    
    for i in range(30):
        rospy.loginfo("taking off")
        takeoff_pub.publish(Empty()) 
        rate.sleep()
        
    before_moving_time = rospy.get_time()
    while ((rospy.get_time() - before_moving_time) < 3):
        move_msg.linear.x = 1
        move_msg.linear.y = 0
        move_msg.linear.z = 0
        rospy.loginfo("moving")
        forward_pub.publish(move_msg)
        #rate.sleep()

    
    for i in range(3):  
        rate.sleep()  
        rospy.loginfo("landing")
        land_pub.publish(Empty()) 



if __name__ == '__main__':
    try:
        takeoff_Move_Land()
    except rospy.ROSInterruptException:
        pass
    
    