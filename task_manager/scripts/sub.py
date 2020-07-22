#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


pose = PoseStamped()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.pose.position.x)
    global pose
    pose = data
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
   
    rospy.Subscriber("pose_luka", PoseStamped, callback)
    pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)

    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():
        global pose
        rospy.loginfo(rospy.get_caller_id() + "I ll pub  %f", pose.pose.position.x)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    listener()
