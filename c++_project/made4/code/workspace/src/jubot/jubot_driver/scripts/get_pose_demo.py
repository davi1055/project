#!/usr/bin/env python 
import rospy
from tf_conversions import transformations 
from * import transformations 
from math import pi
import tf
class Robot:
    def init (self):
        self.tf_listener = tf.TransformListener() 
        print("aaa")
        try:
            self.tf_listener.waitForTransform('/odom', '/base_footprint', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException): return
    def get_pos(self): 
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('/odom', '/base_footprint', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException): 
            return
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error") 
            return None
        euler = transformations.euler_from_quaternion(rot) 
        x = trans[0] 
        y = trans[1]
        th = euler[2] / pi * 180 
        return (x, y, th)
if  __name__  == "__main__": 
    rospy.init_node('get_pos_demo',anonymous=True) 
    robot = Robot()
    r = rospy.Rate(100) 
    r.sleep()
    while not rospy.is_shutdown(): 
        print robot.get_pos()
        r.sleep()
