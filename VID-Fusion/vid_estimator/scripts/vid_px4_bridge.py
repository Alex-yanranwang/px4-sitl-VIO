#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import CompanionProcessStatus
#import MAV_STATE
import math
from pyquaternion import Quaternion
import tf
import sys

from tf import TransformListener
from geometry_msgs.msg import Vector3Stamped

# roll, pitch and yaw angles
#quaternion = tf.transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2)
quaternion_x = tf.transformations.quaternion_from_euler(math.pi*85.17/180, 0, 0)#xyzw
q_x = Quaternion([quaternion_x[3],quaternion_x[0],quaternion_x[1],quaternion_x[2]])

quaternion_y = tf.transformations.quaternion_from_euler(0, math.pi*2.08376/180, 0)#xyzw
q_y = Quaternion([quaternion_y[3],quaternion_y[0],quaternion_y[1],quaternion_y[2]])

quaternion_z = tf.transformations.quaternion_from_euler(0, 0, math.pi*90/180)#xyzw
q_z = Quaternion([quaternion_z[3],quaternion_z[0],quaternion_z[1],quaternion_z[2]])

def vid_callback(data):
    # #just a test
    # quaternion = tf.transformations.quaternion_from_euler(math.pi*1.48575/180, 0, math.pi/2)
    # q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])

    local_pose = Odometry() 
    # local_pose = data

    # local_pose.header.frame_id = 'odom_ned'
    # local_pose.child_frame_id = 'base_link_frd'
    
    
    local_pose.header.seq = data.header.seq
    local_pose.header.stamp = data.header.stamp
    local_pose.header.frame_id = 'odom'#odom_ned
    local_pose.child_frame_id = 'base_link'#base_link_frd
    #print("data.header0", data.header, "child_frame_id: ", data.child_frame_id)
    local_pose.pose.pose.position.x = data.pose.pose.position.x
    local_pose.pose.pose.position.y = data.pose.pose.position.y
    local_pose.pose.pose.position.z = data.pose.pose.position.z

    q_= Quaternion([data.pose.pose.orientation.w,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z])
    q_ = q_*q_x
    # euler = tf.transformations.euler_from_quaternion([q_[1],q_[2],q_[3],q_[0]])
    # print("euler_x: ",euler)
    q_ = q_*q_y
    # euler = tf.transformations.euler_from_quaternion([q_[1],q_[2],q_[3],q_[0]])
    # print("euler_y: ",euler)
    q_ = q_*q_z
    # euler = tf.transformations.euler_from_quaternion([q_[1],q_[2],q_[3],q_[0]])
    # print("euler_z: ",euler)
    # print(" ")
    local_pose.pose.pose.orientation.w = q_[0]
    local_pose.pose.pose.orientation.x = q_[1]
    local_pose.pose.pose.orientation.y = q_[2]
    local_pose.pose.pose.orientation.z = q_[3]

    # print("data.header1", data.header)
    # print("local_pose: ", local_pose)

    try:
        tf_listener.waitForTransform("camera", "world", rospy.Time(), rospy.Duration(4.0))
        velocity_in_baselink = tf_listener.transformVector3("camera", Vector3Stamped(data.header, data.twist.twist.linear))
        #print("velocity_in_baselink: ", velocity_in_baselink)
    except tf.Exception as e:
            #rospy.loginfo('WOW tf error (world -> camera)  : {}'.format(e))
            return
    #don't know why the x-y is opposite
    local_pose.twist.twist.linear.x = velocity_in_baselink.vector.y
    local_pose.twist.twist.linear.y = -velocity_in_baselink.vector.x
    local_pose.twist.twist.linear.z = velocity_in_baselink.vector.z
    position_pub.publish(local_pose)
    

rospy.init_node('vid_px4_bridge')
tf_listener = TransformListener()
# tf_listener.waitForTransform("camera", "world", rospy.Time(), rospy.Duration(500.0))
rospy.Subscriber("/vid_estimator/odometry", Odometry, vid_callback, queue_size=1)
#position_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=1)
position_pub = rospy.Publisher("/mavros/odometry/out", Odometry, queue_size=1)
# mavros_system_status_pub_ = rospy.Publisher("/mavros/companion_process/status", CompanionProcessStatus, queue_size=1)
rate = rospy.Rate(10) 


while not rospy.is_shutdown():
    #print("111111111111")
    #local_pose.header.stamp = rospy.Time.now()
    # tf_listener.waitForTransform("world", "camera", rospy.Time(), rospy.Duration(500.0))
    rate.sleep()
  