import roslib
roslib.load_manifest('flyvr')

import rospy
import geometry_msgs.msg
import std_msgs.msg
import flyvr.display_client as display_client

from math import sin, cos
import numpy as np

rospy.init_node('moveobjects')

pub = rospy.Publisher('osg_submodel_pose', geometry_msgs.msg.PoseStamped)
pub_color = rospy.Publisher('osg_background_color', std_msgs.msg.ColorRGBA)
pub_filename = rospy.Publisher('osg_filename', std_msgs.msg.String)

rospy.sleep(1)
pub_color.publish(0,0,0,1) #black
#pub_color.publish(1,1,1,1) #white
pub_filename.publish('spheres11.osg')

def get_msg(frame):
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = frame
    p.pose.position.z = 0.27	
    #w is interpreted as the scale of the object in the OSG file. 0 scale
    #hides it, for example
    p.pose.orientation.w = 1
    return p

rospy.sleep(1)

display_client.DisplayServerProxy.set_stimulus_mode('StimulusOSG')

radius = 0.2
degrees_per_sec = 30
rad_per_sec = np.pi/180 * degrees_per_sec
experiment_period = 500
direction = +1 #+1 means to the left as seen by the fish

i = 0
while not rospy.is_shutdown():
    i += 1
    p = get_msg('s1')
    if i%experiment_period < 3.1*100:
        p.pose.position.x = +radius*sin(i%experiment_period/100. * rad_per_sec * direction)
        p.pose.position.y = -radius*cos(i%experiment_period/100. * rad_per_sec * direction)
        #	p.pose.position.x = 0 * radius
        #p.pose.position.y = -1 * radius
    else:
        p.pose.orientation.w = 0.
    pub.publish(p)

    p = get_msg('s2')
    p.pose.orientation.w = 0.
    p.pose.position.x = sin(i*0.01)
    pub.publish(p)

    rospy.sleep(1/100.0)

