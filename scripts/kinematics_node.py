#!/usr/bin/env python

import rospy
import numpy
import tf
import geometry_msgs.msg
#from project2.msg import bla

def message_from_transform(T):
	msg = geometry_msgs.msg.Transform()
	q = tf.transformations.quaternion from matrix(T)
