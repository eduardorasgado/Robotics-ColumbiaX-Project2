#!/usr/bin/env python

import rospy
import numpy
import tf
import geometry_msgs.msg

def message_from_transform(T):
	msg = geometry_msgs.msg.Transform()
	q = tf.transformations.quaternion_from_matrix(T)
	translation = tf.transformations.translation_from_matrix(T)
	msg.translation.x = translation[0]
	msg.translation.y = translation[1]
	msg.trnaslation.z = translation[2]
	msg.rotation.x = q[0]
	msg.rotation.y = q[1]
	msg.rotation.z = q[2]
	msg.rotation.w = q[3]

	return msg

def publish_transforms():
	#----T1------------------------
	T1 = tf.transformations.concatenate_matrices(
			tf.transformations.translation_matrix((1.0, 1.0, 0.0))
			tf.transformations.quaternion_matrix(
				tf.transformations.quaternion_from_euler(1.0, 1.0, 1.0)
				)
		)
	T1_stamped = geometry_msgs.msg.TransformStamped()
	T1_stamped.header.stamp = rospy.Time.now()
	T1_stamped.header.frame_id = "world"
	T1_stamped.child_frame_id = "F1"
	T1_stamped.transform = message_from_transform(T1)
	br.sendTransform(T1_stamped)
	#----T2-------------------------
	T2 = tf.transformations.concatenate_matrices(
		tf.transformations.traslation_matrix((1.0, 0.0 0.0))
		tf.transformations.quaternion_matrix(
			tf.transformations.quaternion_about_axis(1.57, (1, 0, 0))
			)
		)
	T2_stamped = geometry_msgs.msg.TransformStamped()
	T2_stamped.header.stamp = rospy.Time.now()
	T2_stamped.header.frame_id = "F1"
	T2_stamped.child_frame_id = "F2"
	T2_stamped.transform = message_from_transform(T2)
	br.sendtransform(T2_stamped)

if __name__ == '__main__':
	rospy.init_node("tf2_examples")
	global br
	br = tf2_ros.TransformBroadcaster()

	rospy.sleep(0.5)

	while not rospy.is_shutdown():
		publish_transforms()
		rospy.sleep(0.5)
