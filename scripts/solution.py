#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

##FOR NEW SPACE POSITION
def message_from_transform(T):
    message = geometry_msgs.msg.Transform()
    quad = tf.transformations.quaternion_from_matrix(T)
    translationMove = tf.transformations.translation_from_matrix(T)
    message.translation.x = translationMove[0]
    message.translation.y = translationMove[1]
    message.translation.z = translationMove[2]
    message.rotation.x = quad[0]
    message.rotation.y = quad[1]
    message.rotation.z = quad[2]
    message.rotation.w = quad[3]

    return message

def one_magnitude_vector(vector):
    #returnning unit vector of a vector
    unitVector = vector / numpy.linealf.norm(vector)
    return unitVector

def angle_calculation_btwn(vector1, vector2):
    #this will return the angle in radians beteen two vectors
    vector1_unit = one_magnitude_vector(vector1)
    vector2_unit = one_magnitude_vector(vector2)

    #getting the dot product between vector1 and vector2
    dotProd = numpy.dot(vector1_unit, vector2_unit)

    """
    clip(a, a_min, a_max, out=None)

    Clip (limit) the values in an array.

    Given an interval, values outside the interval are clipped to the interval
    edges. For example, if an interval of [0, 1] is specified, values smaller
    than 0 become 0, and values larger than 1 become 1.
    """
    clippedVector = numpy.clip(dotProd, -1.0, 1.0)

    angle = numpy.arccos(clippedVector)
    return angle


def publish_transforms():
    # ----------T1-------------------------------------
    T1 = tf.transformations.concatenate_matrices(
        tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)
            ),
        tf.transformations.translation_matrix((0.0, 1.0, 1.0))
        )
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    object_transform.transform = message_from_transform(T1)
    br.sendTransform(object_transform)

    #-------------T2------------------------------------
    T2 = tf.transformations.concatenate_matrices(
        tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(1.5, (0, 0, 1))
            ),
        tf.transformations.translation_matrix((0.0, -1.0, 0.0))
        )
    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    robot_transform.transform = message_from_transform(T2)
    br.sendTransform(robot_transform)
 
    #-------------T3------------------------------------
    #Starter point
    T3 = tf.transformations.concatenate_matrices(
        tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            ),
        tf.transformations.translation_matrix((0.0, 0.1, 0.1))
        )
    #calculations
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    camera_transform.transform = message_from_transform(T3)
    br.sendTransform(camera_transform)

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
