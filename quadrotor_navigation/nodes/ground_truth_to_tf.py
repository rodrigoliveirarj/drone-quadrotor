#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3Stamped

def callback(data):
    br = tf.TransformBroadcaster()
    time = data.header.stamp

    # Publica map -> odom (identidade)
    br.sendTransform(
        (0.0, 0.0, 0.0),
        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
        time,
        "odom",
        "map"
    )

    # Posição e orientação
    pos = data.pose.pose.position
    ori = data.pose.pose.orientation

    # Publica odom -> base_footprint
    br.sendTransform(
        (pos.x, pos.y, 0.0),
        tf.transformations.quaternion_from_euler(0.0, 0.0, tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]),
        time,
        "base_footprint",
        "odom"
    )

    # Publica base_footprint -> base_stabilized
    br.sendTransform(
        (0.0, 0.0, pos.z),
        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
        time,
        "base_stabilized",
        "base_footprint"
    )

    # Publica base_stabilized -> base_link
    br.sendTransform(
        (0.0, 0.0, 0.0),
        (ori.x, ori.y, ori.z, ori.w),
        time,
        "base_link",
        "base_stabilized"
    )

    # Publicações auxiliares (opcional)
    pose_msg = PoseStamped()
    pose_msg.header = data.header
    pose_msg.pose = data.pose.pose
    pose_pub.publish(pose_msg)

    euler = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    euler_msg = Vector3Stamped()
    euler_msg.header = data.header
    euler_msg.vector.x = euler[0]
    euler_msg.vector.y = euler[1]
    euler_msg.vector.z = euler[2]
    euler_pub.publish(euler_msg)

if __name__ == '__main__':
    rospy.init_node('ground_truth_to_tf')
    rospy.Subscriber("/ground_truth/state", Odometry, callback)
    pose_pub = rospy.Publisher("/ground_truth_to_tf/pose", PoseStamped, queue_size=1)
    euler_pub = rospy.Publisher("/ground_truth_to_tf/euler", Vector3Stamped, queue_size=1)
    rospy.spin()

