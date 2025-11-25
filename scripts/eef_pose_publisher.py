#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

"""
ROS1 node: eef_pose_publisher.py

Publishes the pose of a link at 50Hz as a geometry_msgs/PoseStamped by looking up the transform via tf each cycle.

Params (private or ROS param server):
    ~reference_frame   (string) frame in which to express the pose (default: "base_link")
    ~target_frame   (string) the link/frame whose pose will be published (default: "ee_link")
    ~rate           (float) publish rate in Hz (default: 50.0)
"""


class EEFPosePublisher(object):
    def __init__(self):
        rospy.init_node('eef_pose_publisher')

        self.target_frame = rospy.get_param('~target_frame', 'arm_base_link')
        self.reference_frame = rospy.get_param('~reference_frame', 'arm_tool0')
        rate_hz = rospy.get_param('~rate', 50.0)

        self.pub = rospy.Publisher("frame_pose", PoseStamped, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(rate_hz)

        rospy.loginfo("eef_pose_publisher: publishing pose of '%s' in frame '%s' on '%s' @ %.1f Hz",
                                    self.target_frame, self.reference_frame, "frame_pose", rate_hz)

        self.run()

    def run(self):
        self.tf_listener.waitForTransform(self.reference_frame, self.target_frame, rospy.Time(), rospy.Duration(5.0))

        while not rospy.is_shutdown():
            try:
                msg = self.tf_listener._buffer.lookup_transform(self.reference_frame, self.target_frame, rospy.Time(0))
                pose_msg = PoseStamped(header= msg.header)
                pose_msg.pose.position= msg.transform.translation
                pose_msg.pose.orientation= msg.transform.rotation
                self.pub.publish(pose_msg)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn_throttle(1.0,
                        "eef_pose_publisher: tf lookup failed for %s -> %s: %s",
                        self.reference_frame, self.target_frame, str(e))

            self.rate.sleep()


if __name__ == '__main__':
        try:
                EEFPosePublisher()
        except rospy.ROSInterruptException:
                pass
