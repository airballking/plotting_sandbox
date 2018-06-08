#!/usr/bin/env python
# TODO: add license

import rospy
import tf2_ros
import geometry_msgs

class PlottingWorld(object):
    def __init__(self):
        self.tf_broadcoaster = tf2_ros.StaticTransformBroadcaster()

    def publish(self):
        self.publish_robot_loc()

    def publish_robot_loc(self):
        t = geometry_msgs.msg.TransformStamped()

        # TODO: read from param server
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_footprint"
        t.transform.rotation.w = 1.0

        self.tf_broadcoaster.sendTransform(t)

if __name__ == '__main__':
    try:
        rospy.init_node('plotting_world')
        my_world = PlottingWorld()
        my_world.publish()
        rospy.sleep(rospy.Duration(1.0))
    except rospy.ROSInterruptException:
        pass