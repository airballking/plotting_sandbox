#!/usr/bin/env python
# TODO: add license

import rospy
import tf2_ros
import geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

class PlottingWorld(object):
    def __init__(self):
        self.tf_broadcoaster = tf2_ros.StaticTransformBroadcaster()
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=5)
        self.marker_ns = "plotting_world"

    def publish(self):
        self.clear_world()
        self.publish_robot_loc()
        self.publish_scene()

    def clear_world(self):
        self.marker_pub.publish(MarkerArray([self.clear_all_marker()]))

    def publish_robot_loc(self):
        # TODO: read from param server
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_footprint"
        t.transform.rotation.w = 1.0

        self.tf_broadcoaster.sendTransform(t)

    def publish_scene(self):
        # TODO: read from param server
        m1 = self.add_mesh_marker(
            0, "package://iai_kitchen/meshes/misc/big_table_1.dae",
            PoseStamped(Header(0, rospy.Time.now(), "map"),
                        Pose(Point(0.6, 0, 0), Quaternion(0,0,0.7068251811053659, 0.7073882691671998))))

        self.marker_pub.publish(MarkerArray([m1]))

    def sane_empty_marker(self):
        """

        :return: Empty marker filled with sane defaults.
        :rtype: Marker
        """
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.ns = self.marker_ns
        return m

    def clear_all_marker(self):
        m = self.sane_empty_marker()
        m.action = Marker.DELETEALL
        return m

    def add_mesh_marker(self, id, mesh_resource, pose_stamped):
        """

        :param id:
        :param mesh_resource:
        :param pose_stamped:
        :type pose_stamped: PoseStamped
        :return: Correctly filled marker to add the mesh at the given pose.
        :rtype: Marker
        """
        m = self.sane_empty_marker()
        m.header = pose_stamped.header
        m.action = Marker.ADD
        m.type = Marker.MESH_RESOURCE
        m.id = id
        m.pose = pose_stamped.pose
        m.scale = Vector3(1, 1, 1)
        m.mesh_resource = mesh_resource
        m.color = ColorRGBA(0, 0, 0, 1)
        return m


if __name__ == '__main__':
    try:
        rospy.init_node('plotting_world')
        my_world = PlottingWorld()
        rospy.sleep(rospy.Duration(0.5))
        my_world.publish()
        rospy.sleep(rospy.Duration(0.5))
    except rospy.ROSInterruptException:
        pass