#!/usr/bin/env python
# TODO: add license

import rospy
import tf2_ros
import geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState

class PlottingWorld(object):
    def __init__(self):
        self.tf_broadcoaster = tf2_ros.StaticTransformBroadcaster()
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=5)
        self.js_pub = rospy.Publisher("/new_joint_states", JointState, queue_size=5)
        self.marker_ns = "plotting_world"

    def publish(self):
        self.clear_world()
        self.publish_robot_loc()
        self.publish_scene()
        self.publish_joint_state()

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
                        Pose(Point(0.6, 0, 0), Quaternion(0,0,0.7068251811053659, 0.7073882691671998))),
                        ColorRGBA(207 / 255.0, 167 / 255.0, 110 / 255.0, 1.0))
        m2 = self.add_mesh_marker(
            1, "package://plotting_sandbox/meshes/electrical-devices/pancake_maker2.stl",
            PoseStamped(Header(1, rospy.Time.now(), "map"),
                        Pose(Point(0.45, 0, 0.72), Quaternion(0, 0, 1, 0))),
                        ColorRGBA(0.3,0.3,0.3,1.0))

        self.marker_pub.publish(MarkerArray([m1, m2]))

    def publish_joint_state(self):
        # TODO: read from param server
        js = {'head_pan_joint': 0.25,
              'head_tilt_joint': 0.97,
              'torso_lift_joint': 0.3,
              'r_shoulder_pan_joint': -1.31,
              'r_shoulder_lift_joint': 1.28,
              'r_upper_arm_roll_joint': -1.45,
              'r_forearm_roll_joint': -1.13,
              'r_elbow_flex_joint': -0.24,
              'r_wrist_flex_joint': -0.19}

        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        for name in js:
            js_msg.name.append(name)
            js_msg.position.append(js[name])
        self.js_pub.publish(js_msg)

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

    def add_mesh_marker(self, id, mesh_resource, pose_stamped, color=ColorRGBA(0,0,0,1), scale=Vector3(1,1,1)):
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
        m.scale = scale
        m.mesh_resource = mesh_resource
        m.color = color
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