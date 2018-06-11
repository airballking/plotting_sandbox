#!/usr/bin/env python

#  Copyright 2018 Georg Bartels (georg.bartels@cs.uni-bremen.de).
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Transform, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState


def from_point(point_msg):
    """
    
    :param point_msg:
    :type point_msg: Point 
    :return: 
    :rtype Vector3
    """
    return Vector3(point_msg.x, point_msg.y, point_msg.z)


def from_pose(pose_msg):
    """

    :param pose_msg:
    :type pose_msg: Pose
    :return:
    :rtype: Transform
    """
    return Transform(from_point(pose_msg.position), pose_msg.orientation)


def read_js_from_param_server(namespace):
    """
    Reads the content of a robot joint state from the parameter server.
    :param namespace: ROS parameter namespace within which to search.
    :return: Instanstiated joint state
    :rtype: JointState
    """
    js = rospy.get_param(namespace)
    js_msg = JointState()
    js_msg.header.stamp = rospy.Time.now()
    for name in js:
        js_msg.name.append(name)
        js_msg.position.append(js[name])
    return js_msg


def read_transform_from_param_server(namespace):
    """
    Reads the content of a stamped transform from the parameter server.
    :param namespace: ROS parameter namespace within which to search.
    :type namespace: String
    :return: Instantiated stamped transform
    :rtype: TransformStamped
    """
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = rospy.get_param('{}/frame_id'.format(namespace))
    t.child_frame_id = rospy.get_param('{}/child_frame_id'.format(namespace))
    t.transform.rotation = Quaternion(*rospy.get_param('{}/rotation'.format(namespace)))
    t.transform.translation = Point(*rospy.get_param('{}/translation'.format(namespace)))

    return t


class PlottingWorld(object):
    def __init__(self):
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=5)
        self.js_pub = rospy.Publisher("/new_joint_states", JointState, queue_size=5)
        self.marker_ns = "plotting_world"

        self.robot_state = dict()
        self.robot_state['joint_state'] = read_js_from_param_server('~joints')
        self.robot_state['localization'] = read_transform_from_param_server('~localization')

        self.objects = self.read_objects()

    def read_objects(self):
        # TODO: read objects from param server
        m1 = self.add_mesh_marker(
            0, "package://iai_kitchen/meshes/misc/big_table_1.dae",
            PoseStamped(Header(0, rospy.Time.now(), "map"),
                        Pose(Point(0.6, 0, 0), Quaternion(0,0,0.7068251811053659, 0.7073882691671998))),
                        ColorRGBA(207 / 255.0, 167 / 255.0, 110 / 255.0, 1.0))
        m2 = self.add_mesh_marker(
            1, "package://plotting_sandbox/meshes/electrical-devices/pancake_maker2.stl",
            PoseStamped(Header(0, rospy.Time.now(), "map"),
                        Pose(Point(0.45, 0, 0.72), Quaternion(0, 0, 1, 0))),
                        ColorRGBA(0.3,0.3,0.3,1.0))

        m3 = self.add_mesh_marker(
            2, "package://plotting_sandbox/meshes/hand-tools/edeka_spatula2.stl",
            PoseStamped(Header(0, rospy.Time.now(), "l_gripper_tool_frame"),
                        Pose(Point(-0.01,0,0), Quaternion(0.706825181105366, 0.0, 0.0, 0.7073882691671997))),
                        ColorRGBA(0.3, 0.3, 0.3, 1.0), Vector3(1, 1, 1))

        m4 = self.add_cylinder_marker(
            3, PoseStamped(Header(0, rospy.Time.now(), "map"),
                           Pose(Point(0.47, 0, 0.8), Quaternion(0, 0, 0, 1))),
            0.11, 0.01, ColorRGBA(241/255.0, 185/255.0, 94/255.0, 1.0))
        return (m1, m2, m3, m4)

    def publish(self):
        """
        Publishes all internal state. Afterwards, RVIZ should display the desired world state.
        :return: Nothing.
        """
        self.clear_world()
        self.publish_robot_state()
        self.publish_scene()

    def clear_world(self):
        """
        Tells RVIZ to remove all markers from the canvas.
        :return: Nothing.
        """
        self.marker_pub.publish(MarkerArray([self.clear_all_marker()]))

    def publish_robot_state(self):
        """
        Publishes the robots joint state and base localization to the joint_state_publisher and TF, respectively.
        :return: Nothing.
        """
        self.tf_broadcaster.sendTransform(self.robot_state['localization'])
        self.js_pub.publish(self.robot_state['joint_state'])

    def publish_scene(self):
        """
        Publishes object poses to TF and object markers to RVIZ.
        :return: Nothing.
        """
        # publish object poses to TF
        transforms = []
        for obj in self.objects:
            transforms.append(TransformStamped(obj.header, 'object{}_frame'.format(obj.id), from_pose(obj.pose)))
        self.tf_broadcaster.sendTransform(transforms)

        # publish markers to RVIZ
        self.marker_pub.publish(MarkerArray(self.objects))

    def sane_empty_marker(self, id):
        """

        :return: Empty marker filled with sane defaults.
        :rtype: Marker
        """
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.ns = self.marker_ns
        m.id = id
        m.frame_locked = True
        return m

    def clear_all_marker(self, id=0):
        m = self.sane_empty_marker(id)
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
        m = self.sane_empty_marker(id)
        m.header = pose_stamped.header
        m.action = Marker.ADD
        m.type = Marker.MESH_RESOURCE
        m.pose = pose_stamped.pose
        m.scale = scale
        m.mesh_resource = mesh_resource
        m.color = color
        return m

    def add_cylinder_marker(self, id, pose_stamped, diameter, height, color=ColorRGBA(0,0,0,1)):
        """

        :param id:
        :param pose_stamped:
        :rtype: PoseStamped
        :param diameter:
        :param height:
        :param color:
        :return:
        :rtype: Marker
        """
        m = self.sane_empty_marker(id)
        m.header = pose_stamped.header
        m.action = Marker.ADD
        m.type = Marker.CYLINDER
        m.pose = pose_stamped.pose
        m.scale = Vector3(diameter, diameter, height)
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