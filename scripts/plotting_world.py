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


def from_vec3(msg):
    """

    :param msg:
    :type msg: Vector3
    :return:
    :rtype: Point
    """
    return Point(msg.x, msg.y, msg.z)


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


def decode_transfrom(encoded_transform):
    """
    Decodes a stamped transform given as read from the parameter server.
    :param encoded_transform: The transform as read from the parameter server.
    :type encoded_transform: Dict
    :return: The stamped transform object.
    :rtype: TransformStamped
    """
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = encoded_transform['frame_id']
    t.child_frame_id = encoded_transform['child_frame_id']
    t.transform.rotation = Quaternion(*encoded_transform['rotation'])
    t.transform.translation = Point(*encoded_transform['translation'])

    return t


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


def decode_pose(pose_dict):
    """
    Decodes a stamped pose from a dict read from the ROS parameter server.
    :param pose_dict: Dict read from the ROS parameter server.
    :type pose_dict: Dict.
    :return: Instantiated pose stamp.
    :rtype: PoseStamped
    """
    pose = PoseStamped()
    pose.header.frame_id = pose_dict['frame_id']
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = Point(*pose_dict['translation'])
    pose.pose.orientation = Quaternion(*pose_dict['rotation'])
    return pose


class PlottingWorld(object):
    def __init__(self):
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_listener = tf2_ros.BufferClient('tf2_buffer_server')
        self.tf_listener.wait_for_server(rospy.Duration(1.0))
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=5)
        self.js_pub = rospy.Publisher("/new_joint_states", JointState, queue_size=5)
        self.marker_ns = "plotting_world"

        self.robot_state = dict()
        self.robot_state['joint_state'] = read_js_from_param_server('~joints')
        self.robot_state['localization'] = read_transform_from_param_server('~localization')

        self.objects = self.read_objects()
        self.frames = self.read_frames()
        self.goals = rospy.get_param('~goals')

    def read_goals(self):
        goals = []
        goals.append({'from_frame_id': 'object2_frame',
                      'to_frame_id': 'object3_frame'})
        return goals

    def read_frames(self):
        frames = []
        for frame in rospy.get_param('~frames'):
            frames.append(decode_transfrom(frame))
        return frames

    def read_objects(self):
        """
        Read object definitions from param server.
        :return: Object definitions read from parameter server.
        :rtype: list of Marker
        """
        objects = []
        for obj in rospy.get_param('~objects'):
            if obj['type'] == 'MESH':
                objects.append(self.add_mesh_marker(len(objects), obj['mesh'], decode_pose(obj['pose']), ColorRGBA(*obj['color'])))
            elif obj['type'] == 'CYLINDER':
                objects.append(self.add_cylinder_marker(len(objects), decode_pose(obj['pose']),
                                                        Vector3(*obj['scale']), ColorRGBA(*obj['color'])))
            else:
                raise RuntimeError("Encountered object with unknown type: {}".format(obj))
        return objects

    def publish(self):
        """
        Publishes all internal state. Afterwards, RVIZ should display the desired world state.
        :return: Nothing.
        """
        self.clear_world()
        self.publish_robot_state()
        self.publish_scene()
        self.publish_goals()

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
        Publishes object poses to TF and object markers to RVIZ. Also publishes additional TF frames.
        :return: Nothing.
        """
        # publish object poses to TF
        transforms = []
        for obj in self.objects:
            transforms.append(TransformStamped(obj.header, 'object{}_frame'.format(obj.id), from_pose(obj.pose)))
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

        # publish additional TF frames
        if self.frames:
            self.tf_broadcaster.sendTransform(self.frames)

        # publish all markers to RVIZ
        self.marker_pub.publish(MarkerArray(self.objects))

    def publish_goals(self):
        goal_markers = []
        for goal in self.goals:
            goal_markers.append(self.arrow_marker(len(self.objects) + len(goal_markers), goal['from_frame_id'], goal['to_frame_id']))
        self.marker_pub.publish(MarkerArray(goal_markers))

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

    def add_cylinder_marker(self, id, pose_stamped, scale, color=ColorRGBA(0,0,0,1)):
        """

        :param id:
        :param pose_stamped:
        :type pose_stamped: PoseStamped
        :param scale:
        :type scale: Vector3
        :return:
        :rtype: Marker
        """
        m = self.sane_empty_marker(id)
        m.header = pose_stamped.header
        m.action = Marker.ADD
        m.type = Marker.CYLINDER
        m.pose = pose_stamped.pose
        m.scale = scale
        m.color = color
        return m

    def arrow_marker(self, id, from_frame_id, to_frame_id):
        m = self.sane_empty_marker(id)
        m.header.frame_id = from_frame_id
        m.action = Marker.ADD
        m.type = Marker.ARROW
        m.scale = Vector3(0.03, 0.06, 0.0)
        m.color = ColorRGBA(1.0, 0.84, 0.0, 1.0)
        m.points.append(Point(0, 0, 0))
        m.points.append(from_vec3(
            self.tf_listener.lookup_transform(from_frame_id,
                                              to_frame_id,
                                              rospy.Time.now(),
                                              rospy.Duration(1.0)).transform.translation))
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