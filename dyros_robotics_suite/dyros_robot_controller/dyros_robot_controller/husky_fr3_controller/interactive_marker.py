#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer


def main():
    rclpy.init(args=sys.argv)
    node: Node = rclpy.create_node('husky_fr3_interactive_marker')

    # Parameters
    frame_id = node.declare_parameter('frame_id', 'world').get_parameter_value().string_value
    goal_topic = node.declare_parameter('goal_topic', 'husky_fr3_controller/target_pose').get_parameter_value().string_value

    goal_pub = node.create_publisher(PoseStamped, goal_topic, 10)

    # create an interactive marker server on the namespace simple_marker
    server = InteractiveMarkerServer(node, 'simple_marker')

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.name = 'ee_goal'
    int_marker.description = 'EE Goal 6-DOF'
    int_marker.scale = 0.2

    # visual marker (sphere)
    sphere = Marker()
    sphere.type = Marker.SPHERE
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08
    sphere.color.r = 0.1
    sphere.color.g = 0.8
    sphere.color.b = 0.2
    sphere.color.a = 0.95

    base_ctrl = InteractiveMarkerControl()
    base_ctrl.always_visible = True
    base_ctrl.markers.append(sphere)
    int_marker.controls.append(base_ctrl)

    # 6-DOF controls
    ctrl = InteractiveMarkerControl()
    ctrl.orientation.w = 1.0
    ctrl.orientation.x = 1.0
    ctrl.orientation.y = 0.0
    ctrl.orientation.z = 0.0
    ctrl.name = 'rotate_x'
    ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(ctrl)

    ctrl = InteractiveMarkerControl()
    ctrl.orientation.w = 1.0
    ctrl.orientation.x = 0.0
    ctrl.orientation.y = 1.0
    ctrl.orientation.z = 0.0
    ctrl.name = 'rotate_y'
    ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(ctrl)

    ctrl = InteractiveMarkerControl()
    ctrl.orientation.w = 1.0
    ctrl.orientation.x = 0.0
    ctrl.orientation.y = 0.0
    ctrl.orientation.z = 1.0
    ctrl.name = 'rotate_z'
    ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(ctrl)

    ctrl = InteractiveMarkerControl()
    ctrl.orientation.w = 1.0
    ctrl.orientation.x = 1.0
    ctrl.orientation.y = 0.0
    ctrl.orientation.z = 0.0
    ctrl.name = 'move_x'
    ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(ctrl)

    ctrl = InteractiveMarkerControl()
    ctrl.orientation.w = 1.0
    ctrl.orientation.x = 0.0
    ctrl.orientation.y = 1.0
    ctrl.orientation.z = 0.0
    ctrl.name = 'move_y'
    ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(ctrl)

    ctrl = InteractiveMarkerControl()
    ctrl.orientation.w = 1.0
    ctrl.orientation.x = 0.0
    ctrl.orientation.y = 0.0
    ctrl.orientation.z = 1.0
    ctrl.name = 'move_z'
    ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(ctrl)

    # Publish on user feedback
    def process_feedback(feedback):
        msg = PoseStamped()
        msg.header = feedback.header
        if not msg.header.frame_id:
            msg.header.frame_id = frame_id
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose = feedback.pose
        goal_pub.publish(msg)

    server.insert(int_marker, feedback_callback=process_feedback)
    server.applyChanges()

    # Initialize/reset logic from end-effector pose and mode
    last_ee_pose = {'msg': None}
    initialized_from_ee = {'done': False}

    def maybe_publish_goal_from_pose(pose_msg: PoseStamped):
        out = PoseStamped()
        out.header.frame_id = pose_msg.header.frame_id or frame_id
        out.header.stamp = node.get_clock().now().to_msg()
        out.pose = pose_msg.pose
        goal_pub.publish(out)

    def ee_cb(msg: PoseStamped):
        last_ee_pose['msg'] = msg
        if not initialized_from_ee['done']:
            # Set marker to current EE pose on first message
            server.setPose(int_marker.name, msg.pose)
            server.applyChanges()
            initialized_from_ee['done'] = True
            # Also publish as initial goal
            maybe_publish_goal_from_pose(msg)

    def mode_cb(m: Int32):
        # When switching to MPPI (5), reset marker to current EE pose and publish it
        if m.data == 5 and last_ee_pose['msg'] is not None:
            server.setPose(int_marker.name, last_ee_pose['msg'].pose)
            server.applyChanges()
            maybe_publish_goal_from_pose(last_ee_pose['msg'])

    node.create_subscription(PoseStamped, 'husky_fr3_controller/ee_pose', ee_cb, 10)
    node.create_subscription(Int32, 'husky_fr3_controller/mode_input', mode_cb, 10)

    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()