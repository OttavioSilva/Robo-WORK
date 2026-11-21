"""Check the actual ROS adapter never exports metrics after tracking loss."""
import json
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from robo_virtual_wall.node import VirtualWall


def test_tracking_loss_suppresses_pixels_metrics_and_clears_markers():
    root = Path(__file__).resolve().parents[1]
    rclpy.init(domain_id=138, args=['--ros-args', '-p', 'image_topic:=/test/arm_image',
        '-p', 'self_mask_mode:=dynamic',
        '-p', 'self_mask_file:='+str(root/'config/dynamic_gripper_20261113.json'),
        '-p', 'homography_file:='+str(root/'calibration/front_base_link_expanded_validated_20261113.json')])
    node = VirtualWall()
    try:
        statuses, markers = [], []
        node.status_pub.publish = lambda msg: statuses.append(json.loads(msg.data))
        node.markers_pub.publish = markers.append
        header = Image().header
        header.stamp = node.get_clock().now().to_msg()
        node.process(node.self_mask.reference.copy(), header)
        assert statuses[-1]['perception_valid']
        assert statuses[-1]['metric_frame'] == 'base_link'
        node.process(np.full_like(node.self_mask.reference, 220), header)
        assert statuses[-1]['perception_valid'] is False
        assert statuses[-1]['metric_frame'] == ''
        assert statuses[-1]['detections'] == []
        assert len(markers[-1].markers) == 1
        assert markers[-1].markers[0].action == markers[-1].markers[0].DELETEALL
    finally:
        node.destroy_node()
        rclpy.shutdown()
