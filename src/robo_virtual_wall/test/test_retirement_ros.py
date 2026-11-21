import json
import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import ExtrapolationException
from robo_virtual_wall.keepout_node import TapeKeepout


@pytest.fixture
def rig():
    rclpy.init(domain_id=139, args=['--ros-args', '-p', 'remove_visible_missing:=true'])
    node = TapeKeepout()
    clock = [100.]
    class Clock:
        def now(self): return Time(seconds=clock[0])
    node.get_clock = lambda: Clock()
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 100
    grid.info.resolution = .05
    grid.info.origin.orientation.w = 1.
    node.on_map(grid)
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    node.tf_buffer.lookup_transform = lambda *args: tf
    view = [[.4, -.5], [1.4, -.5], [1.4, .5], [.4, .5]]
    wall = [[.8, -.2], [.8, .2]]
    def step(stamp, positive=False, motion=False, valid=True, visible=True, odom=True):
        ns = int(round(stamp*1e9))
        clock[0] = ns/1e9 + .001  # Arrival follows capture; avoid floating-point future stamps.
        if odom:
            msg = Odometry()
            msg.header.stamp.sec, msg.header.stamp.nanosec = ns//10**9, ns%10**9
            msg.twist.twist.linear.x = .1 if motion else 0.
            node.on_odom(msg)
        data = dict(metric_frame='base_link', perception_valid=valid,
                    stamp=dict(sec=ns//10**9, nanosec=ns%10**9),
                    observed_ground_base_link_m=view if visible else None,
                    detections=[dict(base_link_m=wall)] if positive else [])
        message = String(data=json.dumps(data))
        node.on_detections(message)
        return message
    for stamp in (100., 100.1, 100.2): step(stamp, positive=True)
    assert len(node.tracker.walls) == 1
    yield node, clock, step
    node.destroy_node()
    rclpy.shutdown()


def test_stationary_visible_removal_publishes_free_mask(rig):
    node, clock, step = rig
    masks = []
    node.mask_pub.publish = masks.append
    node.publish()
    assert 100 in masks[-1].data
    for stamp in np.arange(100.3, 103.6, .1): step(float(stamp))
    assert node.tracker.walls == []
    node.publish()
    assert not any(masks[-1].data)


@pytest.mark.parametrize('condition', ['moving', 'no_odom', 'no_view', 'invalid'])
def test_no_removal_without_stationary_current_valid_evidence(rig, condition):
    node, clock, step = rig
    for stamp in np.arange(100.3, 106., .1):
        step(float(stamp), motion=condition == 'moving', odom=condition != 'no_odom',
             visible=condition != 'no_view', valid=condition != 'invalid')
    assert len(node.tracker.walls) == 1


def test_brief_tf_delay_retries_exact_image_then_allows_removal(rig):
    node, clock, step = rig
    transform = TransformStamped()
    transform.transform.rotation.w = 1.
    def missing(*args): raise ExtrapolationException('not ready yet')
    for stamp in np.arange(100.3, 103.6, .1):
        node.tf_buffer.lookup_transform = missing
        step(float(stamp))
        assert node.waiting_tf
        node.tf_buffer.lookup_transform = lambda *args: transform
        node.retry_waiting_tf()
        assert not node.waiting_tf
    assert node.tracker.walls == []
