#!/usr/bin/env python3
"""
Localize on a map whose grid is rotated (info.origin.orientation, i.e. the
yaw of map_server's "origin: [x, y, yaw]") and check that amcl_pose is
reported at the true pose in the global frame.

The test publishes the map, a simulated laser scan (ray cast in the grid)
and static tfs itself, so no bag file or map file is needed.
"""
import math
import sys
import unittest

import numpy as np
import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

ORIGIN_X, ORIGIN_Y, RESOLUTION, SIZE = 2.0, 3.0, 0.05, 240


def rotate(x, y, angle):
    return x * math.cos(angle) - y * math.sin(angle), x * math.sin(angle) + y * math.cos(angle)


def make_grid():
    grid = np.zeros((SIZE, SIZE), dtype=np.int8)  # [row = y, col = x]
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    # some asymmetric structure so the pose is unique
    grid[60:140, 80] = 100
    grid[150, 30:120] = 100
    grid[40:70, 170:200] = 100
    grid[180:200, 150:160] = 100
    return grid


class TestRotatedMapLocalization(unittest.TestCase):

    def setUp(self):
        self.yaw = rospy.get_param("~map_yaw")
        self.grid = make_grid()
        self.last_pose = None
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

    def pose_callback(self, msg):
        self.last_pose = msg.pose.pose

    def scan_ranges(self, grid_x, grid_y, grid_theta, beams=180):
        ranges = []
        for i in range(beams):
            angle = grid_theta - math.pi + 2 * math.pi * i / beams
            r = 0.0
            while r < 15.0:
                r += RESOLUTION / 2
                cx = int((grid_x + r * math.cos(angle) - ORIGIN_X) / RESOLUTION)
                cy = int((grid_y + r * math.sin(angle) - ORIGIN_Y) / RESOLUTION)
                if not (0 <= cx < SIZE and 0 <= cy < SIZE) or self.grid[cy, cx] == 100:
                    break
            ranges.append(r)
        return ranges

    def test_pose_in_global_frame(self):
        # true robot pose, defined in the grid and expressed in the global frame
        grid_x, grid_y, grid_theta = ORIGIN_X + 5.5, ORIGIN_Y + 4.0, 0.4
        dx, dy = rotate(grid_x - ORIGIN_X, grid_y - ORIGIN_Y, self.yaw)
        true_x, true_y, true_theta = ORIGIN_X + dx, ORIGIN_Y + dy, grid_theta + self.yaw

        map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=1, latch=True)
        scan_pub = rospy.Publisher("scan", LaserScan, queue_size=1)
        initial_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)

        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = RESOLUTION
        grid_msg.info.width = grid_msg.info.height = SIZE
        grid_msg.info.origin.position.x = ORIGIN_X
        grid_msg.info.origin.position.y = ORIGIN_Y
        grid_msg.info.origin.orientation = Quaternion(*quaternion_from_euler(0, 0, self.yaw))
        grid_msg.data = self.grid.flatten().tolist()
        map_pub.publish(grid_msg)

        static_tf = tf2_ros.StaticTransformBroadcaster()
        transforms = []
        for parent, child in (("odom", "base_link"), ("base_link", "laser")):
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            transforms.append(t)
        static_tf.sendTransform(transforms)
        rospy.sleep(3.0)

        # initial guess in the global frame, slightly off
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.pose.pose.position.x = true_x + 0.25
        initial.pose.pose.position.y = true_y - 0.2
        initial.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, true_theta + 0.08))
        initial.pose.covariance[6 * 0 + 0] = initial.pose.covariance[6 * 1 + 1] = 0.1
        initial.pose.covariance[6 * 5 + 5] = 0.05
        initial_pose_pub.publish(initial)
        rospy.sleep(1.0)

        rospy.wait_for_service("request_nomotion_update", 10.0)
        nomotion_update = rospy.ServiceProxy("request_nomotion_update", Empty)
        ranges = self.scan_ranges(grid_x, grid_y, grid_theta)
        for _ in range(60):
            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id = "laser"
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / len(ranges)
            scan.angle_max = scan.angle_min + scan.angle_increment * (len(ranges) - 1)
            scan.range_min, scan.range_max = 0.05, 20.0
            scan.ranges = ranges
            nomotion_update()
            scan_pub.publish(scan)
            rospy.sleep(0.1)
        rospy.sleep(0.5)

        self.assertIsNotNone(self.last_pose, "no amcl_pose received")
        p = self.last_pose
        theta = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[2]
        theta_error = (theta - true_theta + math.pi) % (2 * math.pi) - math.pi
        self.assertLess(math.hypot(p.position.x - true_x, p.position.y - true_y), 0.05)
        self.assertLess(abs(theta_error), math.radians(1.5))


if __name__ == "__main__":
    rospy.init_node("rotated_map_localization_test", anonymous=True)
    rostest.run("amcl", "rotated_map_localization", TestRotatedMapLocalization, sys.argv)
