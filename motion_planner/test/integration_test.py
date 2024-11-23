import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@pytest.mark.rostest
def generate_test_description():
    test1 = Node(package='motion_planner',
                         executable='integration',
                         )
    return (
        LaunchDescription([
            test1,
            launch_testing.actions.ReadyToTest()
            ]),
        # These are extra parameters that get passed to the test functions
        {
            'integration': test1
        }
    )

# Write an integration test that uses the kinematic Franka simulation to 
# command the Franka Panda robot to move from one pose to another and verify that the pose is achieved.

class Integration_Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_static_transform(self, launch_service, in_out, proc_output):
        buffer = Buffer()
        _ = TransformListener(buffer, self.node)
        proc_output.assertWaitFor('Static Transform: world->base', process=in_out, timeout=3.0)
        rclpy.spin_once(self.node)
        xform = buffer.lookup_transform('world', 'base', rclpy.time.Time())


        # Even though these are floating point numbers, test for exact equality
        # Because this transform is hard-coded and not the result of any math
        # So the exact values should be copied
        assert xform.transform.translation.x == 0.0
        assert xform.transform.translation.y == 0.0
        assert xform.transform.translation.z == 1.0
        assert xform.transform.rotation.x == 0.0
        assert xform.transform.rotation.y == 0.0
        assert xform.transform.rotation.z == 0.0
        assert xform.transform.rotation.w == 1.0