from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
import launch_testing
from builtin_interfaces.msg import Duration
from noether_ros.srv import PlanToolPath
import os
from pcl_msgs.msg import PolygonMesh, Vertices
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import unittest


def generate_test_description():
    return LaunchDescription([
            LaunchNode(
                package='noether_ros',
                executable='noether_ros_tool_path_planning_server',
                output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])


config_str = R"""
mesh_modifiers:
  - name: NormalsFromMeshFaces
tool_path_planner:
  name: Multi
  gui_plugin_name: CrossHatchPlaneSlicer
  planners:
    - name: PlaneSlicer
      direction_generator:
        name: PrincipalAxis
        rotation_offset: 1.571
      origin_generator:
        name: AABBCenter
      line_spacing: 0.1
      point_spacing: 0.025
      min_hole_size: 0.100
      search_radius: 0.100
      min_segment_size: 0.100
      bidirectional: true
    - name: PlaneSlicer
      direction_generator:
        name: PCARotated
        direction_generator:
          name: PrincipalAxis
          rotation_offset: 1.571
        rotation_offset: 1.571
      origin_generator:
        name: AABBCenter
      line_spacing: 0.100
      point_spacing: 0.025
      min_hole_size: 0.100
      search_radius: 0.100
      min_segment_size: 0.100
      bidirectional: true
tool_path_modifiers:
  - name: SnakeOrganization
"""

def grid_mesh(size=1.0, cells=20):
    """Return a flat square mesh in the xy plane, triangulated on a grid of `cells` by `cells`."""
    step = size / cells
    points = [(i * step, j * step, 0.0) for j in range(cells + 1) for i in range(cells + 1)]
    fields = [PointField(name=axis, offset=4 * index, datatype=PointField.FLOAT32, count=1)
              for index, axis in enumerate("xyz")]
    mesh = PolygonMesh(cloud=point_cloud2.create_cloud(Header(frame_id="world"), fields, points))
    for j in range(cells):
        for i in range(cells):
            corner = j * (cells + 1) + i
            mesh.polygons.append(Vertices(vertices=[corner, corner + 1, corner + cells + 2]))
            mesh.polygons.append(Vertices(vertices=[corner, corner + cells + 2, corner + cells + 1]))
    return mesh


# Active test
class TestToolPathPlanningClient(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.node = rclpy.create_node("test_client")

    @classmethod
    def tearDownClass(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_tool_path_planning_client(self):
        # Create the client
        client = self.node.create_client(PlanToolPath, 'plan_tool_path')

        # Check that the service is available
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), "Service does not exist")

        # Create the request
        request = PlanToolPath.Request()

        # Add the TPP config string to the request
        request.config = config_str

        # Add the mesh file to the request
        pkg_prefix = get_package_share_directory('noether_ros')
        request.mesh_file = os.path.join(pkg_prefix, 'test', 'dome.ply')

        # Check that the mesh file is valid
        self.assertIsNotNone(pkg_prefix)
        self.assertTrue(os.path.exists(request.mesh_file))

        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the response
        response = future.result()
        self.assertIsNotNone(response, "No response received")
        self.assertTrue(response.success, response.message)
        self.assertEqual(len(response.tool_paths), 1)
        for tool_paths in response.tool_paths:
            self.assertGreaterEqual(len(tool_paths.tool_paths), 1)
            for tool_path in tool_paths.tool_paths:
                self.assertGreaterEqual(len(tool_path.segments), 1)
                for segment in tool_path.segments:
                    self.assertGreaterEqual(len(segment.poses), 1)

        self.node.destroy_client(client)

    def call(self, request):
        client = self.node.create_client(PlanToolPath, 'plan_tool_path')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), "Service does not exist")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.destroy_client(client)
        self.assertIsNotNone(future.result(), "No response received")
        return future.result()

    def test_tool_path_planning_from_topic(self):
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = self.node.create_publisher(PolygonMesh, 'test_mesh', latched)
        publisher.publish(grid_mesh())

        request = PlanToolPath.Request(config=config_str, mesh_topic='test_mesh', mesh_timeout=Duration(sec=5))
        response = self.call(request)

        self.assertTrue(response.success, response.message)
        self.assertEqual(len(response.tool_paths), 1)
        self.assertGreaterEqual(len(response.tool_paths[0].tool_paths), 1)
        self.node.destroy_publisher(publisher)

    def test_tool_path_planning_from_topic_timeout(self):
        request = PlanToolPath.Request(config=config_str, mesh_topic='nobody_publishes_here',
                                       mesh_timeout=Duration(sec=1))
        response = self.call(request)

        self.assertFalse(response.success)
        self.assertIn("No mesh received on 'nobody_publishes_here'", response.message)
