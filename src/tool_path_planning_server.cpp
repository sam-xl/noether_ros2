#include <noether_ros/conversions.h>
#include <noether_ros/srv/plan_tool_path.hpp>

#include <filesystem>
#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/plugin_interface.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <yaml-cpp/yaml.h>

class ToolPathPlanningServer : public rclcpp::Node
{
public:
  ToolPathPlanningServer() : Node("tool_path_planning_server")
  {
    server_ = this->create_service<noether_ros::srv::PlanToolPath>(
        "plan_tool_path",
        std::bind(&ToolPathPlanningServer::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

protected:
  void callback(const std::shared_ptr<noether_ros::srv::PlanToolPath::Request> request,
                std::shared_ptr<noether_ros::srv::PlanToolPath::Response> response)
  {
    try
    {
      // Convert input configuration string/file to YAML node
      // Load the configuration directly as a YAML-formatted string
      YAML::Node config = YAML::Load(request->config);

      // If the config is a scalar (and not a map), it's likely that the config is actually a file path and not a
      // YAML-formatted string
      if (config.IsScalar())
      {
        // Check if the configuration is an existing file
        if (std::filesystem::is_regular_file(request->config) || std::filesystem::is_symlink(request->config))
        {
          // Reload the configuration from file
          config = YAML::LoadFile(request->config);
        }
        else
        {
          throw std::runtime_error("The request TPP configuration looks like a file path but does not exist: '" +
                                   request->config + "'");
        }
      }

      // Check that the loaded request configuration is a map type
      if (!config.IsMap())
        throw std::runtime_error("The request TPP configuration must be a map type: '" + request->config + "'");

      pcl::PolygonMesh mesh = request->mesh_topic.empty() ? loadMeshFile(request->mesh_file) :
                                                            receiveMesh(request->mesh_topic, request->mesh_timeout);

      // Set the mesh frame from the request
      mesh.header.frame_id = request->mesh_frame;

      // Create and run the tool path planning pipeline
      noether::Factory factory;
      noether::ToolPathPlannerPipeline pipeline(factory, config);
      std::vector<noether::ToolPaths> tool_paths = pipeline.plan(mesh);

      // Fill in the response
      response->success = true;
      response->tool_paths = noether_ros::toMsg(tool_paths, request->mesh_frame);
    }
    catch (const std::exception& ex)
    {
      response->success = false;
      std::stringstream ss;
      noether::printException(ex, ss);
      response->message = ss.str();
    }
  }

  static pcl::PolygonMesh loadMeshFile(const std::string& mesh_file)
  {
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFile(mesh_file, mesh) <= 0)
      throw std::runtime_error("Failed to load mesh from file: " + mesh_file);
    return mesh;
  }

  /**
   * Waits for the mesh latched on `mesh_topic`; a zero `timeout` waits indefinitely.
   *
   * The subscription is created anew for every request, so it receives the mesh latched last.
   */
  pcl::PolygonMesh receiveMesh(const std::string& mesh_topic, const builtin_interfaces::msg::Duration& timeout)
  {
    // A callback group no executor spins, so the message is left for wait_for_message to take
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    const auto subscription = create_subscription<pcl_msgs::msg::PolygonMesh>(
        mesh_topic,
        rclcpp::QoS(1).reliable().transient_local(),
        [](const std::shared_ptr<const pcl_msgs::msg::PolygonMesh>) {},
        options);

    const rclcpp::Duration wait_duration(timeout);
    const auto wait_timeout = wait_duration == rclcpp::Duration(0, 0) ?
                                  std::chrono::nanoseconds(-1) :
                                  wait_duration.to_chrono<std::chrono::nanoseconds>();
    pcl_msgs::msg::PolygonMesh mesh_msg;
    if (!rclcpp::wait_for_message(mesh_msg, subscription, get_node_base_interface()->get_context(), wait_timeout))
    {
      throw std::runtime_error("No mesh received on '" + mesh_topic + "' within " +
                               std::to_string(wait_duration.seconds()) + " s");
    }

    pcl::PolygonMesh mesh;
    pcl_conversions::toPCL(mesh_msg, mesh);
    return mesh;
  }

  rclcpp::Service<noether_ros::srv::PlanToolPath>::SharedPtr server_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolPathPlanningServer>();
  RCLCPP_INFO_STREAM(node->get_logger(), "Started tool path planning server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
