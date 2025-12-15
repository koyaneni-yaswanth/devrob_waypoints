#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/pose.hpp>

// MoveIt2
#include <moveit/move_group_interface/move_group_interface.h>

// YAML
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

// Simple struct to hold a waypoint
struct Waypoint
{
  std::string name;
  geometry_msgs::msg::Pose pose;
};

static std::vector<Waypoint> load_waypoints(
  const std::string & yaml_file,
  const rclcpp::Logger & logger)
{
  std::vector<Waypoint> waypoints;

  try
  {
    YAML::Node root = YAML::LoadFile(yaml_file);
    if (!root["waypoints"]) {
      RCLCPP_ERROR(logger, "YAML file '%s' has no 'waypoints' key", yaml_file.c_str());
      return waypoints;
    }

    for (const auto & node : root["waypoints"])
    {
      Waypoint wp;
      wp.name = node["name"] ? node["name"].as<std::string>() : std::string("unnamed");

      auto pos = node["position"];
      auto ori = node["orientation"];

      if (!pos || !ori || pos.size() != 3 || ori.size() != 4) {
        RCLCPP_ERROR(
          logger,
          "Invalid waypoint format in '%s' (need position[3], orientation[4])",
          yaml_file.c_str());
        continue;
      }

      wp.pose.position.x = pos[0].as<double>();
      wp.pose.position.y = pos[1].as<double>();
      wp.pose.position.z = pos[2].as<double>();

      wp.pose.orientation.x = ori[0].as<double>();
      wp.pose.orientation.y = ori[1].as<double>();
      wp.pose.orientation.z = ori[2].as<double>();
      wp.pose.orientation.w = ori[3].as<double>();

      waypoints.push_back(wp);
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger, "Failed to load YAML '%s': %s", yaml_file.c_str(), e.what());
  }

  return waypoints;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waypoint_node");
  auto logger = node->get_logger();

  // ---- 1) Local parameters: planning group + waypoints file ----
  std::string planning_group =
    node->declare_parameter<std::string>("planning_group", "panda_arm");
  std::string waypoints_file =
    node->declare_parameter<std::string>("waypoints_file", "");

  if (waypoints_file.empty()) {
    RCLCPP_ERROR(logger, "Parameter 'waypoints_file' is empty. Exiting.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Using planning group: %s", planning_group.c_str());
  RCLCPP_INFO(logger, "Loading waypoints from: %s", waypoints_file.c_str());

  // ---- 2) Copy MoveIt robot model parameters from /move_group ----
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "move_group");

  RCLCPP_INFO(logger, "Waiting for move_group parameter service...");
  while (!param_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for move_group service");
      return 1;
    }
    RCLCPP_INFO(logger, "Still waiting for move_group...");
  }

  const std::vector<std::string> param_names = {
    "robot_description",
    "robot_description_semantic",
    "robot_description_kinematics"
  };

  auto future = param_client->get_parameters(param_names);
  if (rclcpp::spin_until_future_complete(node, future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to get parameters from move_group");
    rclcpp::shutdown();
    return 1;
  }

  auto params = future.get();
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name.empty()) {
      continue;
    }

    // All these MoveIt parameters are strings on move_group
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      auto value = p.as_string();
      node->declare_parameter<std::string>(name, value);
      RCLCPP_INFO(logger, "Copied string parameter '%s' from move_group", name.c_str());
    } else {
      RCLCPP_WARN(logger,
        "Parameter '%s' from move_group is not a string (type %d) – skipping",
        name.c_str(), static_cast<int>(p.get_type()));
    }
  }

  // ---- 3) Load waypoints from YAML ----
  auto waypoints = load_waypoints(waypoints_file, logger);
  if (waypoints.empty()) {
    RCLCPP_ERROR(logger, "No valid waypoints loaded – exiting.");
    rclcpp::shutdown();
    return 1;
  }

  // ---- 4) Create MoveGroupInterface (now robot model params are set) ----
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, planning_group);

  RCLCPP_INFO(logger, "Connected to MoveGroupInterface");

  // ---- 5) Plan and execute each waypoint ----
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    const auto & wp = waypoints[i];
    RCLCPP_INFO(logger, "Planning to waypoint %zu: '%s'", i, wp.name.c_str());

    move_group.setPoseTarget(wp.pose);
    MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Plan to '%s' succeeded. Executing...", wp.name.c_str());
      auto exec_result = move_group.execute(plan);
      if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger, "Execution for '%s' failed", wp.name.c_str());
      } else {
        RCLCPP_INFO(logger, "Execution for '%s' finished", wp.name.c_str());
      }
    } else {
      RCLCPP_WARN(logger, "Planning to '%s' failed (error code %d)",
                  wp.name.c_str(), result.val);
    }

    move_group.stop();
    move_group.clearPoseTargets();
  }

  RCLCPP_INFO(logger, "Finished executing all waypoints.");
  rclcpp::shutdown();
  return 0;
}
