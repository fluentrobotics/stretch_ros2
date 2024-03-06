#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/srv/get_parameters.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_control");

int main(int argc, char** argv)
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    int counter;
    int num_sample = 100;
    double min_height = 1.0;
    // rclcpp::NodeOptions node_options.automatically_declare_parameters_from_overrides(true);
    // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("robot_arm_controller");
    rclcpp::NodeOptions node_options = rclcpp::NodeOptions().allow_undeclared_parameters(true);
    auto test_control_node = rclcpp::Node::make_shared("robot_arm_controller", node_options);
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(test_control_node, "/move_group");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(test_control_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(test_control_node->get_logger(), "move_group service not available, waiting again...");
    }
    rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({"robot_description_kinematics"}, 10);
    auto parameters = parameters_client->get_parameters(parameter_list.names);
    test_control_node->set_parameters(parameters);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_control_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Setup
    // static const std::string PLANNING_GROUP_BASE_ARM = "mobile_base_arm";
    static const std::string PLANNING_GROUP_BASE_ARM = "stretch_arm";
    // static const std::string eef_link = "link_grasp_center";
    static const std::string eef_link = "link_wrist_roll";
    // moveit::planning_interface::MoveGroupInterface move_group_base_arm(test_control_node, PLANNING_GROUP_BASE_ARM, nullptr, rclcpp::Duration::from_seconds(-1));
    moveit::planning_interface::MoveGroupInterface move_group_base_arm(test_control_node, PLANNING_GROUP_BASE_ARM);
    auto robot_model = move_group_base_arm.getRobotModel();
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_base_arm.getEndEffectorLink().c_str());

    move_group_base_arm.setMaxVelocityScalingFactor(1.0);
    move_group_base_arm.setMaxAccelerationScalingFactor(1.0);
    move_group_base_arm.setEndEffectorLink(eef_link);
    move_group_base_arm.setStartStateToCurrentState();
    move_group_base_arm.setPlanningTime(20);
    move_group_base_arm.setNumPlanningAttempts(2000);
    move_group_base_arm.allowReplanning(true);
    move_group_base_arm.setReplanAttempts(10);
    // move_group_base_arm.setGoalTolerance(0.01);
    move_group_base_arm.setGoalPositionTolerance(0.05);
    move_group_base_arm.setGoalOrientationTolerance(0.1);
    // move_group_base_arm.setPlannerId("RRTstar");

    // // Get Interface Description
    // moveit_msgs::msg::PlannerInterfaceDescription desc;
    // move_group_base_arm.getInterfaceDescription(desc);
    // RCLCPP_INFO(LOGGER, "Description: %s", desc.name.c_str());
    // RCLCPP_INFO(LOGGER, "pipline: %s", desc.pipeline_id.c_str());
    // for (const auto& planner : desc.planner_ids) {
    //         RCLCPP_INFO(LOGGER, "planner: %s", planner.c_str());
    // }

    // Configure Move Group
    const moveit::core::JointModelGroup* joint_model_group;
    // const moveit::core::LinkModel* ee_parent_link;
    // moveit::core::RobotStatePtr current_state;
    joint_model_group = move_group_base_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE_ARM);
    // ee_parent_link = move_group_base_arm.getCurrentState()->getLinkModel("link_grasp_center");
    // current_state = move_group_base_arm.getCurrentState(10);
    // const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();    

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_base_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_base_arm.getEndEffectorLink().c_str());
    // RCLCPP_INFO(LOGGER, "Current planner id: %s", move_group_base_arm.getPlannerId().c_str());

    auto solver_data = joint_model_group->getGroupKinematics();
    moveit::core::JointModelGroup::KinematicsSolver solver = solver_data.first;
    kinematics::KinematicsBasePtr solver_instance = solver.solver_instance_;
    RCLCPP_INFO(LOGGER, "solver: %s", solver_instance ? "true" : "false");
    RCLCPP_INFO(LOGGER, "getGroupName: %s", solver_instance->getGroupName().c_str());
    RCLCPP_INFO(LOGGER, "getGroupName: %s", solver_instance->getBaseFrame().c_str());

    // Add Collision Obstacles
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_base_arm.getPlanningFrame();
    collision_object.id = "object_id";

    // Define the size of the box in meters
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.07;
    primitive.dimensions[primitive.BOX_Y] = 0.07;
    primitive.dimensions[primitive.BOX_Z] = 0.13;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.0;
    box_pose.position.y = -0.5;
    box_pose.position.z = 0.9;

    // Add the primitive and its pose to the collision object message
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);

    // std::copy(move_group_base_arm.getLinkNames().begin(), move_group_base_arm.getLinkNames().end(),
    //             std::ostream_iterator<std::string>(std::cout, ", "));
    // RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    // std::copy(move_group_base_arm.getJointModelGroupNames().begin(), move_group_base_arm.getJointModelGroupNames().end(),
    //             std::ostream_iterator<std::string>(std::cout, ", "));

    // // Get current joint values
    // std::vector<double> joint_group_positions;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     RCLCPP_INFO(LOGGER, "Old Joint %s: %f", joint_names[i].c_str(), joint_group_positions[i]);
    // }

    // // Plan using random joint values
    // std::vector<double> joint_vales = move_group_base_arm.getRandomJointValues();
    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     RCLCPP_INFO(LOGGER, "New Joint %s: %f", joint_names[i].c_str(), joint_vales[i]);
    // }
    // move_group_base_arm.setJointValueTarget(joint_vales);

    // current_state->setJointGroupPositions(joint_model_group, joint_group_positions);
    // bool within_bounds = move_group_base_arm.setJointValueTarget(joint_group_positions);
    // if (!within_bounds)
    // {
    //     RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }

    // // Get current end-effector pose
    // geometry_msgs::msg::PoseStamped eef_pose = move_group_base_arm.getCurrentPose(eef_link);
    // RCLCPP_INFO(LOGGER, "PoseStamped:\n"
    //             "Header:\n"
    //             "  Frame ID: %s\n"
    //             "  Timestamp: %d\n"
    //             "Pose:\n"
    //             "  Position:\n"
    //             "    x: %.2f\n"
    //             "    y: %.2f\n"
    //             "    z: %.2f\n"
    //             "  Orientation:\n"
    //             "    x: %.2f\n"
    //             "    y: %.2f\n"
    //             "    z: %.2f\n"
    //             "    w: %.2f",
    //             eef_pose.header.frame_id.c_str(),
    //             eef_pose.header.stamp.sec,
    //             eef_pose.pose.position.x, eef_pose.pose.position.y, eef_pose.pose.position.z,
    //             eef_pose.pose.orientation.x, eef_pose.pose.orientation.y,
    //             eef_pose.pose.orientation.z, eef_pose.pose.orientation.w);

    // // Get success rate
    // for (int i = 0; i < num_sample; i++){
    //     geometry_msgs::msg::PoseStamped random_pose = move_group_base_arm.getRandomPose(eef_link);

    //     while (random_pose.pose.position.z < min_height) {
    //         random_pose = move_group_base_arm.getRandomPose(eef_link);
    //     }

    //     geometry_msgs::msg::Pose target_pose;
    //     target_pose.orientation.x = random_pose.pose.orientation.x;
    //     target_pose.orientation.y = random_pose.pose.orientation.y;
    //     target_pose.orientation.z = random_pose.pose.orientation.z;
    //     target_pose.orientation.w = random_pose.pose.orientation.w;
    //     target_pose.position.x = random_pose.pose.position.x;
    //     target_pose.position.y = random_pose.pose.position.y;
    //     target_pose.position.z = random_pose.pose.position.z;

    //     bool success = move_group_base_arm.setApproximateJointValueTarget(target_pose, eef_link);
    //     if (success)
    //     {
    //         moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //         bool planning_success = (move_group_base_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    //         if (planning_success) {
    //             counter++;
    //         }
    //     }
    // }
    // RCLCPP_ERROR(LOGGER, "Success rate: %d/100", counter);


    // // Get random end-effector pose
    // geometry_msgs::msg::PoseStamped random_pose = move_group_base_arm.getRandomPose(eef_link);

    // // Planning to a Pose goal
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.orientation.x = random_pose.pose.orientation.x;
    // target_pose.orientation.y = random_pose.pose.orientation.y;
    // target_pose.orientation.z = random_pose.pose.orientation.z;
    // target_pose.orientation.w = random_pose.pose.orientation.w;
    // target_pose.position.x = random_pose.pose.position.x;
    // target_pose.position.y = random_pose.pose.position.y;
    // target_pose.position.z = random_pose.pose.position.z;
    // // bool success = move_group_base_arm.setPoseTarget(target_pose, eef_link);
    // // bool success = move_group_base_arm.setJointValueTarget(target_pose, eef_link);
    // bool success = move_group_base_arm.setApproximateJointValueTarget(target_pose, eef_link);

    // // Call the planner to compute the plan
    // if (success)
    // {
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     bool planning_success = (move_group_base_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //     // RCLCPP_INFO(LOGGER, "Plan %s", planning_success ? "SUCCESSED" : "FAILED");

    //     if (planning_success) {
    //         // Execute the plan
    //         // move_group_base_arm.move();
    //     } 
    //     else {
    //         RCLCPP_ERROR(LOGGER, "Failed to plan");
    //     }
    // }
    // else
    // {
    //     RCLCPP_ERROR(LOGGER, "Setting approximate joint value target failed");
    // }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}