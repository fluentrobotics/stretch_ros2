#include <rclcpp/rclcpp.hpp>
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
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto test_control_node = rclcpp::Node::make_shared("robot_arm_controller", node_options);
    
    // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("robot_arm_controller");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_control_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Setup
    static const std::string PLANNING_GROUP_BASE_ARM = "mobile_base_arm";
    static const std::string eef_link = "link_grasp_center";
    moveit::planning_interface::MoveGroupInterface move_group_base_arm(test_control_node, PLANNING_GROUP_BASE_ARM, nullptr, rclcpp::Duration::from_seconds(-1));
    auto robot_model = move_group_base_arm.getRobotModel();

    move_group_base_arm.setMaxVelocityScalingFactor(1.0);
    move_group_base_arm.setMaxAccelerationScalingFactor(1.0);
    move_group_base_arm.setEndEffectorLink(eef_link);
    move_group_base_arm.setStartStateToCurrentState();
    move_group_base_arm.setPlanningTime(20);
    move_group_base_arm.setNumPlanningAttempts(10);
    move_group_base_arm.allowReplanning(true);
    move_group_base_arm.setReplanAttempts(10);
    move_group_base_arm.setGoalTolerance(0.01);

    // Configure Move Group
    const moveit::core::JointModelGroup* joint_model_group;
    const moveit::core::LinkModel* ee_parent_link;
    moveit::core::RobotStatePtr current_state;
    joint_model_group = move_group_base_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_BASE_ARM);
    ee_parent_link = move_group_base_arm.getCurrentState()->getLinkModel("link_grasp_center");
    current_state = move_group_base_arm.getCurrentState(10);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();    

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_base_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_base_arm.getEndEffectorLink().c_str());

    auto solver_data = joint_model_group->getGroupKinematics();
    moveit::core::JointModelGroup::KinematicsSolver solver = solver_data.first;
    // RCLCPP_INFO(LOGGER, "solver: %s", solver.solver_instance_ ? "true" : "false");

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

    // Get current end-effector pose
    geometry_msgs::msg::PoseStamped eef_pose = move_group_base_arm.getCurrentPose(eef_link);
    RCLCPP_INFO(LOGGER, "PoseStamped:\n"
                "Header:\n"
                "  Frame ID: %s\n"
                "  Timestamp: %d\n"
                "Pose:\n"
                "  Position:\n"
                "    x: %.2f\n"
                "    y: %.2f\n"
                "    z: %.2f\n"
                "  Orientation:\n"
                "    x: %.2f\n"
                "    y: %.2f\n"
                "    z: %.2f\n"
                "    w: %.2f",
                eef_pose.header.frame_id.c_str(),
                eef_pose.header.stamp.sec,
                eef_pose.pose.position.x, eef_pose.pose.position.y, eef_pose.pose.position.z,
                eef_pose.pose.orientation.x, eef_pose.pose.orientation.y,
                eef_pose.pose.orientation.z, eef_pose.pose.orientation.w);

    // Get random end-effector pose
    geometry_msgs::msg::PoseStamped random_pose = move_group_base_arm.getRandomPose(eef_link);
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
    //             random_pose.header.frame_id.c_str(),
    //             random_pose.header.stamp.sec,
    //             random_pose.pose.position.x, random_pose.pose.position.y, random_pose.pose.position.z,
    //             random_pose.pose.orientation.x, random_pose.pose.orientation.y,
    //             random_pose.pose.orientation.z, random_pose.pose.orientation.w);

    // Planning to a Pose goal
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = eef_pose.pose.orientation.x;
    target_pose.orientation.y = eef_pose.pose.orientation.y;
    target_pose.orientation.z = eef_pose.pose.orientation.z;
    target_pose.orientation.w = eef_pose.pose.orientation.w;
    target_pose.position.x = eef_pose.pose.position.x;
    target_pose.position.y = eef_pose.pose.position.y;
    target_pose.position.z = 0.3;
    // target_pose.orientation.x = random_pose.pose.orientation.x;
    // target_pose.orientation.y = random_pose.pose.orientation.y;
    // target_pose.orientation.z = random_pose.pose.orientation.z;
    // target_pose.orientation.w = random_pose.pose.orientation.w;
    // target_pose.position.x = random_pose.pose.position.x;
    // target_pose.position.y = random_pose.pose.position.y;
    // target_pose.position.z = random_pose.pose.position.z;
    // move_group_base_arm.setPoseTarget(target_pose, eef_link);
    // move_group_base_arm.setJointValueTarget(target_pose, eef_link);
    move_group_base_arm.setApproximateJointValueTarget(target_pose, eef_link);
    // std::vector<double> target_joints;
    // move_group_base_arm.getJointValueTarget(target_joints);
    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     RCLCPP_INFO(LOGGER, "Target Joint %s: %f", joint_names[i].c_str(), target_joints[i]);
    // }

    // Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;
    success = (move_group_base_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan %s", success ? "" : "FAILED");

    if (success) {
        // Execute the plan
        move_group_base_arm.move();
    } 
    else {
        RCLCPP_ERROR(LOGGER, "Failed to plan");
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}