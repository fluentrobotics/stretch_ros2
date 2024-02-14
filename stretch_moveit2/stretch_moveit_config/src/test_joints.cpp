#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <chrono>
#include "rcl_interfaces/srv/get_parameters.hpp"
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::NodeOptions node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions().allow_undeclared_parameters(true);
  auto node = rclcpp::Node::make_shared("testing", node_options);

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return 0;
      }
      RCLCPP_INFO(node->get_logger(), "move_group service not available, waiting again...");
  }
  // try {
  //     auto parameter_value = parameters_client->get_parameter<std::string>(
  //         "robot_description_kinematics.mobile_base_arm.kinematics_solver");
  //     RCLCPP_INFO(node->get_logger(), "Kinematics Solver: %s", parameter_value.c_str());
  // } catch (const std::exception& e) {
  //     RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: %s", e.what());
  // }

  const auto& LOGGER = node->get_logger();
  const std::string eef_link = "link_grasp_center";

  rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({"robot_description_kinematics"}, 10);
  auto parameters = parameters_client->get_parameters(parameter_list.names);
  node->set_parameters(parameters);
  // std::string robot_description = "robot_description";
  // auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(robot_model_loader::RobotModelLoader(node, robot_description, false));
  // loader->loadKinematicsSolvers();

  robot_model_loader::RobotModelLoader robot_model_loader(node);
  // robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description", true);
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  robot_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("mobile_base_arm");
  // joint_model_group->setEndEffectorName(eef_link);

  // const std::string desc = robot_model_loader.getRobotDescription();
  // RCLCPP_INFO(LOGGER, "Description: %s", desc.c_str());
  // kinematics_plugin_loader::KinematicsPluginLoaderPtr kpl =	robot_model_loader.getKinematicsPluginLoader();
  // robot_model_loader.loadKinematicsSolvers(kpl);
  // const srdf::ModelSharedPtr srdf = robot_model_loader.getSRDF(); 
  // moveit::core::SolverAllocatorFn saf = kpl->getLoaderFunction(srdf);
  // kinematics::KinematicsBasePtr kb = saf(joint_model_group);

  // auto solver_data = joint_model_group->getGroupKinematics();
  // moveit::core::JointModelGroup::KinematicsSolver solver = solver_data.first;
  // RCLCPP_INFO(LOGGER, "solver: %s", solver.solver_instance_ ? "true" : "false");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  // for (std::size_t i = 0; i < joint_names.size(); ++i)
  // {
  //   RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  // }

  // Joint Limits, setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  joint_values[3] = 0.6;
  robot_state->setJointGroupPositions(joint_model_group, joint_values);

  // Check whether any joint is outside its joint limits
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  // Enforce the joint limits for this state and check again
  robot_state->enforceBounds();
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  robot_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("link_grasp_center");

  // Print end-effector pose. Remember that this is in the model frame 
  RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");
  RCLCPP_INFO(LOGGER, "End effector link: %s", joint_model_group->getEndEffectorName().c_str());

  // Inverse Kinematics
  double timeout = 0.1;
  bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Did not find IK solution");
  }

  // Get the Jacobian
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                           reference_point_position, jacobian);
  RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

  rclcpp::shutdown();
  return 0;
}