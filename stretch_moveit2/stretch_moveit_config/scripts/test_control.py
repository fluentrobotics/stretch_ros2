import time
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped, Pose
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters

def plan_and_execute(robot, logger, planning_component, single_plan_parameters=None, multi_plan_parameters=None, sleep_time=0.01):
    # plan to goal
    logger.info("Planning trajectory")

    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, blocking=True, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

def main():
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # MoveItPy instance and get planning component
    stretch = MoveItPy(node_name="moveit_py")
    stretch_base_arm = stretch.get_planning_component("mobile_base_arm")
    logger.info("MoveItPy instance created")

    robot_model = stretch.get_robot_model()
    robot_state = RobotState(robot_model)

    # set plan start state to current state
    stretch_base_arm.set_start_state_to_current_state()

    # set pose goal
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = -0.02
    pose_goal.pose.position.y = -0.50
    pose_goal.pose.position.z = 0.70

    # set goal state
    stretch_base_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_grasp_center")

    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        stretch, ["ompl_rrtc", "pilz_lin", "chomp", "ompl_rrt_star"]
    )

    # plan to goal
    plan_and_execute(
        stretch,
        stretch_base_arm,
        logger,
        plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=1.0)

if __name__ == "__main__":
    main()