# -*- coding: utf-8 -*-
from ..interpreter import model, NodeContext

@model('fetch_gazebo', 'prepare_simulated_robot_pickplace')
def prepare_simulated_robot_pickplace(c: NodeContext) -> None:
    c.action_client("head_controller/follow_joint_trajectory", "control_msgs/FollowJointTrajectoryAction")
    c.read("robot/serial", "ABCDEFGHIJKLMNOPQRSTUVWX")


@model('fetch_gazebo', 'prepare_robot')
def prep_robot(c: NodeContext):
    c.action_client("head_controller/follow_joint_trajectory", "control_msgs/FollowJointTrajectoryAction")
    c.action_client("arm_controller/follow_joint_trajectory", "control_msgs/FollowJointTrajectoryAction")
    c.action_client("gripper_controller/gripper_action", "control_msgs/GripperCommandAction")
    c.read("robot/serial", "ABCDEFGHIJKLMNOPQRSTUVWX")

@model('fetch_gazebo_demo', 'demo')
def demo(c: NodeContext) -> None:
    c.action_client("move_base", "move_base_msgs/MoveBaseAction")
    c.action_client("~follow_joint_trajectory", "control_msgs/FollowJointTrajectoryAction")
    c.action_client("torso_controller/follow_joint_trajectory", "control_msgs/FollowJointTrajectoryAction")
    c.action_client("head_controller/point_head", "control_msgs/PointHeadAction")
    c.action_client("basic_grasping_perception/find_object", "grasping_mgs/FindGraspableObjectsAction")
    c.pub("base_link/planning_scene", "moveit_msgs/PlanningScene")
    c.use("base_link/apply_planning_scene", "moveit_msgs/ApplyPlanningScene")
    c.use("base_link/get_planning_scene", "moveit_msgs/GetPlanningScene")
    c.sub("base_link/move_group/monitored_planning_scene", "moveit_msgs/PlanningScene")
    c.action_client("pickup", "moveit_msgs/PickupAction")
    c.action_client("place", "moveit_msgs/PlaceAction")
    c.action_client("move_group", "moveit_msgs/MoveGroupAction")

@model('fetch_gazebo_demo', 'pick_place_demo.py')
def pick_place_demo(c: NodeContext) -> None:
    c.action_client("head_controller/point_head", "control_msgs/PointHeadAction")
    c.action_client("basic_grasping_perception/find_objectw", "grasping_mgs/FindGraspableObjectsAction")
    c.pub("planning_scene", "moveit_msgs/PlanningScene")
    c.use("apply_planning_scene", "moveit_msgs/ApplyPlanningScene")
    c.use("get_planning_scene", "moveit_msgs/GetPlanningScene")
    c.sub("move_group/monitored_planning_scene", "moveit_msgs/PlanningScene")
    c.action_client("pickup", "moveit_msgs/PickupAction")
    c.action_client("place", "moveit_msgs/PlaceAction")
    c.action_client("move_group", "moveit_msgs/MoveGroupAction")





