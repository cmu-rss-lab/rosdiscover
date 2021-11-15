# -*- coding: utf-8 -*-
from ...interpreter import generate_dynamic_plugin, model, ModelPlugin, NodeContext

DEFAULT_CAPABILITIES = {
    "move_group/MoveGroupCartesianPathService": 'MoveGroupCartesianPathService',
    "move_group/MoveGroupKinematicsService": 'MoveGroupKinematicsService',
    "move_group/MoveGroupExecuteTrajectoryAction": "MoveGroupExecuteTrajectoryAction",
    "move_group/MoveGroupMoveAction": "MoveGroupMoveAction",
    "move_group/MoveGroupPickPlaceAction": "MoveGroupPickPlaceAction",
    "move_group/MoveGroupPlanService": "MoveGroupPlanService",
    "move_group/MoveGroupQueryPlannersService": "MoveGroupQueryPlannersService",
    "move_group/MoveGroupStateValidationService": "MoveGroupStateValidationService",
    "move_group/MoveGroupGetPlanningSceneService": "MoveGroupGetPlanningSceneService",
    "move_group/ApplyPlanningSceneService": "ApplyPlanningSceneService",
    "move_group/ClearOctomapService": "ClearOctomapService",
}

PLANNER_SERVICE_NAME = "plan_kinematic_path"  # name of the advertised service (within the ~ namespace) x
EXECUTE_ACTION_NAME = "execute_trajectory"  # name of 'execute' action
QUERY_PLANNERS_SERVICE_NAME = "query_planner_interface"  # name of the advertised query planners service x
GET_PLANNER_PARAMS_SERVICE_NAME = "get_planner_params"  # service name to retrieve planner parameters x
SET_PLANNER_PARAMS_SERVICE_NAME = "set_planner_params"   # service name to set planner parameters x
MOVE_ACTION = "move_group"      # name of 'move' action
IK_SERVICE_NAME = "compute_ik"  # name of ik service x
FK_SERVICE_NAME = "compute_fk"  # name of fk service x
STATE_VALIDITY_SERVICE_NAME = "check_state_validity"  # name of the service that validates states
CARTESIAN_PATH_SERVICE_NAME = "compute_cartesian_path"  # name of the service that computes cartesian paths x
GET_PLANNING_SCENE_SERVICE_NAME = "get_planning_scene"  # name of the service that can be used to query the planning
                                                        # scene x
APPLY_PLANNING_SCENE_SERVICE_NAME = "apply_planning_scene"  # name of the service that applies a given
                                                            # planning scene x
CLEAR_OCTOMAP_SERVICE_NAME = "clear_octomap"  # name of the service that can be used to clear the octomap
PICKUP_ACTION = "pickup"  # name of 'pickup' action
PLACE_ACTION = "place"
LOAD_MAP_SERVICE_NAME = "load_map"
SAVE_MAP_SERVICE_NAME = "save_map"
CHECK_STATE_VALIDITY_SERVICE_NAME = "check_state_validity"

@model('moveit', 'move_group')
def move_group(c: NodeContext) -> None:
    c.read("~allow_trajectory_execution", True)

    planning_pipelines = c.read("~planning_pipelines", [])
    default_planning_pipeline = c.read("~default_planning_pipeline", "")
    if default_planning_pipeline not in planning_pipelines:
        default_planning_pipeline = ""

    if not default_planning_pipeline:
        if not planning_pipelines:
            default_planning_pipeline = "~"
        else:
            default_planning_pipeline = planning_pipelines[0]
    c.write("~default_planning_pipeline", default_planning_pipeline)

    # capability_plugin_loader_ = std::make_shared<pluginlib::ClassLoader<MoveGroupCapability>>(
    #      "moveit_ros_move_group", "move_group::MoveGroupCapability");
    my_plugins = dict(DEFAULT_CAPABILITIES)

    for capability in c.read("~capabilities", []):
        my_plugins[capability] = generate_dynamic_plugin(capability)
        #c.load_plugin(DynamicModelPlugin(capability, c))

    for capability in c.read("~disable_capabilities", []):
        del my_plugins[capability]

    # Process planning pipeline
    # https://github.com/ros-planning/moveit/blob/4db626d9c3b5d6296b012188a4a0bfe4bddf6bce/moveit_ros/move_group/src/move_group.cpp#L142
    for capability in my_plugins.values():
        c.load_plugin(capability)


class MoveGroupCartesianPathService(ModelPlugin):
    def load(self, interpreter: 'Interpreter', c: NodeContext) -> None:
        c.provide(CARTESIAN_PATH_SERVICE_NAME, "moveit_msgs/GetCartesianPath")


class MoveGroupKinematicsService(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.provide(FK_SERVICE_NAME, "moveit_msgs/GetPositionFK")
        context.provide(IK_SERVICE_NAME, "moveit_msgs/GetPositionIK")


class MoveGroupExecuteTrajectoryAction(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.action_server(EXECUTE_ACTION_NAME, "moveit_msgs/ExecuteTrajectoryAction")


class MoveGroupMoveAction(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.action_server(MOVE_ACTION, "moveit_msgs/MoveGroupAction")


class MoveGroupPickPlaceAction(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.action_server(PICKUP_ACTION, "moveit_msgs/PickupAction")
        context.action_server(PLACE_ACTION, "moveit_msgs/PlaceAction")


class MoveGroupPlanService(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.provide(PLANNER_SERVICE_NAME, "moveit_msgs/GetMotionPlan")


class MoveGroupQueryPlannersService(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.provide(QUERY_PLANNERS_SERVICE_NAME, "moveit_msgs/QueryPlannerInterfaces")
        context.provide(GET_PLANNER_PARAMS_SERVICE_NAME, "moveit_msgs/GetPlannerParams")
        context.provide(SET_PLANNER_PARAMS_SERVICE_NAME, "moveit_msgs/SetPlannerParams")


class MoveGroupStateValidationService(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.provide(STATE_VALIDITY_SERVICE_NAME, "moveit_msgs/GetStateValidity")


class MoveGroupGetPlanningSceneService(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.provide(GET_PLANNING_SCENE_SERVICE_NAME, "moveit_msgs/GetPlanningScene")


class ApplyPlanningSceneService(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.provide(APPLY_PLANNING_SCENE_SERVICE_NAME, "moveit_msgs/ApplyPlanningScene")


class ClearOctomapService(ModelPlugin):
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        context.provide(CLEAR_OCTOMAP_SERVICE_NAME, "std_srvs/Empty")
