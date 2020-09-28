__all__ = ('Nav2Plugin',)

from typing import Mapping, Type
import abc

import attr
from loguru import logger
from roswire.name import namespace_join

from ...interpreter import Interpreter, ModelPlugin, NodeContext


class Nav2Plugin(ModelPlugin):

    @classmethod
    def from_dict(cls, dict_: Mapping[str, str], node_name: str) -> 'Nav2Plugin':
        cpp_class = dict_['type']
        plugin_name = dict_['name']
        logger.debug(f'loading navigation2 plugin [{plugin_name}] from file [{cpp_class}]')
        cpp_to_class: Mapping[str, Type[Nav2Plugin]] = {
            'nav2_controller::SimpleProgressChecker': SimpleProgressChecker,
            'nav2_controller::SimpleGoalChecker': SimpleGoalChecker,
            'nav2_controller::StoppedGoalChecker': StoppedGoalChecker,
            'dwb_core::DWBLocalPlanner': DWBLocalPlanner
        }

        cls = cpp_to_class[cpp_class]
        plugin = cls.build(plugin_name, node_name)
        logger.debug(f'loaded navigation2 plugin [{plugin_name}] from file [{cpp_class}]')
        return plugin

    @classmethod
    def build(cls, plugin_name: str, node_name: str) -> 'Nav2Plugin':
        ...


@attr.s(frozen=True, slots=True)
class SimpleProgressChecker(Nav2Plugin):

    class_name = 'nav2_controller::SimpleProgressChecker'
    name: str = attr.ib()
    node_name: str = attr.ib()

    def load(self, interpreter: Interpreter, c: NodeContext) -> None:
        c.read(f'~{self.name}.required_movement_radius', 0.5)
        c.read(f'~{self.name}.movement_time_allowance', 10.0)

    @classmethod
    def build(cls, plugin_name: str, node_name: str) -> 'Nav2Plugin':
        return SimpleProgressChecker(name=plugin_name, node_name=node_name)


@attr.s(frozen=True, slots=True)
class SimpleGoalChecker(Nav2Plugin):

    class_name = 'nav2_controller::SimpleGoalChecker'
    name: str = attr.ib()
    node_name: str = attr.ib()

    def load(self, interpreter: Interpreter, c: NodeContext) -> None:
        c.read(f'{self.name}.xy_goal_tolerance', 0.25)
        c.read(f'{self.name}.yaw_goal_tolerance', 0.25)
        c.read(f'{self.name}.stateful', True)

    @classmethod
    def build(cls, plugin_name: str, node_name: str) -> 'Nav2Plugin':
        return SimpleGoalChecker(name=plugin_name, node_name=node_name)


@attr.s(frozen=True, slots=True)
class StoppedGoalChecker(Nav2Plugin):

    class_name = 'nav2_controller::StoppedGoalChecker'
    name: str = attr.ib()
    node_name: str = attr.ib()

    def load(self, interpreter: 'Interpreter', c: 'NodeContext') -> None:
        c.read(f'~{self.name}.rot_stopped_velocity', 0.25)
        c.read(f'~{self.name}.trans_stopped_velocity', 0.25)

    @classmethod
    def build(cls, plugin_name: str, node_name: str) -> 'Nav2Plugin':
        return StoppedGoalChecker(name=plugin_name, node_name=node_name)


@attr.s(frozen=True, slots=True)
class DWBLocalPlanner(Nav2Plugin):

    class_name = 'dwb_core::DWBLocalPlanner'

    name: str = attr.ib()
    node_name: str = attr.ib()

    def load(self, interpreter: 'Interpreter', c: 'NodeContext') -> None:
        c.read(f'{self.name}.critics')
        c.read(f'{self.name}.default_critic_namespaces')
        c.read(f'{self.name}.prune_plan', True)
        c.read(f'{self.name}.prune_distance', 1.0)
        c.read(f'{self.name}.debug_trajectory_details', False)
        c.read(f'{self.name}.trajectory_generator_name', "dwb_plugins::StandardTrajectoryGenerator")
        c.read(f'{self.name}.transform_tolerance', 0.1)
        c.read(f'{self.name}.short_circuit_trajectory_evaluation', True)

        # From DWBPublisher
        c.read(f'{self.name}.publish_evaluation', True)
        c.read(f'{self.name}.publish_global_plan', True)
        c.read(f'{self.name}.publish_transformed_plan', True)
        c.read(f'{self.name}.publish_local_plan', True)
        c.read(f'{self.name}.publish_cost_grid_pc', False)
        c.read(f'{self.name}.marker_lifetime', 0.1)

        c.pub("evaluation", 'dwb_msgs/msg/LocalPlanEvaluation')
        c.pub("received_global_plan", 'nav_msgs/msg/Path')
        c.pub('transformed_global_plan', 'nav_msgs/msg/Path')
        c.pub('local_plan', 'nav_msgs/msg/Path')
        c.pub('marker', 'visualization_msgs/msg/MarkerArray')
        c.pub('cost_cloud', 'sensor_msgs/msg/PointCloud')

