# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = ("ControllerManagerPlugin",)

import abc
import typing as t

import attr
from loguru import logger
from roswire.name import namespace_join
from ...interpreter import Interpreter, ModelPlugin, NodeContext


@attr.s(frozen=True, slots=True)
class ControllerManagerPlugin(ModelPlugin, abc.ABC):
    type_name: t.ClassVar[str]

    controller_name: str = attr.ib()
    controller_manager_node: str = attr.ib()

    @classmethod
    def from_type(
        cls,
        controller_type: str,
        controller_name: str,
        controller_manager_node: str,
    ) -> ControllerManagerPlugin:
        """Loads a controller manager plugin.

        Parameters
        ----------
        controller_type: str
            The type of the controller
        controller_name: str
            The name of the controller
        controller_manager_node: str
            The fully qualified name of the node that hosts the associated control manager
        """
        logger.debug(
            f"loading controller_manager plugin [{controller_name}] "
            f"with type [{controller_type}] "
            f"via node [{controller_manager_node}]"
        )

        # find the class associated with this type
        # TODO iterate over subclasses and inspect type_name
        type_to_class: t.Mapping[str, t.Type[ControllerManagerPlugin]] = {
            'joint_state_controller/JointStateController': JointStateControllerPlugin,
            'diff_drive_controller/DiffDriveController': DiffDriveControllerPlugin,
            'effort_controllers/JointEffortController': JointEffortController,
            "effort_controllers/JointPositionController": JointPositionController,
        }
        cls = type_to_class[controller_type]

        plugin = cls.build(controller_name, controller_manager_node)
        logger.debug(f'loaded controller_manager plugin [{controller_name}]')
        return plugin

    @classmethod
    def build(cls, controller_name: str, controller_manager_node: str) -> ControllerManagerPlugin:
        return cls(
            controller_name=controller_name,
            controller_manager_node=controller_manager_node,
        )

    @property
    def namespace(self) -> str:
        """The associated namespace for this controller."""
        return namespace_join(self.controller_manager_node, self.controller_name)

    def load(self, interpreter: Interpreter) -> None:
        context = interpreter.nodes[self.controller_manager_node]
        self._load(interpreter, context)

    @abc.abstractmethod
    def _load(self, interpreter: Interpreter, context: NodeContext) -> None:
        """Simulates the architectural effects of this controller.

        Parameters
        ----------
        interpreter: Interpreter
            provides access to the simulated architecture for the entire ROS system
        context: NodeContext
            provides access to the node context for the associated controller_manager node
            that hosts this controller
        """
        ...


@attr.s(auto_attribs=True, frozen=True, slots=True)
class JointStateControllerPlugin(ControllerManagerPlugin):
    def _load(self, interpreter: Interpreter, context: NodeContext) -> None:
        context.read("joints", None)
        context.read("publish_rate")
        context.read("extra_joints")
        context.pub("joint_states", "sensor_msgs::JointState")


@attr.s(auto_attribs=True, frozen=True, slots=True)
class DiffDriveControllerPlugin(ControllerManagerPlugin):
    def _load(self, interpreter: Interpreter, context: NodeContext) -> None:
        # FIXME this is only a partial model!
        # https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/diff_drive_controller.cpp

        # TODO get wheel names

        context.read("publish_rate", 50.0)
        context.read("open_loop", False)
        context.read("wheel_separation_multiplier", 1.0)

        if context.has_param("wheel_radius_multiplier"):
            context.read("wheel_radius_multiplier")
        else:
            context.read("left_wheel_radius_multiplier", 1.0)
            context.read("right_wheel_radius_multiplier", 1.0)

        context.read("velocity_rolling_window_size", 10)
        context.read("cmd_vel_timeout", 0.5)
        context.read("allow_multiple_cmd_vel_publishers", True)

        context.read("base_frame_id", "base_link")
        context.read("odom_frame_id", "odom")
        context.read("enable_odom_tf", True)

        context.read("linear/x/has_velocity_limits", False)
        context.read("linear/x/has_acceleration_limits", False)
        context.read("linear/x/has_jerk_limits", False)
        context.read("linear/x/max_velocity", 0.0)
        context.read("linear/x/min_velocity", 0.0)
        context.read("linear/x/max_acceleration", 0.0)
        context.read("linear/x/min_acceleration", 0.0)
        context.read("linear/x/max_jerk", 0.0)
        context.read("linear/x/min_jerk", 0.0)

        context.read("angular/z/has_velocity_limits", False)
        context.read("angular/z/has_acceleration_limits", False)
        context.read("angular/z/has_jerk_limits", False)
        context.read("angular/z/max_velocity", 0.0)
        context.read("angular/z/min_velocity", 0.0)
        context.read("angular/z/max_acceleration", 0.0)
        context.read("angular/z/min_acceleration", 0.0)
        context.read("angular/z/max_jerk", 0.0)
        context.read("angular/z/min_jerk", 0.0)

        should_publish_command = context.read("publish_cmd", False)
        should_publish_joint_state = context.read("publish_wheel_joint_controller_state", False)

        context.read("wheel_separation", 0.0)
        context.read("wheel_radius", 0.0)

        context.sub("cmd_vel", "geometry_msgs/Twist")

        if should_publish_command:
            context.pub("cmd_vel_out", "geometry_msgs/TwistStamped")

        if should_publish_joint_state:
            context.pub("wheel_joint_controller_state", "control_msgs/JointTrajectoryControllerState")

        context.pub("odom", "nav_msgs/Odometry")
        context.pub("/tf", "tf/tfMessage")

        # TODO this node supports dynamic reconfiguration
        logger.warning("plugin model is likely incomplete: DiffDriveControllerPlugin")


@attr.s(auto_attribs=True, frozen=True, slots=True)
class ForwardCommandController(ControllerManagerPlugin):

    @property
    def namespace(self) -> str:
        return self.controller_name

    def _load(self, interpreter: Interpreter, context: NodeContext) -> None:
        ns = self.namespace
        # https://github.com/ros-controls/ros_controllers/blob/melodic-devel/forward_command_controller/include/forward_command_controller/forward_command_controller.h
        context.sub(f"{ns}/command", "std_msgs/Float64")


class JointEffortController(ForwardCommandController):
    def _load(self, interpreter: Interpreter, context: NodeContext) -> None:
        super()._load(interpreter, context)


@attr.s(auto_attribs=True, frozen=True, slots=True)
class JointPositionController(ControllerManagerPlugin):

    @property
    def namespace(self) -> str:
        return self.controller_name

    def _load(self, interpreter: Interpreter, context: NodeContext) -> None:
        ns = self.namespace
        # https://github.com/ros-controls/ros_controllers/blob/melodic-devel/forward_command_controller/include/forward_command_controller/forward_command_controller.h
        context.sub(f"{ns}/command", "std_msgs/Float64")
        context.pub(f"{ns}/state", "control_msgs/JointControllerStates")
