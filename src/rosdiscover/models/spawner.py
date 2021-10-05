# -*- coding: utf-8 -*-
import argparse
import os
import typing as t

from loguru import logger
from roswire.name import namespace_join

from .plugins.controller_manager import ControllerManagerPlugin
from ..interpreter import model


@model('controller_manager', 'spawner')
def spawner(c):
    parser = argparse.ArgumentParser("spawner")
    # FIXME this is not quite how this argument works:
    # https://github.com/ros-controls/ros_control/blob/5db3baaa71c9dcd8a2fadb3b2c0b5085ea49b3a1/controller_manager/scripts/spawner#L174-L185
    parser.add_argument("--namespace", default="/")
    parser.add_argument("controllers", nargs="+")
    args = parser.parse_args(c.args.split())

    controllers_to_spawn: t.List[str] = []
    for controller_name_or_filename in args.controllers:
        if os.path.isabs(controller_name_or_filename):
            m = "no support for spawners that load their controllers from a config file"
            raise NotImplementedError(m)
        else:
            controllers_to_spawn.append(controller_name_or_filename)

    # the spawner interacts with the controller_manager services
    load_controller_service = namespace_join(args.namespace, "controller_manager/load_controller")
    unload_controller_service = namespace_join(args.namespace, "controller_manager/unload_controller")
    switch_controller_service = namespace_join(args.namespace, "controller_manager/switch_controller")

    c.use(load_controller_service, "controller_manager_msgs/LoadController")
    c.use(unload_controller_service, "controller_manager_msgs/UnloadController")
    c.use(switch_controller_service, "controller_manager_msgs/SwitchController")

    # FIXME load each controller as a plugin for the associated controller_manager
    # this is a little tricky since we need to find the correct controller_manager
    # going forward, we can (after solving some ordering/dependency issues) iterate
    # over the system to find the node that provides the necessary services
    #
    # for now, I'm hardcoding gazebo so that we crack on with our experiments
    controller_manager_node = "/gazebo"
    controller_manager_namespace = "/"

    for controller_name in controllers_to_spawn:
        logger.debug(f"finding type for controller [{controller_name}]")
        controller_namespace = namespace_join(controller_manager_namespace, controller_name)
        controller_type = c.read(f"{controller_namespace}/type")
        logger.debug(f"found type for controller [{controller_name}]: {controller_type}")

        plugin = ControllerManagerPlugin.build(
            controller_name=controller_name,
            controller_type=controller_type,
            controller_manager_node=controller_manager_node,
        )
        c.load_plugin(plugin)
