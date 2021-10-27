# -*- coding: utf-8 -*-
import argparse
import os
import typing as t

import yaml
import attr
from loguru import logger
from roswire.name import namespace_join

from .plugins.controller_manager import ControllerManagerPlugin
from ..interpreter import model, NodeContext


@attr.s(frozen=True, slots=True, auto_attribs=True)
class _Controller:
    name: str = attr.ib()
    type_: str = attr.ib()


@model('controller_manager', 'spawner')
def spawner(c: NodeContext):
    parser = argparse.ArgumentParser("spawner")
    # FIXME this is not quite how this argument works:
    # https://github.com/ros-controls/ros_control/blob/5db3baaa71c9dcd8a2fadb3b2c0b5085ea49b3a1/controller_manager/scripts/spawner#L174-L185
    parser.add_argument("--namespace", default="/")
    parser.add_argument("controllers", nargs="+")
    args = parser.parse_args(c.args.split())

    controllers_to_spawn: t.List[_Controller] = []
    controller_manager_namespace = "/"

    for controller_name_or_filename in args.controllers:
        if os.path.isabs(controller_name_or_filename):
            try:
                contents = c.app.files.read(controller_name_or_filename)
                controllers_yml = yaml.safe_load(contents)
                for name, info in controllers_yml.items():
                    controller_type = info['type']
                    controllers_to_spawn.append(_Controller(name=name, type_=controller_type))
            except Exception:
                m = f"Error reading controllers from a config file: {controller_name_or_filename}"
                logger.error(m)
                raise

        else:
            controller_name = controller_name_or_filename
            logger.debug(f"finding type for controller [{controller_name}]")
            controller_namespace = namespace_join(controller_manager_namespace, controller_name)
            controller_type = c.read(f"{controller_namespace}/type")
            logger.debug(f"found type for controller [{controller_name}]: {controller_type}")
            controller = _Controller(name=controller_name, type_=controller_type)
            controllers_to_spawn.append(controller)

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

    for controller in controllers_to_spawn:
        # This used to be ControllerManagerPlugin.from_type
        plugin = ControllerManagerPlugin.from_type(
            controller_name=controller.name,
            controller_type=controller.type_,
            controller_manager_node=controller_manager_node,
        )
        c.load_plugin(plugin)
