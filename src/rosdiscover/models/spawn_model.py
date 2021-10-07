# -*- coding: utf-8 -*-
import argparse
import xml.etree.ElementTree as ET  # noqa

from loguru import logger

from .plugins.gazebo import GazeboPlugin
from ..interpreter import model


@model('gazebo_ros', 'spawn_model')
def spawn_model(c):
    parser = argparse.ArgumentParser('spawn_model')
    parser.add_argument('-urdf', action='store_true')
    parser.add_argument('-model', type=str)
    parser.add_argument('-param', type=str)
    parser.add_argument('-unpause', action='store_true')
    parser.add_argument('-wait', action='store_true')
    parser.add_argument('-x', type=float)
    parser.add_argument('-y', type=float)
    parser.add_argument('-z', type=float)
    parser.add_argument('-R', type=float)
    parser.add_argument('-P', type=float)
    parser.add_argument('-Y', type=float)
    parser.add_argument('-J', type=float)
    parser.add_argument('-gazebo_namespace', default='/gazebo')
    args = parser.parse_args(c.args.split())

    ns_gz = args.gazebo_namespace
    c.sub(f'{ns_gz}/model_states', 'gazebo_msgs/ModelStates')
    c.use(f'{ns_gz}/unpause_physics', 'std_srvs/Empty')
    c.use(f'{ns_gz}/delete_model', 'gazebo_msgs/DeleteModel')
    c.use(f'{ns_gz}/spawn_urdf_model', 'gazebo_msgs/SpawnModel')
    c.use(f'{ns_gz}/spawn_sdf_model', 'gazebo_msgs/SpawnModel')
    c.use(f'{ns_gz}/set_model_configuration', 'gazebo_msgs/SetModelConfiguration')

    urdf_param_name = args.param
    logger.debug(f'spawning model using parameter [{urdf_param_name}]')
    urdf_contents = c.read(urdf_param_name).strip()
    logger.debug(f'parsing URDF model from parameter [{urdf_param_name}]:'
                 f'\n{urdf_contents}')

    with open('urdf.xml', 'w') as f:
        f.write(urdf_contents)

    #  Workaround to find the <robot> begin tag
    begin_tag = "<robot>"
    begin_tag_starts_at = urdf_contents.find(begin_tag)
    urdf_contents = urdf_contents[begin_tag_starts_at:]

    # #94 workaround to deal with inclusion of warning when xacro.py is used
    end_tag = '</robot>'
    end_tag_starts_at = urdf_contents.rfind(end_tag)
    end_tag_ends_at = end_tag_starts_at + len(end_tag)
    urdf_contents = urdf_contents[:end_tag_ends_at]
    logger.debug(urdf_param_name)
    logger.debug(urdf_contents)
    urdf_xml = ET.fromstring(urdf_contents)
    for plugin_xml in urdf_xml.findall('.//plugin'):
        try:
            plugin = GazeboPlugin.from_xml(plugin_xml)
            c.load_plugin(plugin)
        except ValueError:
            logger.exception(f"failed to load gazebo plugin [skipping!]: {plugin_xml}")
