# -*- coding: utf-8 -*-
import argparse
import xml.etree.ElementTree as ET  # noqa

from loguru import logger

from .plugins.gazebo import GazeboPlugin
from ..interpreter import model
import re


@model('gazebo_ros', 'spawn_model')
def spawn_model(c):
    ns_gz = '/gazebo'
    c.sub(f'{ns_gz}/model_states', 'gazebo_msgs/ModelStates')
    c.use(f'{ns_gz}/unpause_physics', 'std_srvs/Empty')
    c.use(f'{ns_gz}/delete_model', 'gazebo_msgs/DeleteModel')
    c.use(f'{ns_gz}/spawn_urdf_model_client', 'gazebo_msgs/SpawnModel')
    c.use(f'{ns_gz}/spawn_sdf_model_client', 'gazebo_msgs/SpawnModel')
    c.use(f'{ns_gz}/spawn_sdf_model_client', 'gazebo_msgs/SpawnModel')
    c.use(f'{ns_gz}/set_model_configuration', 'gazebo_msgs/SetModelConfiguration')

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
    args = parser.parse_args(c.args.split())

    urdf_param_name = args.param
    logger.debug(f'spawning model using parameter [{urdf_param_name}]')
    urdf_contents = c.read(urdf_param_name).strip()
    logger.debug(f'parsing URDF model from parameter [{urdf_param_name}]:'
                 f'\n{urdf_contents}')
    with open('urdf.xml', 'w') as f:
        f.write(urdf_contents)
    # Non XML stuff can appear at the end, so strip it out by finding the position of the last
    # end tag and then stripping everything after that tag
    end_tags_end_position = [i.end() for i in re.finditer(r'</.*>', urdf_contents)]
    last_end_tag_position = end_tags_end_position[-1]
    urdf_contents = urdf_contents[:last_end_tag_position]
    urdf_xml = ET.fromstring(urdf_contents)
    for plugin_xml in urdf_xml.findall('.//plugin'):
        plugin = GazeboPlugin.from_xml(plugin_xml)
        c.load_plugin(plugin)
