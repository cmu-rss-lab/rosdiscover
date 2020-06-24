# -*- coding: utf-8 -*-
from ..interpreter import model

import argparse
import xml.etree.ElementTree as ET

from loguru import logger


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

    urdf_xml = ET.fromstring(urdf_contents)

    for plugin_xml in urdf_xml.findall('.//plugin'):
        plugin_name = plugin_xml.attrib['name']
        plugin_filename = plugin_xml.attrib['filename']
        logger.debug(f'loading gazebo plugin [{plugin_name}] '
                     f'from file [{plugin_filename}]')

    c.load_gazebo_plugin(plugin_xml)
