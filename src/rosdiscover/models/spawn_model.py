# -*- coding: utf-8 -*-
from ..interpreter import model

import argparse

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

    parser = argparse.ArgumentParser()
    parser.add_argument('-urdf', action='store_true')
    parser.add_argument('-model', type=str)
    parser.add_argument('-param', type=str)
    parser.add_argument('-x', type=float)
    parser.add_argument('-y', type=float)
    parser.add_argument('-z', type=float)
    args = parser.parse_args(c.args.split())

    urdf_param_name = args.param
    logger.debug(f'spawning model using parameter [{urdf_param_name}]')
    urdf_contents = c.read(urdf_param_name)
    logger.debug(f'model contents: {urdf_contents}')
