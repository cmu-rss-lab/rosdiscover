# -*- coding: utf-8 -*-
__all__ = ("spawn_model",)

import argparse
import os
import typing as t
import xml.etree.ElementTree as ET  # noqa

from loguru import logger

from .plugins.gazebo import GazeboPlugin
from ..interpreter import model


def _repair_urdf_contents(contents: str) -> str:
    #  Workaround to find the <robot> begin tag
    begin_tag = "<robot>"
    begin_tag_starts_at = contents.find(begin_tag)
    if begin_tag_starts_at == -1:
        begin_tag_starts_at = contents.find("<robot ")
    if begin_tag_starts_at != -1:
        contents = contents[begin_tag_starts_at:]

    # #94 workaround to deal with inclusion of warning when xacro.py is used
    end_tag = '</robot>'
    end_tag_starts_at = contents.rfind(end_tag)
    end_tag_ends_at = end_tag_starts_at + len(end_tag)
    contents = contents[:end_tag_ends_at]
    logger.debug(f"New contents are:\n{contents}")
    return contents


def _load_urdf_xml_from_parameter(c, parameter_name: str) -> t.Optional[ET.Element]:
    logger.debug(f'spawning model using parameter [{parameter_name}]')
    urdf_contents = c.read(parameter_name).strip()
    logger.debug(f'parsing URDF model from parameter [{parameter_name}]:'
                 f'\n{urdf_contents}')
    urdf_contents = _repair_urdf_contents(urdf_contents)
    return ET.fromstring(urdf_contents)


def _load_urdf_xml_from_file(c, filename: str) -> t.Optional[ET.Element]:
    if not os.path.isabs(filename):
        logger.error(f"unable to load URDF XML from file [{filename}]: expected absolute path")
        return None

    logger.debug(f"loading URDF XML from file [{filename}]")
    urdf_contents = c.read_file(filename)
    logger.debug(f"loaded URDF XML contents from file [{filename}]:\n{urdf_contents}")
    urdf_contents = _repair_urdf_contents(urdf_contents)
    logger.debug("parsing URDF XML contents")
    xml = ET.fromstring(urdf_contents)
    logger.debug("parsed UDF XML contents")
    return xml


@model('gazebo_ros', 'spawn_model')
def spawn_model(c):
    parser = argparse.ArgumentParser('spawn_model')
    parser.add_argument('-urdf', action='store_true')
    parser.add_argument('-model', type=str)
    parser.add_argument('-file', type=str)
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

    # load the URDF file into an XML object
    urdf_xml: t.Optional[ET.Element] = None
    if args.file:
        urdf_xml = _load_urdf_xml_from_file(c, args.file)
    elif args.param:
        urdf_xml = _load_urdf_xml_from_parameter(c, args.param)

    if not urdf_xml:
        logger.error("failed to load URDF XML")
        return

    for plugin_xml in urdf_xml.findall('.//plugin'):
        try:
            plugin = GazeboPlugin.from_xml(plugin_xml)
            c.load_plugin(plugin)
        except ValueError:
            logger.exception(f"failed to load gazebo plugin [skipping!]: {plugin_xml}")
