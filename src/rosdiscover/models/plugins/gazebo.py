# -*- coding: utf-8 -*-
"""
This file provides model plugins that represent various Gazebo plugins.
"""
__all__ = ('GazeboPlugin',)

from typing import Type
import abc
import xml.etree.ElementTree as ET

from loguru import logger
import attr

from ..interpreter.plugin import ModelPlugin as _ModelPlugin


class GazeboPlugin(_ModelPlugin):
    """Represents the architectural effects of a Gazebo plugin."""
    @classmethod
    def from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        name = xml.attrib['name']
        filename = xml.attrib['filename']
        logger.debug(f'loading gazebo plugin [{name}] from file [{filename}]')

        # TODO locate the class for the plugin based on filename
        cls: Type[GazeboPlugin]
        raise NotImplementedError

        return cls.build_from_xml(xml)

    @classmethod
    @abc.abstractmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        ...


@attr.s(frozen=True, slots=True)
class LibGazeboROSLaserPlugin(GazeboPlugin):
    """
    Example definition
    ------------------

    .. code:: xml

        <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_lds_lfcd_controller">
          <topicName>scan</topicName>
          <frameName>base_scan</frameName>
        </plugin>
    """
    filename = 'libgazebo_ros_laser.so'
    topic_name: str = attr.ib()
    frame_name: str = attr.ib()

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        raise NotImplementedError
