# -*- coding: utf-8 -*-
"""
This file provides model plugins that represent various Gazebo plugins.
"""
__all__ = ('GazeboPlugin',)

import xml.etree.ElementTree as ET

from loguru import logger

from ..interpreter.plugin import ModelPlugin as _ModelPlugin


class GazeboPlugin(_ModelPlugin):
    """Represents the architectural effects of a Gazebo plugin."""
    @classmethod
    def from_xml_element(cls, xml: ET.Element) -> 'GazeboPlugin':
        name = xml.attrib['name']
        filename = xml.attrib['filename']
        logger.debug(f'loading gazebo plugin [{name}] from file [{filename}]')
        raise NotImplementedError
