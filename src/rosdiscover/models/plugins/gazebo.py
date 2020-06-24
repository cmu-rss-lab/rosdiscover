# -*- coding: utf-8 -*-
"""
This file provides model plugins that represent various Gazebo plugins.
"""
__all__ = ('GazeboPlugin',)

from ..interpreter.plugin import ModelPlugin as _ModelPlugin


class GazeboPlugin(_ModelPlugin):
    """Represents the architectural effects of a Gazebo plugin."""
