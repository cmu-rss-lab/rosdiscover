# -*- coding: utf-8 -*-
"""
This module is used to provide partial architecture recovery capabilities.
That is, it can be used to produce a (partial) model for a given node by
performing a simple static analysis of its associated source code. This code
operates under fairly heavy restrictions and is most successful when applied
to simple nodes with static models (rather than nodes that may publish and
subscribe to different topics at run-time depending on their configuration).
"""
from typing import Iterator
import contextlib

from comby import Comby
from loguru import logger
from roswire import ROSWire
import attr
import roswire as _roswire

from ..config import Config

_ADVERTISE = ":[[nh]].advertise<:[type]>(\":[topic]\", :[queue_size]);"


@attr.s(slots=True, frozen=True, auto_attribs=True)
class PublisherDefinition:
    """Describes a topic publish call.

    Attributes
    ----------
    type_: str
        The name of the type used by the publisher.
    topic: str
        The name of the topic to which messages are published.
    queue_size: int
        The queue size for the publisher.
    """
    # location: comby.LocationRange
    type_: str
    topic: str
    queue_size: int


@attr.s(slots=True, frozen=True, auto_attribs=True)
class PythonModelExtractor:
    """Extracts architectural models from Python source code."""
    _source: str
    _comby: Comby = attr.ib(factory=Comby)

    @property
    def publishers(self) -> Iterator[PublisherDefinition]:
        for match in self._comby.matches(source, _ADVERTISE, language='.cpp'):
            type_ = match['type_']
            topic = match['topic']
            queue_size = int(match['queue_size'])
            yield PublisherDefinition(type_=type_,
                                      topic=topic,
                                      queue_size=queue_size)


@attr.s(slots=True, frozen=True, auto_attribs=True)
class RecoveryTool:
    _config: Config
    _system: _roswire.System = attr.ib(repr=False)

    @classmethod
    @contextlib.contextmanager
    def for_config(cls, config: Config) -> Iterator['RecoveryTool']:
        roswire = ROSWire()
        with roswire.launch(config.image, config.sources) as app:
            yield cls.for_app_instance(config, app)

    @classmethod
    def for_app_instance(cls,
                         config: Config,
                         app: _roswire.System) -> 'RecoveryTool':
        return RecoveryTool(config=config, system=app)

    def recover_node(self, package: str, node: str) -> None:
        logger.debug(f'recovering model for node [{node}] '
                     f'in package [{package}]')
        # FIXME hardcoded
        filename = '/ros_ws/src/fetch_ros/fetch_navigation/scripts/tilt_head.py'
        raise NotImplementedError
