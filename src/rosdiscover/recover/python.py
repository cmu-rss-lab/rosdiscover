# -*- coding: utf-8 -*-
__all__ = ('PythonModelExtractor',)

from typing import Iterator

from loguru import logger
from comby import Comby
import attr

from .core import PublisherDefinition, RecoveredNodeModel

_PUBLISHER_TEMPLATE = \
    'rospy.Publisher(:[topic_expr], :[type], queue_size=:[queue_size])'


@attr.s(slots=True, frozen=True, auto_attribs=True)
class PythonModelExtractor:
    """Extracts architectural models from Python source code."""
    _source: str
    _comby: Comby = attr.ib(factory=Comby)

    def __attrs_post_init__(self) -> None:
        logger.debug(f"extracting model from Python source:\n{self._source}")

    def extract(self) -> RecoveredNodeModel:
        publishers = list(self._publishers)
        raise NotImplementedError

    @property
    def _publishers(self) -> Iterator[PublisherDefinition]:
        logger.debug('extracting publishers')
        comby = self._comby
        template = _PUBLISHER_TEMPLATE
        for match in comby.matches(self._source, template, language='.py'):
            type_ = match['type_']
            topic = match['topic_expr']
            queue_size = int(match['queue_size'])
            yield PublisherDefinition(type_=type_,
                                      topic=topic,
                                      queue_size=queue_size)
