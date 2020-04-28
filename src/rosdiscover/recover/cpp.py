# -*- coding: utf-8 -*-
__all__ = ('CppModelExtractor',)

from typing import Iterator

from comby import Comby
import attr

from .core import PublisherDefinition, RecoveredNodeModel


@attr.s(slots=True, frozen=True, auto_attribs=True)
class CppModelExtractor:
    """Extracts architectural models from C++ source code."""
    _source: str
    _comby: Comby = attr.ib(factory=Comby)

    def extract(self) -> RecoveredNodeModel:
        raise NotImplementedError

    @property
    def publishers(self) -> Iterator[PublisherDefinition]:
        comby = self._comby
        pattern = ":[[nh]].advertise<:[type]>(\":[topic]\", :[queue_size]);"
        for match in comby.matches(self._source, pattern, language='.cpp'):
            type_ = match['type_']
            topic = match['topic']
            queue_size = int(match['queue_size'])
            yield PublisherDefinition(type_=type_,
                                      topic=topic,
                                      queue_size=queue_size)
