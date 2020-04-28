# -*- coding: utf-8 -*-
__all__ = ('PythonModelExtractor',)

from typing import Iterator

from comby import Comby
import attr

from .core import PublisherDefinition


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
