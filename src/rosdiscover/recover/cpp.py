# -*- coding: utf-8 -*-
__all__ = ('CppModelExtractor',)

from typing import Iterator

from comby import Comby
import attr


@attr.s(slots=True, frozen=True, auto_attribs=True)
class CppModelExtractor:
    """Extracts architectural models from C++ source code."""
    _source: str
    _comby: Comby = attr.ib(factory=Comby)

    def extract_publishers(self, source: str) -> Iterator[PublisherDefinition]:
        for match in self._comby.matches(source, _ADVERTISE, language='.cpp'):
            type_ = match['type_']
            topic = match['topic']
            queue_size = int(match['queue_size'])
            yield PublisherDefinition(type_=type_,
                                      topic=topic,
                                      queue_size=queue_size)
