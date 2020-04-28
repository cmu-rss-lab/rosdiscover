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

import attr
import comby

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


@attr.s(slots=True, frozen=True)
class CppModelExtractor:
    """Extracts architectural models from C++ source code."""
    _comby: Comby = attr.ib(factory=Comby)

    def extract_publishers(source: str) -> Iterator[PublisherDefinition]:
        for match in self._comby.matches(source, _ADVERTISE, language='.cpp'):
            type_ = match['type_']
            topic = match['topic']
            queue_size = int(match['queue_size'])
            yield PublisherDefinition(type_=type_,
                                      topic=topic,
                                      queue_size=queue_size)
