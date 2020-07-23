# -*- coding: utf-8 -*-
"""
This module provides several data structures that are used to describe
architectural models that have been recovered from nodes.
"""
__all__ = ('PublisherDefinition', 'RecoveredNodeModel')

from typing import Collection

import attr


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
    type_: str
    topic: str
    queue_size: int


@attr.s(slots=True, frozen=True, auto_attribs=True)
class RecoveredNodeModel:
    publishers: Collection[PublisherDefinition]

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, 'publishers', tuple(self.publishers))
