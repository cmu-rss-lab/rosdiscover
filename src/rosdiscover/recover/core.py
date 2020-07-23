# -*- coding: utf-8 -*-
"""
This module provides several data structures that are used to describe
architectural models that have been recovered from nodes.
"""
__all__ = ('PublisherDefinition',
           'ParameterDelete',
           'ParameterRead',
           'ParameterExistence',
           'ParameterWrite',
           'RecoveredNodeModel')

from typing import Any, Collection, Optional

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
class ParameterRead:
    """Describes a parameter read operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    default_value: Optional[Any]
        The default value of the parameter, if any.
    """
    parameter_name: str
    default_value: Optional[Any]


@attr.s(slots=True, frozen=True, auto_attribs=True)
class ParameterDelete:
    """Describes a parameter delete operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    """
    parameter_name: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class ParameterExistence:
    """Describes a parameter existence operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    """
    parameter_name: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class ParameterWrite:
    """Describes a parameter write operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    value_expression: str
        The expression used to obtain the parameter value.
    """
    parameter_name: str
    value_expression: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class RecoveredNodeModel:
    publishers: Collection[PublisherDefinition]
    parameter_deletes: Collection[ParameterDelete]
    parameter_existences: Collection[ParameterExistence]
    parameter_reads: Collection[ParameterRead]
    parameter_writes: Collection[ParameterWrite]

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, 'publishers', tuple(self.publishers))
        object.__setattr__(self, 'parameter_deletes',
                           tuple(self.parameter_deletes))
        object.__setattr__(self, 'parameter_existences',
                           tuple(self.parameter_existences))
        object.__setattr__(self, 'parameter_reads', tuple(self.parameter_reads))
        object.__setattr__(self, 'parameter_writes', tuple(self.parameter_writes))
