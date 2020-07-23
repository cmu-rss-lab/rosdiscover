# -*- coding: utf-8 -*-
"""
This module provides several data structures that are used to describe
architectural models that have been recovered from nodes.
"""
__all__ = ('PublisherDefinition',
           'ParameterDelete',
           'ParameterExistence',
           'ParameterRead',
           'ParameterSearch',
           'ParameterWrite',
           'RecoveredNodeModel',
           'RecoveredNodeModelElement',
           'SimpleActionClientDefinition',
           'SubscriberDefinition')

from typing import Any, Collection, Optional
import abc

import attr
import sourcelocation


class RecoveredNodeModelElement(abc.ABC):
    """Represents an element of a recovered node model.

    Attributes
    ----------
    source: str
        The associated source code for this element.
    location: sourcelocation.FileLocationRange
        The location of the source code for the element.
    """
    @abc.abstractmethod
    @property
    def source(self) -> str:
        ...

    @abc.abstractmethod
    @property
    def location(self) -> sourcelocation.FileLocationRange:
        ...


@attr.s(slots=True, frozen=True, auto_attribs=True)
class SimpleActionClientDefinition(RecoveredNodeModelElement):
    """Describes the creation of a simple action client.

    Attributes
    ----------
    type_: str
        The name of the type used by the action server.
    server: str
        The name of the action server.
    """
    source: str
    location: sourcelocation.FileLocationRange
    type_: str
    server: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class PublisherDefinition(RecoveredNodeModelElement):
    """Describes the creation of a publisher.

    Attributes
    ----------
    type_: str
        The name of the type used by the publisher.
    topic: str
        The name of the topic to which messages are published.
    queue_size: int
        The queue size for the publisher.
    """
    source: str
    location: sourcelocation.FileLocationRange
    type_: str
    topic: str
    queue_size: int


@attr.s(slots=True, frozen=True, auto_attribs=True)
class SubscriberDefinition(RecoveredNodeModelElement):
    """Describes the creation of a subscriber.

    Attributes
    ----------
    type_: str
        The name of the type used by the publisher.
    topic: str
        The name of the topic to which messages are published.
    """
    source: str
    location: sourcelocation.FileLocationRange
    type_: str
    topic: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class ParameterDelete(RecoveredNodeModelElement):
    """Describes a parameter delete operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    """
    source: str
    location: sourcelocation.FileLocationRange
    parameter_name: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class ParameterExistence(RecoveredNodeModelElement):
    """Describes a parameter existence operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    """
    source: str
    location: sourcelocation.FileLocationRange
    parameter_name: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class ParameterRead(RecoveredNodeModelElement):
    """Describes a parameter read operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    default_value: Optional[Any]
        The default value of the parameter, if any.
    """
    source: str
    location: sourcelocation.FileLocationRange
    parameter_name: str
    default_value: Optional[Any]


@attr.s(slots=True, frozen=True, auto_attribs=True)
class ParameterSearch(RecoveredNodeModelElement):
    """Describes a parameter search operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    """
    source: str
    location: sourcelocation.FileLocationRange
    parameter_name: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class ParameterWrite(RecoveredNodeModelElement):
    """Describes a parameter write operation.

    Attributes
    ----------
    parameter_name: str
        The name of the parameter.
    value_expression: str
        The expression used to obtain the parameter value.
    """
    source: str
    location: sourcelocation.FileLocationRange
    parameter_name: str
    value_expression: str


@attr.s(slots=True, frozen=True, auto_attribs=True)
class RecoveredNodeModel:
    publishers: Collection[PublisherDefinition]
    parameter_deletes: Collection[ParameterDelete]
    parameter_existences: Collection[ParameterExistence]
    parameter_reads: Collection[ParameterRead]
    parameter_writes: Collection[ParameterWrite]
    simple_action_clients: Collection[SimpleActionClientDefinition]
    subscribers: Collection[SubscriberDefinition]

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, 'publishers', tuple(self.publishers))
        object.__setattr__(self, 'parameter_deletes',
                           tuple(self.parameter_deletes))
        object.__setattr__(self, 'parameter_existences',
                           tuple(self.parameter_existences))
        object.__setattr__(self, 'parameter_reads', tuple(self.parameter_reads))
        object.__setattr__(self, 'parameter_writes', tuple(self.parameter_writes))
        object.__setattr__(self, 'simple_action_clients',
                           tuple(self.simple_action_clients))
        object.__setattr__(self, 'subscribers', tuple(self.subscribers))
