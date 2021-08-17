# -*- coding: utf-8 -*-
__all__ = (
    "DeleteParam",
    "HasParam",
    "Publisher",
    "ReadParam",
    "ReadParamWithDefault",
    "RosInit",
    "ServiceCaller",
    "ServiceProvider",
    "Subscriber",
    "SymbolicRosApiCall",
    "WriteParam",
)

import abc

import attr

from .symbolic import (
    SymbolicBool,
    SymbolicStatement,
    SymbolicString,
    SymbolicValue,
)


class SymbolicRosApiCall(SymbolicStatement, abc.ABC):
    """Represents a symbolic call to a ROS API."""


@attr.s(frozen=True, auto_attribs=True, slots=True)
class RosInit(SymbolicRosApiCall):
    name: SymbolicString


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Publisher(SymbolicRosApiCall):
    topic: SymbolicString
    format_: str


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Subscriber(SymbolicRosApiCall):
    topic: SymbolicString
    format_: str


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ServiceProvider(SymbolicRosApiCall):
    service: SymbolicString
    format_: str


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ServiceCaller(SymbolicRosApiCall):
    service: SymbolicValue
    format_: str


@attr.s(frozen=True, auto_attribs=True, slots=True)
class WriteParam(SymbolicRosApiCall):
    param: SymbolicString
    value: SymbolicValue


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ReadParam(SymbolicRosApiCall, SymbolicValue):
    param: SymbolicString
    value: SymbolicValue


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ReadParamWithDefault(SymbolicRosApiCall, SymbolicValue):
    param: SymbolicString
    value: SymbolicValue
    default: SymbolicValue


@attr.s(frozen=True, auto_attribs=True, slots=True)
class HasParam(SymbolicRosApiCall, SymbolicBool):
    param: SymbolicString


@attr.s(frozen=True, auto_attribs=True, slots=True)
class DeleteParam(SymbolicRosApiCall):
    param: SymbolicString
