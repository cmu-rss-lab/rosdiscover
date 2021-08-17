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
import typing as t

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

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "ros-init",
            "name": self.name,
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Publisher(SymbolicRosApiCall):
    topic: SymbolicString
    format_: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "publishes-to",
            "name": self.topic,
            "format": self.format_,
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Subscriber(SymbolicRosApiCall):
    topic: SymbolicString
    format_: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "subscribes-to",
            "name": self.topic,
            "format": self.format_,
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ServiceProvider(SymbolicRosApiCall):
    service: SymbolicString
    format_: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "provides-service",
            "name": self.service,
            "format": self.format_,
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ServiceCaller(SymbolicRosApiCall):
    service: SymbolicValue
    format_: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "calls-service",
            "name": self.service,
            "format": self.format_,
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class WriteParam(SymbolicRosApiCall):
    param: SymbolicString
    value: SymbolicValue

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "writes-to-param",
            "param": self.param.to_dict(),
            "value": self.value.to_dict(),
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ReadParam(SymbolicRosApiCall, SymbolicValue):
    param: SymbolicString
    value: SymbolicValue

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "reads-param",
            "param": self.param.to_dict(),
            "value": self.value.to_dict(),
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ReadParamWithDefault(SymbolicRosApiCall, SymbolicValue):
    param: SymbolicString
    value: SymbolicValue
    default: SymbolicValue

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "reads-param-with-default",
            "param": self.param.to_dict(),
            "value": self.value.to_dict(),
            "default": self.default.to_dict(),
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class HasParam(SymbolicRosApiCall, SymbolicBool):
    param: SymbolicString

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "checks-for-param",
            "param": self.param.to_dict(),
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class DeleteParam(SymbolicRosApiCall):
    param: SymbolicString

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "deletes-param",
            "param": self.param.to_dict(),
        }
