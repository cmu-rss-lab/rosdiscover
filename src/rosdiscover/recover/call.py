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
    SymbolicContext,
    SymbolicStatement,
    SymbolicString,
    SymbolicValue,
)


class SymbolicRosApiCall(SymbolicStatement, abc.ABC):
    """Represents a symbolic call to a ROS API."""
    @abc.abstractmethod
    def is_unknown(self) -> bool:
        """Determines whether the target of this API call is unknown."""
        ...


@attr.s(frozen=True, auto_attribs=True, slots=True)
class RosInit(SymbolicRosApiCall):
    name: SymbolicString

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "ros-init",
            "name": self.name.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> None:
        return None

    def is_unknown(self) -> bool:
        return self.name.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Publisher(SymbolicRosApiCall):
    topic: SymbolicString
    format_: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "publishes-to",
            "name": self.topic.to_dict(),
            "format": self.format_,
        }

    def eval(self, context: SymbolicContext) -> None:
        topic = self.topic.eval(context)
        context.node.pub(topic, self.format_)

    def is_unknown(self) -> bool:
        return self.topic.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Subscriber(SymbolicRosApiCall):
    topic: SymbolicString
    format_: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "subscribes-to",
            "name": self.topic.to_dict(),
            "format": self.format_,
        }

    def eval(self, context: SymbolicContext) -> None:
        topic = self.topic.eval(context)
        context.node.sub(topic, self.format_)

    def is_unknown(self) -> bool:
        return self.topic.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ServiceProvider(SymbolicRosApiCall):
    service: SymbolicString
    format_: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "provides-service",
            "name": self.service.to_dict(),
            "format": self.format_,
        }

    def eval(self, context: SymbolicContext) -> None:
        service = self.service.eval(context)
        context.node.provide(service, self.format_)

    def is_unknown(self) -> bool:
        return self.service.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ServiceCaller(SymbolicRosApiCall):
    service: SymbolicValue
    format_: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "calls-service",
            "name": self.service.to_dict(),
            "format": self.format_,
        }

    def eval(self, context: SymbolicContext) -> None:
        service = self.service.eval(context)
        context.node.use(service, self.format_)

    def is_unknown(self) -> bool:
        return self.service.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class WriteParam(SymbolicRosApiCall):
    param: SymbolicString
    value: SymbolicValue

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "writes-to-param",
            "name": self.param.to_dict(),
            "value": self.value.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> None:
        param = self.param.eval(context)
        value = self.value.eval(context)
        context.node.write(param, value)

    def is_unknown(self) -> bool:
        # NOTE we don't consider the parameter value
        # we should discuss whether or not we should
        return self.param.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ReadParam(SymbolicRosApiCall, SymbolicValue):
    param: SymbolicString

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "reads-param",
            "name": self.param.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> None:
        param = self.param.eval(context)
        context.node.read(param)

    def is_unknown(self) -> bool:
        return self.param.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ReadParamWithDefault(SymbolicRosApiCall, SymbolicValue):
    param: SymbolicString
    default: SymbolicValue

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "reads-param-with-default",
            "name": self.param.to_dict(),
            "default": self.default.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> None:
        param = self.param.eval(context)
        default = self.default.eval(context)
        context.node.read(param, default)

    def is_unknown(self) -> bool:
        # NOTE same comment about default
        return self.param.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class HasParam(SymbolicRosApiCall, SymbolicBool):
    param: SymbolicString

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "checks-for-param",
            "name": self.param.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> None:
        param = self.param.eval(context)
        context.node.has_param(param)

    def is_unknown(self) -> bool:
        return self.param.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class DeleteParam(SymbolicRosApiCall):
    param: SymbolicString

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "deletes-param",
            "name": self.param.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> None:
        param = self.param.eval(context)
        context.node.delete_param(param)

    def is_unknown(self) -> bool:
        return self.param.is_unknown()
