# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = ("SymbolicProgramLoader",)

import typing as t

import attr
import roswire
from loguru import logger

from .call import (
    DeleteParam,
    HasParam,
    Publisher,
    ReadParam,
    ReadParamWithDefault,
    RosInit,
    ServiceCaller,
    ServiceProvider,
    Subscriber,
    WriteParam,
)
from .symbolic import (
    Concatenate,
    StringLiteral,
    SymbolicArg,
    SymbolicAssignment,
    SymbolicCompound,
    SymbolicFunction,
    SymbolicFunctionCall,
    SymbolicNodeHandle,
    SymbolicNodeHandleImpl,
    SymbolicNodeName,
    SymbolicParameter,
    SymbolicProgram,
    SymbolicStatement,
    SymbolicString,
    SymbolicUnknown,
    SymbolicValue,
    SymbolicValueType,
    SymbolicVariableReference,
)
from ..config import Config


@attr.s
class SymbolicProgramLoader:
    _app: roswire.app.App = attr.ib()

    @classmethod
    def for_config(cls, config: Config) -> SymbolicProgramLoader:
        return SymbolicProgramLoader(config.app)

    def _load_parameter(self, dict_: t.Mapping[str, t.Any]) -> SymbolicParameter:
        index: int = dict_["index"]
        name: str = dict_["name"]
        type_ = SymbolicValueType.from_name(dict_["type"])
        return SymbolicParameter(
            index=index,
            name=name,
            type_=type_,
        )

    def _load_string_literal(self, dict_: t.Mapping[str, t.Any]) -> StringLiteral:
        assert dict_["kind"] == "string-literal"
        return StringLiteral(dict_["literal"])

    def _load_arg(self, dict_: t.Mapping[str, t.Any]) -> SymbolicArg:
        assert dict_["kind"] == "arg"
        return SymbolicArg(dict_["name"])

    def _load_variable_reference(self, dict_: t.Mapping[str, t.Any]) -> SymbolicVariableReference:
        assert dict_["kind"] == "variable-reference"
        type_ = SymbolicValueType.from_name(dict_["type"])
        return SymbolicVariableReference(
            variable=dict_["variable"],
            type_=type_,
        )

    def _load_node_handle(self, dict_: t.Mapping[str, t.Any]) -> SymbolicNodeHandle:
        assert dict_["kind"] == "node-handle"
        namespace = self._load_string(dict_["namespace"])
        return SymbolicNodeHandleImpl(namespace)

    def _load_string(self, dict_: t.Mapping[str, t.Any]) -> SymbolicString:
        value = self._load_value(dict_)
        assert isinstance(value, SymbolicString)
        return value

    def _load_value(self, dict_: t.Mapping[str, t.Any]) -> SymbolicValue:
        kind: str = dict_["kind"]
        if kind == "concatenate":
            return self._load_concatenate(dict_)
        elif kind == "arg":
            return self._load_arg(dict_)
        elif kind == "string-literal":
            return self._load_string_literal(dict_)
        elif kind == "node-handle":
            return self._load_node_handle(dict_)
        elif kind == "variable-reference":
            return self._load_variable_reference(dict_)
        elif kind == "reads-param":
            return self._load_reads_param(dict_)
        elif kind == "reads-param-with-default":
            return self._load_reads_param_with_default(dict_)
        elif kind == "checks-for-param":
            return self._load_checks_for_param(dict_)
        elif kind == "unknown":
            return SymbolicUnknown()
        elif kind == "node-name":
            return SymbolicNodeName()
        else:
            raise ValueError(f"failed to load value type: {kind}")

    def _load_concatenate(self, dict_: t.Mapping[str, t.Any]) -> Concatenate:
        lhs = self._load_string(dict_["lhs"])
        rhs = self._load_string(dict_["rhs"])
        return Concatenate(lhs, rhs)

    def _load_assignment(self, dict_: t.Mapping[str, t.Any]) -> SymbolicAssignment:
        variable = dict_["variable"]
        value = self._load_value(dict_["value"])
        return SymbolicAssignment(variable, value)

    def _load_rosinit(self, dict_: t.Mapping[str, t.Any]) -> RosInit:
        name = self._load_string(dict_["name"])
        return RosInit(name)

    def _load_publishes_to(self, dict_: t.Mapping[str, t.Any]) -> Publisher:
        topic = self._load_string(dict_["name"])
        return Publisher(topic, dict_["format"])

    def _load_subscribes_to(self, dict_: t.Mapping[str, t.Any]) -> Subscriber:
        topic = self._load_string(dict_["name"])
        return Subscriber(topic, dict_["format"])

    def _load_calls_service(self, dict_: t.Mapping[str, t.Any]) -> ServiceCaller:
        service = self._load_string(dict_["name"])
        return ServiceCaller(service, dict_["format"])

    def _load_provides_service(self, dict_: t.Mapping[str, t.Any]) -> ServiceProvider:
        service = self._load_string(dict_["name"])

        if "format" in dict_:
            return ServiceProvider(service, dict_["format"])

        response_format_name = dict_["response-format"]
        request_format_name = dict_["request-format"]
        logger.debug(
            f"locating corresponding format for service [{service}]:\n"
            f" - response format: {response_format_name}\n"
            f" - request format: {request_format_name}"
        )

        if response_format_name.endswith("Response"):
            service_format_name = response_format_name[:-8]
            logger.debug(f"determined format for service [{service}]: {service_format_name}")
            return ServiceProvider(service, service_format_name)

        logger.warning(f"unable to determine format for service: {service}")
        return ServiceProvider(service, "\\unknown")

    def _load_reads_param(self, dict_: t.Mapping[str, t.Any]) -> ReadParam:
        param = self._load_string(dict_["name"])
        return ReadParam(param)

    def _load_reads_param_with_default(
        self,
        dict_: t.Mapping[str, t.Any],
    ) -> ReadParamWithDefault:
        param = self._load_string(dict_["name"])
        default = self._load_value(dict_["default"])
        return ReadParamWithDefault(param, default)

    def _load_writes_to_param(self, dict_: t.Mapping[str, t.Any]) -> WriteParam:
        param = self._load_string(dict_["name"])
        value = self._load_value(dict_["value"])
        return WriteParam(param, value)

    def _load_deletes_param(self, dict_: t.Mapping[str, t.Any]) -> DeleteParam:
        param = self._load_string(dict_["name"])
        return DeleteParam(param)

    def _load_checks_for_param(self, dict_: t.Mapping[str, t.Any]) -> HasParam:
        param = self._load_string(dict_["name"])
        return HasParam(param)

    def _load_function_call(self, dict_: t.Mapping[str, t.Any]) -> SymbolicFunctionCall:
        arguments: t.Mapping[str, SymbolicValue] = {
            arg_name: self._load_value(arg_dict) for (arg_name, arg_dict) in dict_["arguments"].items()
        }
        logger.debug(f"loading symbolic function call arguments: {arguments}")
        return SymbolicFunctionCall(
            callee=dict_["callee"],
            arguments=arguments,
        )

    def _load_statement(self, dict_: t.Mapping[str, t.Any]) -> SymbolicStatement:
        kind: str = dict_["kind"]
        if kind == "assignment":
            return self._load_assignment(dict_)
        elif kind == "call":
            return self._load_function_call(dict_)
        elif kind == "ros-init":
            return self._load_rosinit(dict_)
        elif kind == "publishes-to":
            return self._load_publishes_to(dict_)
        elif kind == "subscribes-to":
            return self._load_subscribes_to(dict_)
        elif kind == "calls-service":
            return self._load_calls_service(dict_)
        elif kind == "provides-service":
            return self._load_provides_service(dict_)
        elif kind == "writes-to-param":
            return self._load_writes_to_param(dict_)
        elif kind == "deletes-param":
            return self._load_deletes_param(dict_)
        elif kind == "checks-for-param":
            return self._load_checks_for_param(dict_)
        elif kind == "compound":
            return self._load_compound(dict_)
        else:
            raise ValueError(f"unknown statement kind: {kind}")

    def _load_compound(self, dict_: t.Mapping[str, t.Any]) -> SymbolicCompound:
        assert dict_["kind"] == "compound"
        statements = [self._load_statement(d) for d in dict_["statements"]]
        return SymbolicCompound(statements)

    def _load_function(self, dict_: t.Mapping[str, t.Any]) -> SymbolicFunction:
        name: str = dict_["name"]
        parameters = [self._load_parameter(d) for d in dict_["parameters"]]
        body = self._load_compound(dict_["body"])
        return SymbolicFunction.build(
            name,
            parameters,
            body,
        )

    def load(self, dict_: t.Mapping[str, t.Any]) -> SymbolicProgram:
        dict_ = dict_["program"]
        return SymbolicProgram.build(
            dict_["entrypoint"],
            (self._load_function(d) for d in dict_["functions"])
        )
