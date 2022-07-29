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
    Publish,
    RateSleep,
    ReadParam,
    ReadParamWithDefault,
    RosInit,
    ServiceCaller,
    ServiceProvider,
    Subscriber,
    WriteParam,
)
from .symbolic import (
    BinaryMathExpr,
    CompareExpr,
    Concatenate,
    OrExpr,
    AndExpr,
    StringLiteral,
    FloatLiteral,
    IntLiteral,
    BoolLiteral,
    SymbolicArg,
    SymbolicAssignment,
    SymbolicCompound,
    SymbolicExpr,
    SymbolicFloat,
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
    SymbolicWhile,
    SymbolicIf,
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

    def _load_bool_literal(self, dict_: t.Mapping[str, t.Any]) -> BoolLiteral:
        assert dict_["kind"] == "bool-literal"
        return BoolLiteral(dict_["literal"])

    def _load_string_literal(self, dict_: t.Mapping[str, t.Any]) -> StringLiteral:
        assert dict_["kind"] == "string-literal"
        return StringLiteral(dict_["literal"])

    def _load_float_literal(self, dict_: t.Mapping[str, t.Any]) -> FloatLiteral:
        assert dict_["kind"] == "float-literal"
        return FloatLiteral(value=float(dict_["literal"]))

    def _load_int_literal(self, dict_: t.Mapping[str, t.Any]) -> IntLiteral:
        assert dict_["kind"] == "int-literal"
        return IntLiteral(value=int(dict_["literal"]))

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

    def _load_float(self, dict_: t.Mapping[str, t.Any]) -> SymbolicFloat:
        value = self._load_value(dict_)
        assert isinstance(value, SymbolicFloat)
        return value

    def _load_or_expr(self, dict_: t.Mapping[str, t.Any]) -> OrExpr:
        lhs = self._load_expr(dict_["lhs"])
        rhs = self._load_expr(dict_["rhs"])
        return OrExpr(lhs=lhs, rhs=rhs)

    def _load_and_expr(self, dict_: t.Mapping[str, t.Any]) -> AndExpr:
        lhs = self._load_expr(dict_["lhs"])
        rhs = self._load_expr(dict_["rhs"])
        return AndExpr(lhs=lhs, rhs=rhs)

    def _load_binary_math_expr(self, dict_: t.Mapping[str, t.Any]) -> BinaryMathExpr:
        lhs = self._load_expr(dict_["lhs"])
        rhs = self._load_expr(dict_["rhs"])
        operator = dict_["operator"]
        return BinaryMathExpr(lhs=lhs, rhs=rhs, operator=operator)

    def _load_compare_expr(self, dict_: t.Mapping[str, t.Any]) -> CompareExpr:
        lhs = self._load_expr(dict_["lhs"])
        rhs = self._load_expr(dict_["rhs"])
        operator = dict_["operator"]
        return CompareExpr(lhs=lhs, rhs=rhs, operator=operator)

    def _load_binary_expr(self, dict_: t.Mapping[str, t.Any]) -> SymbolicExpr:
        operator: str = dict_["operator"]
        if operator == "||":
            return self._load_or_expr(dict_)
        elif operator == "&&":
            return self._load_and_expr(dict_)
        elif operator in ["+", "-", "/", "*", "%"]:
            return self._load_binary_math_expr(dict_)
        elif operator in ["<", "<=", ">", ">=", "=="]:
            return self._load_compare_expr(dict_)
        else:
            raise ValueError(f"failed to load binary expression with operator: {operator}")

    def _load_expr(self, dict_: t.Mapping[str, t.Any]) -> SymbolicExpr:
        kind: str = dict_["kind"]
        if kind == "concatenate":
            return self._load_concatenate(dict_)
        elif kind == "arg":
            return self._load_arg(dict_)
        else:
            return self._load_value(dict_)

    def _load_value(self, dict_: t.Mapping[str, t.Any]) -> SymbolicValue:
        kind: str = dict_["kind"]
        if kind == "concatenate":
            return self._load_concatenate(dict_)
        elif kind == "arg":
            return self._load_arg(dict_)
        elif kind == "string-literal":
            return self._load_string_literal(dict_)
        elif kind == "bool-literal":
            return self._load_bool_literal(dict_)
        elif kind == "int-literal":
            return self._load_int_literal(dict_)
        elif kind == "float-literal":
            return self._load_float_literal(dict_)
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

    def _load_publish(self, dict_: t.Mapping[str, t.Any]) -> Publish:
        return Publish(publisher=dict_["publisher"], path_condition=self._load_expr(dict_["path_condition"]))

    def _load_rate_sleep(self, dict_: t.Mapping[str, t.Any]) -> RateSleep:
        rate = self._load_float(dict_["rate"])
        return RateSleep(rate)

    def _load_publishes_to(self, dict_: t.Mapping[str, t.Any]) -> Publisher:
        topic = self._load_string(dict_["name"])
        return Publisher(topic, dict_["format"])

    def _load_subscribes_to(self, dict_: t.Mapping[str, t.Any]) -> Subscriber:
        topic = self._load_string(dict_["name"])
        return Subscriber(topic, dict_["format"], dict_["callback-name"])

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
            path_condition=self._load_expr(dict_["path_condition"]),
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
        elif kind == "publish":
            return self._load_publish(dict_)
        elif kind == "ratesleep":
            return self._load_rate_sleep(dict_)
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
        elif kind == "while":
            return self._load_while(dict_)
        elif kind == "if":
            return self._load_if(dict_)
        else:
            raise ValueError(f"unknown statement kind: {kind}")

    def _load_while(self, dict_: t.Mapping[str, t.Any]) -> SymbolicWhile:
        assert dict_["kind"] == "while"
        return SymbolicWhile(self._load_compound(dict_["body"]), self._load_value(dict_["condition"]))

    def _load_if(self, dict_: t.Mapping[str, t.Any]) -> SymbolicIf:
        assert dict_["kind"] == "if"
        return SymbolicIf(
            true_body=self._load_compound(dict_["trueBranchBody"]),
            false_body=self._load_compound(dict_["falseBranchBody"]),
            condition=self._load_value(dict_["condition"]),
        )

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
