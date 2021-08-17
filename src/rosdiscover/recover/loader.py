# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = ("SymbolicProgramLoader",)

import typing as t

from .call import (
    HasParam,
    Publisher,
    ReadParam,
    ReadParamWithDefault,
    RosInit,
    ServiceCaller,
    ServiceProvider,
    Subscriber,
    SymbolicRosApiCall,
    WriteParam,
)
from .symbolic import (
    StringLiteral,
    SymbolicAssignment,
    SymbolicCompound,
    SymbolicFunction,
    SymbolicParameter,
    SymbolicProgram,
    SymbolicStatement,
    SymbolicString,
    SymbolicValue,
    SymbolicValueType,
    SymbolicVariableReference,
)


class SymbolicProgramLoader:
    def _load_parameter(self, dict_: t.Mapping[str, t.Any]) -> SymbolicParameter:
        assert dict_["kind"] == "parameter"
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

    def _load_variable_reference(self, dict_: t.Mapping[str, t.Any]) -> SymbolicVariableReference:
        assert dict_["kind"] == "variable-reference"
        type_ = SymbolicValueType.from_name(dict_["type"])
        return SymbolicVariableReference(
            variable=dict_["variable"],
            type_=type_,
        )

    def _load_string(self, dict_: t.Mapping[str, t.Any]) -> SymbolicString:
        value = self._load_value(dict_)
        assert isinstance(value, SymbolicString)
        return value

    def _load_value(self, dict_: t.Mapping[str, t.Any]) -> SymbolicValue:
        kind: str = dict_["kind"]
        try:
            loader: t.Callable[[t.Mapping[str, t.Any]], SymbolicValue] = ({
                "string-literal": self._load_string_literal,
                "variable-reference": self._load_variable_reference,
            })[kind]
        except KeyError:
            raise ValueError(f"failed to load value type: {kind}")
        return loader(dict_)

    def _load_assignment(self, dict_: t.Mapping[str, t.Any]) -> SymbolicAssignment:
        assert dict_["kind"] == "assignment"
        variable = dict_["variable"]
        value = self._load_value(dict_["value"])
        return SymbolicAssignment(variable, value)

    def _load_rosinit(self, dict_: t.Mapping[str, t.Any]) -> RosInit:
        assert dict_["kind"] == "ros-init"
        name = self._load_string(dict_["name"])
        return RosInit(name)

    def _load_publishes_to(self, dict_: t.Mapping[str, t.Any]) -> Publisher:
        assert dict_["kind"] == "publishes-to"
        topic = self._load_string(dict_["name"])
        return Publisher(topic=topic, format_=dict_["format"])

    def _load_statement(self, dict_: t.Mapping[str, t.Any]) -> SymbolicStatement:
        kind: str = dict_["kind"]

        try:
            loader: t.Callable[[t.Mapping[str, t.Any]], SymbolicValue] = ({
                "ros-init": self._load_rosinit,
                "publishes-to": self._load_publishes_to,
            })[kind]
        except KeyError:
            raise ValueError(f"failed to load statement: {dict_}")

        return loader(dict_)

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
        return SymbolicProgram.build(
            self._load_function(d) for d in dict_["functions"]
        )
