# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "StringLiteral",
    "SymbolicAssignment",
    "SymbolicBool",
    "SymbolicCompound",
    "SymbolicFunction",
    "SymbolicParameter",
    "SymbolicProgram",
    "SymbolicStatement",
    "SymbolicString",
)

import abc
import enum
import typing
import typing as t

import attr

from ..interpreter import NodeContext


class SymbolicValueType(enum.Enum):
    BOOL = "bool"
    INTEGER = "integer"
    STRING = "string"
    UNSUPPORTED = "unsupported"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_name(cls, name: str) -> SymbolicValueType:
        name_to_type = {
            "bool": cls.BOOL,
            "integer": cls.INTEGER,
            "string": cls.STRING,
            "unsupported": cls.UNSUPPORTED,
        }
        if name not in name_to_type:
            raise ValueError(f"unknown value type: {name}")
        return name_to_type[name]


class SymbolicValue(abc.ABC):
    """Represents a symbolic value in a function summary."""
    @abc.abstractmethod
    def to_dict(self) -> t.Dict[str, t.Any]:
        ...


class SymbolicString(SymbolicValue, abc.ABC):
    """Represents a symbolic string value."""


@attr.s(frozen=True, auto_attribs=True, slots=True)
class StringLiteral(SymbolicString):
    """Represents a literal string value."""
    value: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "string-literal",
            "literal": self.value,
        }


class SymbolicInteger(SymbolicValue, abc.ABC):
    """Represents a symbolic integer value."""


class SymbolicBool(SymbolicValue, abc.ABC):
    """Represents a symbolic boolean value."""


class SymbolicStatement(abc.ABC):
    """Represents a statement in a symbolic function summary."""
    @abc.abstractmethod
    def to_dict(self) -> t.Dict[str, t.Any]:
        ...


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicAssignment(SymbolicStatement):
    """Represents an assignment to a symbolic variable.

    Attributes
    ----------
    variable: str
        The name of the variable whose value is being assigned.
    value: SymbolicValue
        The value of the variable, provided as a symbolic expression.
    """
    variable: str
    value: SymbolicValue

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "assignment",
            "variable": self.variable,
            "value": self.value.to_dict(),
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicCompound(t.Sequence[SymbolicStatement], SymbolicStatement):
    """Represents a sequence of symbolic statements."""
    _statements: t.Sequence[SymbolicStatement]

    def __len__(self) -> int:
        return len(self._statements)

    @typing.overload
    def __getitem__(self, at: int) -> SymbolicStatement:
        ...

    @typing.overload
    def __getitem__(self, at: slice) -> t.Sequence[SymbolicStatement]:
        ...

    def __getitem__(
        self,
        at: t.Union[int, slice],
    ) -> t.Union[SymbolicStatement, t.Sequence[SymbolicStatement]]:
        return self._statements[at]

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "compound",
            "statements": [s.to_dict() for s in self._statements],
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicFunctionCall(SymbolicStatement):
    """Represents a call to a symbolic function.

    Attributes
    ----------
    callee: str
        The name of the function that is being called.
    arguments: t.Mapping[str, SymbolicValue]
        The arguments that should be used during the function call, indexed by
        the name of the corresponding argument.
    """
    callee: str
    arguments: t.Mapping[str, SymbolicValue]

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "call",
            "callee": self.callee,
            "arguments": {
                name: arg.to_dict() for (name, arg) in self.arguments.items()
            },
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicVariableReference(SymbolicValue):
    """Represents a symbolic variable reference.

    Attributes
    ----------
    variable: str
        The name of the value that is referenced.
    type: SymbolicValueType
        The type of the referenced variable.
    """
    variable: str
    type_: SymbolicValueType

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "variable-reference",
            "variable": self.variable,
            "type": str(self.type_),
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicParameter:
    """Provides the definition for a symbolic function parameter."""
    index: int
    name: str
    type_: SymbolicValueType

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "index": self.index,
            "name": self.name,
            "type": str(self.type_),
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicFunction:
    """Provides the definition of a single symbolic function.

    Attributes
    ----------
    name: str
        The fully-qualified name of the function.
    parameters: t.Mapping[str, SymbolicParameter]
        The parameters of this function, indexed by name.
    body: SymbolicCompound
        The definition of the function.
    """
    name: str
    parameters: t.Mapping[str, SymbolicParameter]
    body: SymbolicCompound

    @classmethod
    def build(
        cls,
        name: str,
        parameters: t.Iterable[SymbolicParameter],
        body: SymbolicCompound,
    ) -> SymbolicFunction:
        name_to_parameter = {param.name: param for param in parameters}
        return SymbolicFunction(name, name_to_parameter, body)

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "name": self.name,
            "parameters": [p.to_dict() for p in self.parameters.values()],
            "body": self.body.to_dict(),
        }


@attr.s(auto_attribs=True, slots=True)
class _SymbolicContext:
    program: SymbolicProgram
    function: SymbolicFunction
    node: NodeContext

    @classmethod
    def create(
        cls,
        program: SymbolicProgram,
        node: NodeContext,
    ) -> _SymbolicContext:
        return _SymbolicContext(
            program=program,
            function=program.main,
            node=node,
        )

    def for_function_call(self, function: SymbolicFunction) -> _SymbolicContext:
        """Creates a new symbolic context that represents the scope of a function call."""
        return _SymbolicContext(self.program, function, self.node)


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicProgram:
    """Provides a symbolic summary for a given program.

    Attributes
    ----------
    functions: t.Mapping[str, SymbolicFunction]
        The symbolic functions within this program, indexed by name.

    Raises
    ------
    ValueError
        If this program does not provide a "main" function.
    """
    functions: t.Mapping[str, SymbolicFunction]

    @functions.validator
    def must_have_main_function(
        self,
        attribute: str,
        value: t.Any,
    ) -> None:
        if "main" not in self.functions:
            raise ValueError("symbolic programs must provide a 'main' function")

    @classmethod
    def build(cls, functions: t.Iterable[SymbolicFunction]) -> SymbolicProgram:
        name_to_function = {function.name: function for function in functions}
        return SymbolicProgram(name_to_function)

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {"program": {name: f.to_dict() for (name, f) in self.functions.items()}}

    @property
    def main(self) -> SymbolicFunction:
        """Returns the main function (i.e., entrypoint) for this program."""
        return self.functions["main"]

    def eval(self, node: NodeContext) -> None:
        context = _SymbolicContext.create(program, context)


        raise NotImplementedError
