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


@attr.s(auto_attribs=True, slots=True)
class SymbolicContext:
    """Used to maintain program state during interpretation.

    Attributes
    ----------
    program: SymbolicProgram
        The program that is being interpreted.
    function: SymbolicFunction
        The current function that is being interpreted.
    node: NodeContext
        The node context that the symbolic program is being used to construct.
    _vars: t.Dict[str, t.Any]
        A mapping from the names of in-scope variables to their values.
    """
    program: SymbolicProgram
    function: SymbolicFunction
    node: NodeContext
    _vars: t.Dict[str, t.Any] = attr.ib(factory=dict)

    @classmethod
    def create(
        cls,
        program: SymbolicProgram,
        node: NodeContext,
    ) -> SymbolicContext:
        return SymbolicContext(
            program=program,
            function=program.main,
            node=node,
        )

    def for_function_call(self, function: SymbolicFunction) -> SymbolicContext:
        """Creates a new symbolic context that represents the scope of a function call."""
        return SymbolicContext(self.program, function, self.node)

    def load(self, variable: str) -> t.Any:
        """Loads the value of a given variable.

        Raises
        ------
        ValueError
            If no variable exists with the given name.
        """
        return self._vars[variable]

    def store(self, variable: str, value: t.Any) -> None:
        """Stores the value of a given variable.

        Raises
        ------
        ValueError
            If a definition for the given variable already exists as that would violate
            single static assignment (SSA) rules.
        """
        if variable in self._vars:
            raise ValueError(f"variable already defined in this scope: {variable}")

        self._vars[variable] = value


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

    @abc.abstractmethod
    def eval(self, context: SymbolicContext) -> t.Any:
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

    def eval(self, context: SymbolicContext) -> t.Any:
        return self.value


class SymbolicInteger(SymbolicValue, abc.ABC):
    """Represents a symbolic integer value."""


class SymbolicBool(SymbolicValue, abc.ABC):
    """Represents a symbolic boolean value."""


class SymbolicStatement(abc.ABC):
    """Represents a statement in a symbolic function summary."""
    @abc.abstractmethod
    def to_dict(self) -> t.Dict[str, t.Any]:
        ...

    @abc.abstractmethod
    def eval(self, context: SymbolicContext) -> None:
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

    def eval(self, context: SymbolicContext) -> None:
        concrete_value = self.value.eval(context)
        context.store(self.variable, concrete_value)


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

    def eval(self, context: SymbolicContext) -> None:
        for statement in self._statements:
            statement.eval(context)


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

    def eval(self, context: SymbolicContext) -> None:
        function = context.program.functions[self.callee]
        context = context.for_function_call(function)
        function.body.eval(context)


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

    def eval(self, context: SymbolicContext) -> t.Any:
        return context.load(self.variable)


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


@attr.s(frozen=True, slots=True)
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
    functions: t.Mapping[str, SymbolicFunction] = attr.ib()

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
        return {
            "program": {
                "functions": [f.to_dict() for f in self.functions.values()],
            },
        }

    @property
    def main(self) -> SymbolicFunction:
        """Returns the main function (i.e., entrypoint) for this program."""
        return self.functions["main"]

    def eval(self, node: NodeContext) -> None:
        context = SymbolicContext.create(self, node)
        self.main.body.eval(context)
