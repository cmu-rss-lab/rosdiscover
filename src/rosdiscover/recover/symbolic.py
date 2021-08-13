# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "SymbolicAssignment",
    "SymbolicBool",
    "SymbolicCompound",
    "SymbolicFunction",
    "SymbolicStatement",
    "SymbolicString",
    "SymbolicParameter",
    "SymbolicProgram",
)

import abc
import typing as t

import attr


class SymbolicValue(abc.ABC):
    """Represents a symbolic value in a function summary."""


class SymbolicString(abc.ABC, SymbolicValue):
    """Represents a symbolic string value."""


class SymbolicInteger(abc.ABC, SymbolicValue):
    """Represents a symbolic integer value."""


class SymbolicBool(abc.ABC, SymbolicValue):
    """Represents a symbolic boolean value."""


class SymbolicStatement(abc.ABC):
    """Represents a statement in a symbolic function summary."""


class SymbolicAssignment(abc.ABC):
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


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicCompound(t.Sequence[SymbolicStatement]):
    """Represents a sequence of symbolic statements."""
    _statements: t.Sequence[SymbolicStatement]

    def __len__(self) -> int:
        return len(self._statements)

    def __getitem__(self, at: t.Union[int, slice]) -> SymbolicStatement:
        return self._statements[at]


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicFunctionCall(SymbolicFunction):
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


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicParameter:
    """Provides the definition for a symbolic function parameter."""
    name: str


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
        self,
        name: str,
        parameters: t.Iterable[SymbolicParameter],
        body: SymbolicCompound,
    ) -> SymbolicFunction:
        name_to_parameter = {param.name: param for param in parameters}
        return SymbolicFunction(name, name_to_parameter, body)


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicProgram:
    """Provides a symbolic summary for a given program.

    Attributes
    ----------
    functions: t.Mapping[str, SymbolicFunction]
        The symbolic functions within this program, indexed by name.
    """
    functions: t.Mapping[str, SymbolicFunction]

    @classmethod
    def build(self, functions: t.Iterable[SymbolicFunction]) -> SymbolicProgram:
        name_to_function = {function.name: function for function in functions}
        return SymbolicProgram(name_to_function)
