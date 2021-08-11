# -*- coding: utf-8 -*-
__all__ = ("SymbolicStatement", "SymbolicFunction")

import abc
import typing as t

import attr


class SymbolicValue(abc.ABC):
    """Represents a symbolic value in a function summary."""


class SymbolicStatement(abc.ABC):
    """Represents a statement in a symbolic function summary."""


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
class SymbolicFunction:
    name: str
    body: SymbolicCompound
