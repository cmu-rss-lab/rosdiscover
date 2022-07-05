# -*- coding: utf-8 -*-
from __future__ import annotations
from turtle import st

__all__ = (
    "Concatenate",
    "StringLiteral",
    "FloatLiteral",
    "SymbolicArg",
    "SymbolicAssignment",
    "SymbolicBool",
    "SymbolicCompound",
    "SymbolicFunction",
    "SymbolicNodeHandle",
    "SymbolicNodeHandleImpl",
    "SymbolicNodeName",
    "SymbolicParameter",
    "SymbolicProgram",
    "SymbolicStatement",
    "SymbolicString",
    "SymbolicFloat",
    "SymbolicUnknown",
)

import abc
import enum
import typing
import typing as t

from loguru import logger
import attr

from .symbolic import *
from .call import Publish, RateSleep
from .call import Subscriber


@attr.s(frozen=True, slots=True)
class SymbolicProgram:
    """Provides a symbolic summary for a given program.

    Attributes
    ----------
    entrypoint_name: str
        The name of the function that serves as the entry point for the
        program.
    functions: t.Mapping[str, SymbolicFunction]
        The symbolic functions within this program, indexed by name.

    Raises
    ------
    ValueError
        If this program does not provide a "main" function.
    """
    entrypoint_name: str = attr.ib()
    functions: t.Mapping[str, SymbolicFunction] = attr.ib()

    @functions.validator
    def must_have_entry_function(
        self,
        attribute: str,
        value: t.Any,
    ) -> None:
        if self.entrypoint_name not in self.functions:
            raise ValueError(f"failed to find definition for entrypoint function: {self.entrypoint_name}")

    @classmethod
    def build(cls, entrypoint: str, functions: t.Iterable[SymbolicFunction]) -> SymbolicProgram:
        name_to_function = {function.name: function for function in functions}
        if entrypoint not in name_to_function:
            logger.warning(
                f"The entrypoint '{entrypoint}' does not appear to reach any ROS API calls."
                " Adding an empty placeholder function."
            )
            name_to_function[entrypoint] = SymbolicFunction.empty(entrypoint)
        return SymbolicProgram(entrypoint, name_to_function)

    @property
    def subscribers(self) -> t.Set[Subscriber]:
        result = set()
        for func in self.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Subscriber):
                    result.add(stmt)

        return result        

    @property
    def rate_sleeps(self) -> t.Set[RateSleep]:
        result = set()
        for func in self.functions.values():
            for stmt in func.body:
                if isinstance(stmt, RateSleep):
                    result.add(stmt)

        return result

    @property
    def subscriber_callbacks(self) -> t.Set[SymbolicFunction]:
        result = set()
        for sub in self.subscribers:
            result.add(self.functions[sub.callback_name])

        return result

    @property
    def publish_calls(self) -> t.Set[Publish]:
        result = set()
        for func in self.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Publish):
                    result.add(stmt)

        return result

    @property
    def publish_calls_in_sub_callback(self) -> t.Set[Publish]:
        result = set()
        for pub_call in self.publish_calls:
            for callback in self.subscriber_callbacks:
                if callback.body.contains(pub_call, self.functions):
                    result.add(pub_call)

        return result        

    @property
    def unreachable_functions(self) -> t.Set[SymbolicFunction]:
        """Returns the set of functions that are unreachable from the entrypoint of this program.
        Unreachable functions almost always indicate incomplete control flow information
        (due to, e.g., certain callbacks). In some cases, an architecturally relevant function may
        truly be unreachable.
        """
        queue: t.List[SymbolicFunction] = [self.entrypoint]
        reached: t.Set[SymbolicFunction] = set()

        while queue:
            function = queue.pop(0)
            reached.add(function)
            calls = set(self.functions[name] for name in function.calls)
            for called_function in calls:
                if called_function not in reached:
                    queue.append(called_function)

        unreachable = set(self.functions.values()).difference(reached)
        return unreachable

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "program": {
                "entrypoint": self.entrypoint_name,
                "functions": [f.to_dict() for f in self.functions.values()],
            },
        }

    @property
    def entrypoint(self) -> SymbolicFunction:
        """The entrypoint for this program."""
        return self.functions[self.entrypoint_name]

    def eval(self, node: NodeContext) -> None:
        context = SymbolicContext.create(self, node)
        self.entrypoint.body.eval(context)
