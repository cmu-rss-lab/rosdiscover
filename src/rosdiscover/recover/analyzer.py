# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "SymbolicProgramAnalyzer"
)

import typing as t

import attr

from .symbolic import (
    SymbolicProgram,
    SymbolicFunction,
    SymbolicWhile
    )

from .call import Publish, RateSleep
from .call import Subscriber


@attr.s
class SymbolicProgramAnalyzer:

    @classmethod
    def subscribers(cls, program: SymbolicProgram) -> t.Set[Subscriber]:
        result = set()
        for func in program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Subscriber):
                    result.add(stmt)

        return result

    @classmethod
    def rate_sleeps(cls, program: SymbolicProgram) -> t.Set[RateSleep]:
        result = set()
        for func in program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, RateSleep):
                    result.add(stmt)

        return result

    @classmethod
    def subscriber_callbacks(cls, program: SymbolicProgram) -> t.Set[SymbolicFunction]:
        result = set()
        for sub in cls.subscribers(program):
            result.add(program.functions[sub.callback_name])

        return result

    @classmethod
    def publish_calls(cls, program: SymbolicProgram) -> t.Set[Publish]:
        result = set()
        for func in program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Publish):
                    result.add(stmt)

        return result

    @classmethod
    def publish_calls_in_sub_callback(cls, program: SymbolicProgram) -> t.Set[Publish]:
        result = set()
        for pub_call in cls.publish_calls(program):
            for callback in cls.subscriber_callbacks(program):
                if callback.body.contains(pub_call, program.functions):
                    result.add(pub_call)

        return result

    @classmethod
    def while_loops(cls, program: SymbolicProgram) -> t.List[SymbolicWhile]:
        result = []
        for func in program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, SymbolicWhile):
                    result.append(stmt)

        return result

    @classmethod
    def periodic_publish_calls(cls, program: SymbolicProgram) -> t.Set[Publish]:
        result = set()
        for pub_call in cls.publish_calls(program):
            for while_stmt in cls.while_loops(program):
                if while_stmt.body.contains(pub_call, cls.functions(program)):
                    for rate in cls.rate_sleeps(program):
                        if while_stmt.body.contains(rate, cls.functions(program)):
                            result.add(pub_call)

        return result
