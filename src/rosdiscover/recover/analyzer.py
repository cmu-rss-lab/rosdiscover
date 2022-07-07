# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "SymbolicProgramAnalyzer"
)

import typing as t

import attr

from .symbolic import (
    SymbolicProgram,
    SymbolicFunction)

from .call import Publish, RateSleep
from .call import Subscriber


@attr.s
class SymbolicProgramAnalyzer:

    def subscribers(program: SymbolicProgram) -> t.Set[Subscriber]:
        result = set()
        for func in program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Subscriber):
                    result.add(stmt)

        return result

    def rate_sleeps(program: SymbolicProgram) -> t.Set[RateSleep]:
        result = set()
        for func in program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, RateSleep):
                    result.add(stmt)

        return result

    def subscriber_callbacks(program: SymbolicProgram) -> t.Set[SymbolicFunction]:
        result = set()
        for sub in SymbolicProgramAnalyzer.subscribers(program):
            result.add(program.functions[sub.callback_name])

        return result

    def publish_calls(program: SymbolicProgram) -> t.Set[Publish]:
        result = set()
        for func in program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Publish):
                    result.add(stmt)

        return result

    def publish_calls_in_sub_callback(program: SymbolicProgram) -> t.Set[Publish]:
        result = set()
        for pub_call in SymbolicProgramAnalyzer.publish_calls(program):
            for callback in SymbolicProgramAnalyzer.subscriber_callbacks(program):
                if callback.body.contains(pub_call, program.functions):
                    result.add(pub_call)

        return result

SymbolicProgramAnalyzer.subscribers = staticmethod(SymbolicProgramAnalyzer.subscribers)
SymbolicProgramAnalyzer.rate_sleeps = staticmethod(SymbolicProgramAnalyzer.rate_sleeps)
SymbolicProgramAnalyzer.subscriber_callbacks = staticmethod(SymbolicProgramAnalyzer.subscriber_callbacks)
SymbolicProgramAnalyzer.publish_calls = staticmethod(SymbolicProgramAnalyzer.publish_calls)
SymbolicProgramAnalyzer.publish_cpublish_calls_in_sub_callbackalls = staticmethod(SymbolicProgramAnalyzer.publish_calls_in_sub_callback)
