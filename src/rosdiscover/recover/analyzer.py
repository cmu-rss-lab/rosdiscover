# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "SymbolicProgramAnalyzer"
)
from functools import cached_property
import typing as t

import attr

from .symbolic import (
    AndExpr,
    SymbolicAssignment,
    SymbolicExpr,
    SymbolicProgram,
    SymbolicFunction,
    SymbolicFunctionCall,
    SymbolicWhile,
)

from .call import Publish, RateSleep
from .call import Subscriber


@attr.s(auto_attribs=True)  # Can't use slots with cached_property
class SymbolicProgramAnalyzer:

    program: SymbolicProgram

    @cached_property
    def assigned_vars(self) -> t.Set[str]:
        return {a.variable for a in self.assignments}

    def inter_procedual_condition(self, publish_call: Publish) -> SymbolicExpr:
        expr = publish_call.condition
        transitive_callers = self.program.transitive_callers(self.program.func_of_stmt(publish_call))
        for call in transitive_callers:
            expr = AndExpr.build(expr, call.condition)
        return expr

    def assignments_of_var(self, variable: str) -> t.Set[SymbolicAssignment]:
        result = set()
        for assign in self.assignments:
            if assign.variable == variable:
                result.add(assign)

        return result

    @cached_property
    def assignments(self) -> t.Set[SymbolicAssignment]:
        result = set()
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, SymbolicAssignment):
                    result.add(stmt)

        return result

    @cached_property
    def subscribers(self) -> t.Set[Subscriber]:
        result = set()
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Subscriber):
                    result.add(stmt)

        return result

    @cached_property
    def rate_sleeps(self) -> t.Set[RateSleep]:
        result = set()
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, RateSleep):
                    result.add(stmt)

        return result

    @cached_property
    def rate_sleeps_json(self) -> t.List[t.Dict]:
        result = set()
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, RateSleep):
                    result.add(stmt)

        return result

    @cached_property
    def subscriber_callbacks_map(self) -> t.Set[t.Tuple[Subscriber, SymbolicFunction]]:
        result = set()
        for sub in self.subscribers:
            if sub.callback_name == "unknown":
                continue
            result.add((sub, self.program.functions[sub.callback_name]))

        return result

    @cached_property
    def subscriber_callbacks(self) -> t.Set[SymbolicFunction]:
        return set(callback for (sub, callback) in self.subscriber_callbacks_map)

    @cached_property
    def subscriber_callbacks_json(self) -> t.List[t.Dict]:
        result = []
        for o in self.subscriber_callbacks:
            result.append(o.to_dict())

        return result

    @cached_property
    def publish_calls(self) -> t.List[Publish]:
        result = []
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Publish) and stmt not in result:
                    result.append(stmt)

        return result

    @cached_property
    def publish_calls_json(self) -> t.List[t.Dict]:
        result = []
        for p in self.publish_calls:
            result.append(p.to_dict())

        return result

    @cached_property
    def publish_calls_in_sub_callback(self) -> t.List[Publish]:
        result = []
        for pub_call in self.publish_calls:
            for callback in self.subscriber_callbacks:
                if callback.body.contains(pub_call, self.program.functions) and pub_call not in result:
                    result.append(pub_call)

        return result

    @cached_property
    def publish_calls_in_sub_callback_json(self) -> t.List[t.Dict]:
        result = []
        for o in self.publish_calls_in_sub_callback:
            result.append(o.to_dict())

        return result

    @cached_property
    def while_loops(self) -> t.List[SymbolicWhile]:
        result = []
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, SymbolicWhile) and stmt not in result:
                    result.append(stmt)

        return result

    @cached_property
    def while_loops_json(self) -> t.List[t.Dict]:
        result = []
        for o in self.while_loops:
            result.append(o.to_dict())

        return result

    @cached_property
    def periodic_publish_calls(self) -> t.List[Publish]:
        result = []
        for pub_call in self.publish_calls:
            for while_stmt in self.while_loops:
                if while_stmt.body.contains(pub_call, self.program.functions):
                    for rate in self.rate_sleeps:
                        if while_stmt.body.contains(rate, self.program.functions) and pub_call not in result:
                            result.append(pub_call)

        return result

    @cached_property
    def periodic_publish_calls_json(self) -> t.List[t.Dict]:
        result = []
        for o in self.periodic_publish_calls:
            result.append(o.to_dict())

        return result


    @cached_property
    def function_calls(self) -> t.List[SymbolicFunctionCall]:
        result = []
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, SymbolicFunctionCall) and stmt not in result:
                    result.append(stmt)

        return result
