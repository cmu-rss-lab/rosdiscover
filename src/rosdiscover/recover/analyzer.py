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
    SymbolicPublisherImpl,
    SymbolicRateImpl,
    SymbolicVariableReference,
    SymbolicWhile,
)

from .call import Publish, RateSleep
from .call import Subscriber, ServiceProvider, CreateTimer


@attr.s(auto_attribs=True)  # Can't use slots with cached_property
class SymbolicProgramAnalyzer:

    program: SymbolicProgram

    @cached_property
    def assigned_vars(self) -> t.Set[str]:
        return {a.variable for a in self.assignments}

    def inter_procedual_condition_var_assign(self, var_assign: SymbolicAssignment) -> SymbolicExpr:
        expr = var_assign.path_condition
        transitive_callers = self.program.transitive_callers(self.program.func_of_stmt(var_assign))
        for call in transitive_callers:
            expr = AndExpr.build(expr, call.condition)
        return expr
    
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
        result = []
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, SymbolicAssignment):
                    if not stmt in result:
                        result.append(stmt)

        return result

    @cached_property
    def pub_assignments(self) -> t.Map[str, SymbolicExpr]:
        result = {}
        for assign in self.assignments:
            if assign.unqualified_variable in self.publish_call_names:
                result[assign.unqualified_variable] = assign.value
            
        return result

    @cached_property
    def services(self) -> t.Set[ServiceProvider]:
        result = set()
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, ServiceProvider):
                    result.add(stmt)

        return result
    
    @cached_property
    def subscribers(self) -> t.Set[Subscriber]:
        result = set()
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, Subscriber) or isinstance(stmt, ServiceProvider) :
                    result.add(stmt)

        return result

    @cached_property
    def create_timers(self) -> t.Set[CreateTimer]:
        result = set()
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, CreateTimer):
                    result.add(stmt)

        return result


    @cached_property
    def rate_sleeps(self) -> t.List[RateSleep]:
        result = []
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, RateSleep):
                    result.append(stmt)

        return result

    @cached_property
    def rate_sleeps_json(self) -> t.List[t.Dict]:
        result = []
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, RateSleep):
                    result.append(stmt)

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
    def timer_callbacks_map(self) -> t.Set[t.Tuple[CreateTimer, SymbolicFunction]]:
        result = set()
        for timer in self.create_timers:
            if timer.callback_name == "unknown":
                continue
            result.add((timer, self.program.functions[timer.callback_name]))

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
            print(func)
            for stmt in func.body:
                if isinstance(stmt, Publish) and stmt not in result:
                    result.append(stmt)

        return result
    
    
    @cached_property
    def publish_call_names(self) -> t.List[str]:
        result = []
        for pub in self.publish_calls:
            result.append(pub.publisher)

        return result

    @cached_property
    def publisher_call_remaps(self) -> t.Map[str, t.List[str]]:
        result = {}
        for call in self.function_calls:
            for arg_name in call.arguments:
                if isinstance(call.arguments[arg_name], SymbolicPublisherImpl):
                    if arg_name not in result:
                        result[arg_name] = []
                    result[arg_name].append(str(call.arguments[arg_name].name))

        return result
    
    
    @cached_property
    def rate_call_remaps(self) -> t.Map[str, t.List[str]]:
        result = {}
        for call in self.function_calls:
            for arg_name in call.arguments:
                if isinstance(call.arguments[arg_name], SymbolicRateImpl):
                    if arg_name not in result:
                        result[arg_name] = []
                    result[arg_name].append(str(call.arguments[arg_name].name))

        return result
    
    @cached_property
    def function_calls(self) -> t.List[SymbolicFunctionCall]:
        result = []
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, SymbolicFunctionCall) and stmt not in result:
                    result.append(stmt)

        return result

    @cached_property
    def publish_calls_json(self) -> t.List[t.Dict]:
        result = []
        for p in self.publish_calls:
            result.append(p.to_dict())

        return result
                
    @cached_property
    def unclassified_publish_calls(self) -> t.List[Publish]:
        result = []
        for pub in self.publish_calls:
            if pub in self.periodic_publish_calls:
                continue
            if pub in self.reactive_publish:
                continue
            if pub not in result:
                result.append(pub)
        return result

    @cached_property
    def unclassified_publish_calls_json(self) -> t.List[t.Dict]:
        result = []
        for p in self.unclassified_publish_calls:
            result.append(p.to_dict())

        return result
    
    @cached_property
    def publish_calls_in_main(self) -> t.List[Publish]:
        result = []
        for pub_call in self.publish_calls:
            if self.program.entrypoint.body.contains(pub_call, self.program.functions):
                result.append(pub_call)


        return result

    @cached_property
    def publish_calls_in_sub_callback(self) -> t.List[Publish]:
        result = []
        for pub_call in self.publish_calls:
            for callback in self.subscriber_callbacks:
                if callback.body.contains(pub_call, self.program.functions):
                    result.append(pub_call)


        return result

    @cached_property
    def reactive_behavior_json(self) -> t.List[t.Dict]:
        result = []
        for t in self.sub_reactive_behavior:
            if (t[0].publisher in self.publisher_call_remaps):
                for pub in self.publisher_call_remaps[t[0].publisher]:
                    result.append({"publisher":{"variable" : pub}, "subscriber" : {"callback" : t[1]}})
            else:        
                result.append({"publisher":{"variable" : t[0].publisher}, "subscriber" : {"callback" : t[1]}})

        for pub_call in self.publish_calls_in_main:
            if (pub_call.publisher in self.publisher_call_remaps):
                for pub in self.publisher_call_remaps[pub_call.publisher]:
                    result.append({"publisher":{"variable" : pub}, "event" : "component-init"})
            else:  
                result.append({"publisher":{"variable" : pub_call.publisher}, "event" : "component-init"})
        return result

    @cached_property
    def sub_reactive_behavior(self) -> t.List[t.Tuple[Publish, str]]:
        result = []
        for pub_call in self.publish_calls:
            for (sub, callback) in self.subscriber_callbacks_map:
                if callback.body.contains(pub_call, self.program.functions) and (pub_call, sub.callback_name) not in result:
                    result.append((pub_call, sub.callback_name))
        return result
    
    @cached_property
    def reactive_behavior_map(self) -> t.Mapping[str, t.List[Publish]]:
        result = {}
        for pub_call in self.publish_calls:
            for (sub, callback) in self.subscriber_callbacks_map:
                if callback.body.contains(pub_call, self.program.functions):
                    if sub.callback_name not in result:
                        result[sub.callback_name] = []
                    result[sub.callback_name].append(pub_call)
        return result
    
    @cached_property
    def reactive_publish(self) -> t.Set[Publish]:
        result = set()
        for l in self.reactive_behavior_map.values():
            for pub_call in l:
                result.add(pub_call) 

        for pub_call in self.publish_calls_in_main:
            result.add(pub_call)
            
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
        for (pub_call, rate) in self.periodic_publish_calls_and_rates:
            result.append(pub_call)

        return result

    @cached_property
    def periodic_publish_calls_and_rates(self) -> t.List[t.Tuple(Publish, SymbolicExpr)]:
        result = []
        for pub_call in self.publish_calls:
            for while_stmt in self.while_loops:
                if while_stmt.body.contains(pub_call, self.program.functions):
                    for rate in self.rate_sleeps:
                        if while_stmt.body.contains(rate, self.program.functions) and pub_call not in result:
                            result.append((pub_call,rate.rate))

            for (timer, callback) in self.timer_callbacks_map:
                if callback.body.contains(pub_call, self.program.functions) and pub_call not in result:
                    result.append((pub_call, timer.rate))


        return result

    @cached_property
    def periodic_publish_calls_json(self) -> t.List[t.Dict]:
        result = []
        for (pub_call, rate) in self.periodic_publish_calls_and_rates:
            if (pub_call.publisher in self.publisher_call_remaps):
                for pub in self.publisher_call_remaps[pub_call.publisher]:
                    result.append({"publisher":{"variable" : pub}, "condition" : pub_call.condition.to_dict(), "rate" : rate.to_dict()})
            else:
                result.append({"publisher":{"variable" : pub_call.publisher}, "condition" : pub_call.condition.to_dict(), "rate" : rate.to_dict()})

        return result


    @cached_property
    def function_calls(self) -> t.List[SymbolicFunctionCall]:
        result = []
        for func in self.program.functions.values():
            for stmt in func.body:
                if isinstance(stmt, SymbolicFunctionCall) and stmt not in result:
                    result.append(stmt)

        return result
