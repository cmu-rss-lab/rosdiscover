# -*- coding: utf-8 -*-
from __future__ import annotations

from .call import Subscriber
from .call import Publish
__all__ = (
    "SymbolicStatesAnalyzer"
)

import typing as t
import attr
from functools import cached_property

from .symbolic import (
    SymbolicAssignment,
    SymbolicExpr,
    SymbolicFunction,
    SymbolicProgram,
    SymbolicVariableReference,
)
from .analyzer import SymbolicProgramAnalyzer

from loguru import logger

@attr.s(auto_attribs=True)  # Can't use slots with cached_property
class PeriodicTransition:
    interval: str 
    condition: SymbolicExpr
    state_changes: t.List[SymbolicAssignment]
    outputs: t.List[Publish]

    def to_dict(self) -> t.Dict[str, t.Any]:
        state_changes_dict = []
        for assign in self.state_changes:
            state_changes_dict.append(
                {"variable" : assign.variable,  
                 "new_value" : str(assign.value)}
            )

        outputs_dict = []
        for o in self.outputs:
            outputs_dict.append({"publisher": {"variable" : o.publisher}})

        dict_ = {
            "type": "interval",
            "condition" : str(self.condition),
            "interval": f"{self.interval}Hz",
            "state_changes": state_changes_dict,
            "outputs": outputs_dict,
        }

        return dict_

@attr.s(auto_attribs=True)  # Can't use slots with cached_property
class MessageTransition:
    trigger: Subscriber 
    condition: SymbolicExpr
    state_changes: t.List[SymbolicAssignment]
    outputs: t.List[Publish]

    def to_dict(self) -> t.Dict[str, t.Any]:
        state_changes_dict = []
        for assign in self.state_changes:
            state_changes_dict.append(
                {"variable" : assign.variable,  
                 "new_value" : str(assign.value)}
            )

        outputs_dict = []
        for o in self.outputs:
            outputs_dict.append({"publisher": {"variable" : o.publisher}})

        dict_ = {
            "type": "message",
            "condition" : str(self.condition),
            "callback": self.trigger.callback_name,
            "state_changes": state_changes_dict,
            "outputs": outputs_dict,
        }

        return dict_


@attr.s(auto_attribs=True)  # Can't use slots with cached_property
class SymbolicStatesAnalyzer:

    program: SymbolicProgram
    program_analyzer: SymbolicProgramAnalyzer

    def initial_value_assigns(self, stateVar) :
        result = set()
        if not stateVar.initial_value.is_unknown():
            return set()

        assigns = []
        for assign in self.program_analyzer.assignments_of_var(stateVar.variable):
            if self.program.entrypoint.body.contains(assign, self.program.functions) and assign not in assigns:
                assigns.append(assign)
        
        for v in assigns: #TODO: FIX ME
            if not v.value.is_unknown():
                result.add(self.program.func_of_stmt(v))
        return result

    @cached_property
    def state_vars(self) -> t.List[SymbolicVariableReference]:
        var_refs: t.List[SymbolicVariableReference] = []
        for pub in self.program_analyzer.publish_calls:
            cond = self.program_analyzer.inter_procedual_condition(pub)
            for expr in cond.decendents(True):
                if isinstance(expr, SymbolicVariableReference) and expr.variable in self.program_analyzer.assigned_vars:
                    pub_func = self.program.func_of_stmt(pub)
                    assign_funcs: t.Set[SymbolicFunction] = set()
                    for assign in self.program_analyzer.assignments_of_var(expr.variable):
                        assign_funcs.add(self.program.func_of_stmt(assign))
                    print(self.initial_value_assigns(expr))
                    if len(assign_funcs - {pub_func} - self.initial_value_assigns(expr)) > 0 and expr not in var_refs:
                        var_refs.append(expr)
        result = var_refs
        for var_ref in var_refs:
            for assign in self.program_analyzer.assignments_of_var(var_ref.variable):
                cond = self.program_analyzer.inter_procedual_condition_var_assign(assign)
                for expr in cond.decendents(True):
                    if isinstance(expr, SymbolicVariableReference) and expr.variable in self.program_analyzer.assigned_vars:
                        assign_func = self.program.func_of_stmt(assign)
                        assign_funcs: t.Set[SymbolicFunction] = set()
                        for assign in self.program_analyzer.assignments_of_var(expr.variable):
                            assign_funcs.add(self.program.func_of_stmt(assign))
                        print(self.initial_value_assigns(expr))
                        if len(assign_funcs - {assign_func} - self.initial_value_assigns(expr)) > 0 and expr not in var_refs:
                            result.append(expr)

        return result

    @cached_property
    def state_vars_json(self) -> t.List[t.Dict]:
        result = []
        logger.debug(f"Running state_vars_json")

        for var in self.state_vars:
            if var.initial_value.is_unknown():
                logger.debug(f"{var} is unknown")
                if var.variable in self.main_state_var_assigns_map:
                    logger.debug(f"GETTING INITIAL VALUE FROM MAIN: {self.main_state_var_assigns_map[var.variable]}. Value: {self.main_state_var_assigns_map[var.variable][0].value}")
                    vardic = var.to_dict()
                    for v in self.main_state_var_assigns_map[var.variable]:
                        if not v.value.is_unknown():
                            vardic["initial-value"] = v.value.to_dict() #TODO: FIX ME
                            continue
                    result.append(vardic)
                    continue

            result.append(var.to_dict())

        return result


    @cached_property
    def periodic_transitions(self) -> t.List[PeriodicTransition]:
        result: t.List[PeriodicTransition] = []
        for (pub_call, rate) in self.program_analyzer.periodic_publish_calls_and_rates:
            #t = MessageTransition(sub, )
            if str(pub_call.condition) is not "True":
                result.append(PeriodicTransition(interval=str(rate), condition=pub_call.condition, state_changes=[], outputs=[]))

        return result
    

    @cached_property
    def message_transitions(self) -> t.List[MessageTransition]:
        result: t.List[MessageTransition] = []
        for sub in self.program_analyzer.subscribers:
            #t = MessageTransition(sub, )
            if sub in self.sub_state_var_assigns:
                sub_state_var_assigns_ = self.sub_state_var_assigns[sub]
                outputs = []
                if sub.callback_name in self.program_analyzer.reactive_behavior_map:
                    outputs = self.program_analyzer.reactive_behavior_map[sub.callback_name]
                result.append(MessageTransition(trigger=sub, condition=sub_state_var_assigns_[0].path_condition, state_changes=sub_state_var_assigns_, outputs=outputs))
            elif sub.callback_name in self.program_analyzer.reactive_behavior_map:
                outputs = []
                if sub.callback_name in self.program_analyzer.reactive_behavior_map:
                    outputs = self.program_analyzer.reactive_behavior_map[sub.callback_name]
                result.append(MessageTransition(trigger=sub, condition=outputs[0].condition, state_changes=[], outputs=outputs))             

        return result
    
    @cached_property
    def message_transitions_json(self) -> t.List[t.Dict]:
        result = []

        for t in self.message_transitions:
            result.append(t.to_dict())


        for t in self.periodic_transitions:
            result.append(t.to_dict())

        return result

    @cached_property
    def _state_var_assigns(self) -> t.List[SymbolicAssignment]:
        result: t.List[SymbolicAssignment] = []
        for var in self.state_vars:
            for assign in self.program_analyzer.assignments_of_var(var.variable):
                if assign not in result:
                    result.append(assign)
        return result

    @cached_property
    def sub_state_var_assigns(self) -> t.Mapping[Subscriber, t.List[SymbolicAssignment]]:
        result: t.Mapping[Subscriber, t.List[SymbolicAssignment]] = {}
        for assign in self._state_var_assigns:
            for (sub, callback) in self.program_analyzer.subscriber_callbacks_map:
                if callback.body.contains(assign, self.program.functions):
                    if sub not in result:
                        result[sub] = []
                    result[sub].append(assign)
        return result

    @cached_property
    def main_state_var_assigns_map(self) -> t.Mapping[str, t.List[SymbolicAssignment]]:
        result: t.Mapping[str, t.List[SymbolicAssignment]] = {}
        for assign in self._state_var_assigns:
            if self.program.entrypoint.body.contains(assign, self.program.functions):
                if assign.variable not in result:
                    result[assign.variable] = []
                result[assign.variable].append(assign)
        return result

    @cached_property
    def main_state_var_assigns(self) -> t.List[SymbolicAssignment]:
        result: t.List[SymbolicAssignment] = []
        for assign in self._state_var_assigns:
            if self.program.entrypoint.body.contains(assign, self.program.functions):
                result.append(assign)
        return result
