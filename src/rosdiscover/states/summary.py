# -*- coding: utf-8 -*-
__all__ = ('StateMachineSummary')

import typing as t
import attr
from ..recover.symbolic import SymbolicValue, SymbolicValueType


@attr.s(frozen=True, slots=True, auto_attribs=True)
class StateVariable:
    """Identifies a variable that denotes state"""
    name: str
    type: SymbolicValueType

    def to_dict(self):
        return {'name': self.name,
                'type': self.type}


@attr.s(frozen=True, slots=True, auto_attribs=True)
class State:
    """Describes a concrete state of a node"""
    variables: t.Dict[StateVariable, SymbolicValue]

    def to_dict(self):
        vars = [{'variable': k.to_dict(),
                 'value': v.to_dict()} for (k, v) in self.variables.keys]
        return {'variables': vars}


@attr.s(frozen=True, slots=True, auto_attribs=True)
class StateMachineSummary:
    """Summarises the state machine of a given node."""
    initial: State
    states: t.Set[State]

    def to_dict(self) -> t.Dict[str, t.Any]:
        states = [s.to_dict() for s in self.states]
        return {'states': states,
                'initial': self.initial.to_dict()}
