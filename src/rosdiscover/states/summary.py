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


@attr.s(frozen=True, slots=True, auto_attribs=True)
class State:
    """Describes a concrete state of a node"""
    variables: t.Dict[StateVariable, SymbolicValue]


@attr.s(frozen=True, slots=True, auto_attribs=True)
class StateMachineSummary:
    """Summarises the state machine of a given node."""
    initial: State
    states: t.Set[State]
