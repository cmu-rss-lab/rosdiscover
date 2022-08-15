# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "SymbolicStatesAnalyzer"
)

import attr

from .symbolic import (
    SymbolicProgram,
)
from .analyzer import SymbolicProgramAnalyzer


@attr.s(auto_attribs=True, slots=True)
class SymbolicStatesAnalyzer:

    program: SymbolicProgram
    program_analyzer: SymbolicProgramAnalyzer
