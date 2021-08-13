# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = ("SymbolicProgramLoader",)

import json
import typing as t

from .symbolic import SymbolicProgram


class SymbolicProgramLoader:
    def load_from_dict(self, dict_: t.Mapping[str, t.Any]) -> SymbolicProgram:
        raise NotImplementedError

    def load(self, filename: str) -> SymbolicProgram:
        with open(filename, "r") as f:
            return self.load_from_dict(json.load(f))
