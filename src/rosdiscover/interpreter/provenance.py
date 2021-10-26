# -*- coding: utf-8 -*-
__all__ = ("Provenance",)

import enum


class Provenance(enum.Enum):
    RECOVERED = "recovered"
    PLACEHOLDER = "placeholder"
    HANDWRITTEN = "handwritten"
    UNKNOWN = "unknown"
