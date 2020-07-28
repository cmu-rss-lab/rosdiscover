# -*- coding: utf-8 -*-
__all__ = ('Topic',)

from typing import Any, Dict

import attr


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Topic:
    """Describes a topic"""
    name: str
    format: str
    implicit: bool

    def to_dict(self) -> Dict[str, Any]:
        return {'name': self.name, 'format': self.format, 'implicit': self.implicit}
