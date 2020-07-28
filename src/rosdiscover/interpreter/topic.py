# -*- coding: utf-8 -*-
__all__ = ('Topic',)
from typing import Any, Dict
import attr


@attr.s(frozen=True, slots=True)
class Topic:
    """Describes a topic"""
    name: str = attr.ib()
    format: str = attr.ib()
    implicit: bool = attr.ib()

    def to_dict(self) -> Dict[str, Any]:
        return {'name': self.name, 'format': self.format, 'implicit': self.implicit}
