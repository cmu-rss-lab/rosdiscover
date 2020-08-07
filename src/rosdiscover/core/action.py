# -*- coding: utf-8 -*-
__all__ = ('Action',)

from typing import Any, Dict

import attr


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Action:
    """Describes an action.

    Attributes
    ----------
    name: str
        The fully-qualified name of the action.
    format: str
        The name of the format used by the action.
    """
    name: str
    format: str

    def to_dict(self) -> Dict[str, Any]:
        return {'name': self.name, 'format': self.format}
