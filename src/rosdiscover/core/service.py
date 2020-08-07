# -*- coding: utf-8 -*-
__all__ = ('Service',)

from typing import Any, Dict

import attr


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Service:
    """Describes a service.

    Attributes
    ----------
    name: str
        The fully-qualified name of the service.
    format: str
        The name of the format used by the service.
    """
    name: str
    format: str

    def to_dict(self) -> Dict[str, Any]:
        return {'name': self.name, 'format': self.format}
