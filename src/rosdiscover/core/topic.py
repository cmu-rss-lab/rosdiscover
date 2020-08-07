# -*- coding: utf-8 -*-
__all__ = ('Topic',)

from typing import Any, Dict

import attr


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Topic:
    """Describes a topic.

    Attributes
    ----------
    name: str
        The fully-qualified name of the topic.
    format: str
        The name of the format used by messages published to the topic.
    implicit: bool
        If :code:`True`, the topic is implicitly created by an associated
        action server.
    """
    name: str
    format: str
    implicit: bool

    def to_dict(self) -> Dict[str, Any]:
        return {'name': self.name, 'format': self.format, 'implicit': self.implicit}
