# -*- coding: utf-8 -*-
__all__ = ('Config',)

from typing import Sequence

import attr


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Config:
    """
    Attributes
    ----------
    image: str
        The name of the Docker image for the application.
    sources: Sequence[str]
        The sequence of source files that should be sourced to obtain the
        catkin workspace for the application.
    launches: Sequence[str]
        The sequence of launch files that should be used to launch the
        application.
    """
    image: str
    sources: Sequence[str]
    launches: Sequence[str]

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, 'sources', tuple(self.sources))
        object.__setattr__(self, 'launches', tuple(self.launches))
