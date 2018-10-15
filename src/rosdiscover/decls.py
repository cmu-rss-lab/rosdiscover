from typing import FrozenSet

import attr


class Declaration(object):
    pass


@attr.s(frozen=True)
class NodeInit(Declaration):
    name = attr.ib(type=str)
    defined_in_file = attr.ib(type=str)


@attr.s(frozen=True)
class ParamRead(Declaration):
    name = attr.ib(type=str)
    defined_in_file = attr.ib(type=str)


@attr.s(frozen=True)
class FileDeclarations(object):
    """
    Holds all declarations contained in a given file.
    """
    filename = attr.ib(type=str)
    node_inits = attr.ib(type=FrozenSet[NodeInit], converter=frozenset)
    param_reads = attr.ib(type=FrozenSet[NodeInit], converter=frozenset)
