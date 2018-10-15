import attr


class Declaration(object):
    pass


@attr.s(frozen=True)
class NodeInit(Declaration):
    name = attr.ib(type=str)
    package = attr.ib(type=str)
    defined_in_file = attr.ib(type=str)


@attr.s(frozen=True)
class ParamRead(Declaration):
    name = attr.ib(type=str)
    package = attr.ib(type=str)
    defined_in_file = attr.ib(type=str)
