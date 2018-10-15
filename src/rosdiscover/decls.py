import attr


@attr.s(frozen=True)
class NodeInit(object):
    name = attr.ib(type=str)
    package = attr.ib(type=str)
    defined_in_file = attr.ib(type=str)


@attr.s(frozen=True)
class ParamRead(object):
    name = attr.ib(type=str)
    package = attr.ib(type=str)
    defined_in_file = attr.ib(type=str)
