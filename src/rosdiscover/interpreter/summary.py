__all__ = ['NodeSummary']

from typing import FrozenSet, Tuple

import attr


@attr.s(frozen=True)
class NodeSummary(object):
    name = attr.ib(type=str)
    fullname = attr.ib(type=str)
    namespace = attr.ib(type=str)
    kind = attr.ib(type=str)
    package = attr.ib(type=str)
    pubs = attr.ib(type=FrozenSet[Tuple[str, str]],
                   converter=frozenset)
    subs = attr.ib(type=FrozenSet[Tuple[str, str]],
                   converter=frozenset)
    reads = attr.ib(type=FrozenSet[str],
                    converter=frozenset)
    writes = attr.ib(type=FrozenSet[str],
                    converter=frozenset)
    provides = attr.ib(type=FrozenSet[Tuple[str, str]],
                       converter=frozenset)

    def to_dict(self):
        # type: () -> Dict[str, Any]
        pubs = [{'name': str(n), 'format': str(f)} for (n, f) in self.pubs]
        subs = [{'name': str(n), 'format': str(f)} for (n, f) in self.subs]
        provides = \
            [{'name': str(n), 'format': str(f)} for (n, f) in self.provides]
        return {'name': str(self.name),
                'fullname': str(self.fullname),
                'namespace': str(self.namespace),
                'kind': str(self.kind),
                'package': str(self.package),
                'reads': list(self.reads),
                'writes': list(self.writes),
                'provides': provides,
                'pubs': pubs,
                'subs': subs}



