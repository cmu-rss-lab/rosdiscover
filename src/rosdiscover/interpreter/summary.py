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
    nodelet = attr.ib(type=bool)
    pubs = attr.ib(type=FrozenSet[Tuple[str, str]],
                   converter=frozenset)
    subs = attr.ib(type=FrozenSet[Tuple[str, str]],
                   converter=frozenset)
    reads = attr.ib(type=FrozenSet[Tuple[str, bool]],
                    converter=frozenset)
    writes = attr.ib(type=FrozenSet[str],
                    converter=frozenset)
    uses = attr.ib(type=FrozenSet[Tuple[str, str]],
                   converter=frozenset)
    provides = attr.ib(type=FrozenSet[Tuple[str, str]],
                       converter=frozenset)
    action_servers = attr.ib(type=FrozenSet[Tuple[str, str]],
                             converter=frozenset)
    action_clients = attr.ib(type=FrozenSet[Tuple[str, str]],
                             converter=frozenset)

    def to_dict(self):
        # type: () -> Dict[str, Any]
        pubs = [{'name': str(n), 'format': str(f)} for (n, f) in self.pubs]
        subs = [{'name': str(n), 'format': str(f)} for (n, f) in self.subs]
        provides = \
            [{'name': str(n), 'format': str(f)} for (n, f) in self.provides]
        uses = \
            [{'name': str(n), 'format': str(f)} for (n, f) in self.uses]
        action_servers = [{'name': str(n), 'format': str(f)}
                          for (n, f) in self.action_servers]
        action_clients = [{'name': str(n), 'format': str(f)}
                          for (n, f) in self.action_clients]
        reads = [{'name' : str(n), 'dynamic' : bool(d)} for (n, d) in self.reads]
        return {'name': str(self.name),
                'fullname': str(self.fullname),
                'namespace': str(self.namespace),
                'kind': str(self.kind),
                'package': str(self.package),
                'nodelet': bool(self.nodelet),
                'reads': reads,
                'writes': list(self.writes),
                'provides': provides,
                'uses': uses,
                'action-servers': action_servers,
                'action-clients': action_clients,
                'pubs': pubs,
                'subs': subs}
