# -*- coding: utf-8 -*-
__all__ = ('NodeSummary',)

from typing import Any, Dict, FrozenSet, Tuple

import attr


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeSummary:
    name: str
    fullname: str
    namespace: str
    kind: str
    package: str
    nodelet: bool
    pubs: FrozenSet[Tuple[str, str]] = attr.ib(converter=frozenset)
    subs: FrozenSet[Tuple[str, str]]  = attr.ib(converter=frozenset)
    # The tuple is (name, dynamic) where name is the name of the parameter
    # and dynamic is whether the node reacts to updates to the parameter via reconfigure
    reads: FrozenSet[Tuple[str, bool]] = attr.ib(converter=frozenset)
    writes: FrozenSet[str] = attr.ib(converter=frozenset)
    uses: FrozenSet[Tuple[str, str]] = attr.ib(converter=frozenset)
    provides: FrozenSet[Tuple[str, str]] = attr.ib(converter=frozenset)
    action_servers: FrozenSet[Tuple[str, str]] = attr.ib(converter=frozenset)
    action_clients: FrozenSet[Tuple[str, str]] = attr.ib(converter=frozenset)

    def to_dict(self) -> Dict[str, Any]:
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
        reads = [{'name' : n, 'dynamic' : d } for (n, d) in self.reads]
        return {'name': self.name,
                'fullname': self.fullname,
                'namespace': self.namespace,
                'kind': self.kind,
                'package': self.package,
                'nodelet': self.nodelet,
                'reads': reads,
                'writes': list(self.writes),
                'provides': provides,
                'uses': uses,
                'action-servers': action_servers,
                'action-clients': action_clients,
                'pubs': pubs,
                'subs': subs}
