# -*- coding: utf-8 -*-
__all__ = ('NodeSummary',)

from typing import Any, Collection, Dict, Tuple

import attr


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeSummary:
    name: str
    fullname: str
    namespace: str
    kind: str
    package: str
    nodelet: bool
    pubs: Collection[Tuple[str, str]]
    subs: Collection[Tuple[str, str]]
    # The tuple is (name, dynamic) where name is the name of the parameter
    # and dynamic is whether the node reacts to updates to the parameter via reconfigure
    reads: Collection[Tuple[str, bool]]
    writes: Collection[str]
    uses: Collection[Tuple[str, str]]
    provides: Collection[Tuple[str, str]]
    action_servers: Collection[Tuple[str, str]]
    action_clients: Collection[Tuple[str, str]]

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, 'pubs', frozenset(self.pubs))
        object.__setattr__(self, 'subs', frozenset(self.subs))
        object.__setattr__(self, 'reads', frozenset(self.reads))
        object.__setattr__(self, 'writes', frozenset(self.writes))
        object.__setattr__(self, 'uses', frozenset(self.uses))
        object.__setattr__(self, 'provides', frozenset(self.provides))
        object.__setattr__(self, 'action_servers', frozenset(self.action_servers))
        object.__setattr__(self, 'action_clients', frozenset(self.action_clients))

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
