# -*- coding: utf-8 -*-
__all__ = ('NodeSummary', 'SystemSummary')

from typing import Any, Collection, Dict, Iterator, List, Mapping, Tuple

import attr

from ..core import Action, Service, Topic


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeSummary:
    """Summarises the architectural effects of a given node."""
    name: str
    fullname: str
    namespace: str
    kind: str
    package: str
    nodelet: bool
    filename: str
    # placeholder indicates whether the node was not really discovered, but
    # was put in place to "complete" the architecture. Placeholder is set
    # if the component template could not be found in the library, either
    # because it is not a predefined model, or it's interactions were not
    # discovered otherwise. Typically, placeholders will have no information
    # about topics, services, etc.
    placeholder: bool
    pubs: Collection[Topic]
    subs: Collection[Topic]
    # The tuple is (name, dynamic) where name is the name of the parameter
    # and dynamic is whether the node reacts to updates to the parameter via reconfigure
    reads: Collection[Tuple[str, bool]]
    writes: Collection[str]
    uses: Collection[Service]
    provides: Collection[Service]
    action_servers: Collection[Action]
    action_clients: Collection[Action]

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
        pubs = [t.to_dict() for t in self.pubs]
        subs = [t.to_dict() for t in self.subs]
        provides = [s.to_dict() for s in self.provides]
        uses = [s.to_dict() for s in self.uses]
        action_servers = [a.to_dict() for a in self.action_servers]
        action_clients = [a.to_dict() for a in self.action_clients]
        reads = [{'name': n, 'dynamic': d} for (n, d) in self.reads]
        return {'name': self.name,
                'fullname': self.fullname,
                'namespace': self.namespace,
                'kind': self.kind,
                'package': self.package,
                'nodelet': self.nodelet,
                'filename': self.filename,
                'placeholder': self.placeholder,
                'reads': reads,
                'writes': list(self.writes),
                'provides': provides,
                'uses': uses,
                'action-servers': action_servers,
                'action-clients': action_clients,
                'pubs': pubs,
                'subs': subs}

    @classmethod
    def from_dict(cls, dict: Dict[str, Any]) -> 'NodeSummary':
        name = dict["name"]
        fullname = dict.get('fullname', name)
        namepsace = dict.get('namespace', '')
        kind = dict.get('kind', '')
        package = dict.get('package', '')
        nodelet = dict.get('nodelet', False)
        filename = dict.get('filename', '')
        placeholder = dict.get('placeholder', False)
        reads = [(p['name'], p['implicit']) for p in dict.get('reads', [])]
        writes = dict.get('writes', [])
        pubs = [Topic(name=t['name'], format=t['format'], implicit=t.get('implicit', False))
                for t in dict.get('pubs', [])]
        subs = [Topic(name=t['name'], format=t['format'], implicit=t.get('implicit', False))
                for t in dict.get('subs', [])]
        provides = [Service(name=s['name'], format=s['format'])
                    for s in dict.get('provides', [])]
        uses = [Service(name=s['name'], format=s['format'])
                for s in dict.get('uses', [])]
        action_servers = [Action(name=a['name'], format=a['format'])
                          for a in dict.get('action-servers', [])]
        action_clients = [Action(name=a['name'], format=a['format'])
                          for a in dict.get('action-clients', [])]
        return NodeSummary(name=name,
                           fullname=fullname,
                           namepsace=namepsace,
                           kind=kind,
                           package=package,
                           nodelet=nodelet,
                           filename=filename,
                           placeholder=placeholder,
                           reads=reads,
                           writes=writes,
                           pubs=pubs,
                           subs=subs,
                           provides=provides,
                           uses=uses,
                           action_servers=action_servers,
                           action_clients=action_clients)


@attr.s(frozen=True, slots=True, auto_attribs=True)
class SystemSummary(Mapping[str, NodeSummary]):
    """Summarises the architectural effects of all nodes in a system.
    Provides a mapping from node names to the architectural effects of those
    nodes."""
    _node_to_summary: Mapping[str, NodeSummary]

    def __len__(self) -> int:
        return len(self._node_to_summary)

    def __iter__(self) -> Iterator[str]:
        yield from self._node_to_summary

    def __getitem__(self, name: str) -> NodeSummary:
        return self._node_to_summary[name]

    def to_dict(self) -> List[Dict[str, Any]]:
        return [n.to_dict() for n in self.values()]

    @classmethod
    def from_dict(cls, arr: Collection[Any]) -> 'SystemSummary':
        summaries = [NodeSummary.from_dict(s) for s in arr]
        return SystemSummary(node_to_summary={summary.name: summary for summary in summaries})

    @property
    def unresolved(self) -> Iterator[NodeSummary]:
        for n in self._node_to_summary.values():
            if n.placeholder:
                yield n

