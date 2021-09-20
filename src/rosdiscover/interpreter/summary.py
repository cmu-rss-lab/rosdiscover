# -*- coding: utf-8 -*-
__all__ = ('NodeSummary', 'SystemSummary')

from typing import Any, Collection, Dict, Iterator, List, Mapping, Set, Tuple

import attr
from loguru import logger

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

    @classmethod
    def merge(cls, summary1: 'NodeSummary', summary2: 'NodeSummary') -> 'NodeSummary':
        def merge_collections(s1: Collection[Any], s2: Collection[Any]) -> Collection[Any]:
            s: Set[Any] = set()
            s.update(s1)
            s.update(s2)
            return s

        reads = merge_collections(summary1.reads, summary2.reads)
        writes = merge_collections(summary1.writes, summary2.writes)
        pubs = merge_collections(summary1.pubs, summary2.pubs)
        subs = merge_collections(summary1.subs, summary2.subs)
        uses = merge_collections(summary1.uses, summary2.uses)
        provides = merge_collections(summary1.provides, summary2.provides)
        actions_servers = merge_collections(summary1.action_servers, summary2.action_servers)
        action_clients = merge_collections(summary1.action_clients, summary2.action_clients)

        if summary1.name != summary2.name and summary1.name:
            logger.warning(f"Merging two nodes that are named differently: {summary1.name} & {summary2.name}")
        name = summary1.name if summary1.name else summary2.name

        if summary1.package != summary2.package and summary1.package:
            logger.warning(f"{summary1.name} from package {summary2.package} retaining original package {summary1.package}")
        package = summary1.package if summary1.package else summary2.package

        if summary1.filename != summary2.filename and summary1.filename:
            logger.warning(f"{summary1.name} fullname {summary2.fullname} retaining original fullname {summary1.fullname}")
        fullname = summary1.fullname if summary1.fullname else summary2.fullname

        if summary1.namespace != summary2.namespace and summary1.namespace:
            logger.warning(f"{summary1.name} namespace {summary2.fullname} retaining original namespace {summary1.fullname}")
        namespace = summary1.namespace if summary1.namespace else summary2.namespace

        if summary1.kind != summary2.kind and summary1.kind:
            logger.warning(f"{summary1.name} namespace {summary2.kind} retaining original namespace {summary1.kind}")
        kind = summary1.kind if summary1.kind else summary2.kind

        if summary1.placeholder != summary2.placeholder:
            logger.warning(f"{summary1.name} placeholder {summary2.placeholder} retaining original placeholder {summary1.placeholder}")
        placeholder = summary1.placeholder or summary2.placeholder

        if summary1.filename != summary2.filename:
            logger.warning(f"{summary1.name} filename {summary2.filename} retaining original filename {summary1.filename}")
        filename = summary1.filename if summary1.filename else summary2.filename

        return NodeSummary(
            name=name,
            fullname=fullname,
            namespace=namespace,
            kind=kind,
            package=package,
            nodelet=summary1.nodelet,
            filename=filename,
            placeholder=placeholder,
            reads=reads,
            writes=writes,
            provides=provides,
            uses=uses,
            action_servers=actions_servers,
            action_clients=action_clients,
            pubs=pubs,
            subs=subs,
        )

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
        reads = [(p['name'], p['dynamic']) for p in dict.get('reads', [])]
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
                           namespace=namepsace,
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

    @classmethod
    def merge(cls, sum1: 'SystemSummary', sum2: 'SystemSummary') -> 'SystemSummary':
        node_summaries = {}
        for key, summary in sum1.items():
            if key not in sum2:
                node_summaries[key] = summary
            else:
                node_summaries[key] = NodeSummary.merge(summary, sum2[key])
        for key, summary in sum2.items():
            if key not in sum1:
                node_summaries[key] = summary
        return SystemSummary(node_to_summary=node_summaries)
