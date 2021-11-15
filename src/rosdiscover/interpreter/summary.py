# -*- coding: utf-8 -*-
__all__ = ('NodeSummary', 'SystemSummary')

from typing import Any, Collection, Dict, Iterator, List, Mapping, Set, Tuple

import attr
from loguru import logger

from .provenance import Provenance
from ..core import Action, Service, Topic


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeSummary:
    """Summarises the architectural effects of a given node."""
    name: str
    fullname: str
    namespace: str
    kind: str
    package: str
    filename: str
    # Provenance indicates where the model comes from.
    #    PLACEHOLDER indicates whether the node was not really discovered, but
    #    was put in place to "complete" the architecture. Placeholder is set
    #    if the component template could not be found in the library, either
    #    because it is not a predefined model, or it's interactions were not
    #    discovered otherwise. Typically, placeholders will have no information
    #    about topics, services, etc.
    #    HANDWRITTEN indicates whether the node was derived from a handwritten
    #    model.
    #    RECOVERED indicates that the node was recovered through static analysis
    provenance: Provenance
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
    def _merge_collections(s1: Collection[Any], s2: Collection[Any]) -> Collection[Any]:
        s: Set[Any] = set()
        s.update(s1)
        s.update(s2)
        return s

    @classmethod
    def merge(cls, lhs: 'NodeSummary', rhs: 'NodeSummary') -> 'NodeSummary':

        reads = cls._merge_collections(lhs.reads, rhs.reads)
        writes = cls._merge_collections(lhs.writes, rhs.writes)
        pubs = cls._merge_collections(lhs.pubs, rhs.pubs)
        subs = cls._merge_collections(lhs.subs, rhs.subs)
        uses = cls._merge_collections(lhs.uses, rhs.uses)
        provides = cls._merge_collections(lhs.provides, rhs.provides)
        actions_servers = cls._merge_collections(lhs.action_servers, rhs.action_servers)
        action_clients = cls._merge_collections(lhs.action_clients, rhs.action_clients)

        if lhs.name != rhs.name and lhs.name:
            logger.warning(f"Merging two nodes that are named differently: {lhs.name} & {rhs.name}")
        name = lhs.name if lhs.name else rhs.name

        if lhs.package != rhs.package and lhs.package:
            logger.warning(f"{lhs.name} from package {rhs.package} retaining original package {lhs.package}")
        package = lhs.package if lhs.package else rhs.package

        if lhs.filename != rhs.filename and lhs.filename:
            logger.warning(f"{lhs.name} fullname {rhs.fullname} retaining original fullname {lhs.fullname}")
        fullname = lhs.fullname if lhs.fullname else rhs.fullname

        if lhs.namespace != rhs.namespace and lhs.namespace:
            logger.warning(f"{lhs.name} namespace {rhs.fullname} retaining original namespace {lhs.fullname}")
        namespace = lhs.namespace if lhs.namespace else rhs.namespace

        if lhs.kind != rhs.kind and lhs.kind:
            logger.warning(f"{lhs.name} namespace {rhs.kind} retaining original namespace {lhs.kind}")
        kind = lhs.kind if lhs.kind else rhs.kind

        if lhs.provenance != rhs.provenance:
            logger.warning(f"{lhs.name} provenance {rhs.provenance} retaining original provenance {lhs.provenance}")
        provenance = lhs.provenance

        if lhs.filename != rhs.filename:
            logger.warning(f"{lhs.name} filename {rhs.filename} retaining original filename {lhs.filename}")
        filename = lhs.filename if lhs.filename else rhs.filename

        return NodeSummary(
            name=name,
            fullname=fullname,
            namespace=namespace,
            kind=kind,
            package=package,
            filename=filename,
            provenance=provenance,
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
                'provenance': self.provenance.value,
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
        provenance = Provenance(dict.get('provenance', Provenance.PLACEHOLDER))
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
                           filename=filename,
                           provenance=provenance,
                           reads=reads,
                           writes=writes,
                           pubs=pubs,
                           subs=subs,
                           provides=provides,
                           uses=uses,
                           action_servers=action_servers,
                           action_clients=action_clients)


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeletSummary(NodeSummary):
    """Summarises the architectural effects of a given node."""
    nodelet_manager: str

    @classmethod
    def merge(cls, lhs: 'NodeletSummary', rhs: 'NodeletSummary') -> 'NodeletSummary':
        nc = NodeSummary.merge(lhs, rhs)

        if lhs.nodelet_manager != rhs.nodelet_manager:
            logger.warning(f"{lhs.fullname} nodelet manager {rhs.fullname} retaining original nodelet manager"
                           f" {lhs.fullname}")
        nodelet_manager = lhs.nodelet_manager if lhs.nodelet_manager else rhs.nodelet_manager
        return NodeletSummary(
            name=nc.name,
            fullname=nc.fullname,
            namespace=nc.namepsace,
            kind=nc.kind,
            package=nc.package,
            filename=nc.filename,
            provenance=nc.provenance,
            reads=nc.reads,
            writes=nc.writes,
            pubs=nc.pubs,
            subs=nc.subs,
            provides=nc.provides,
            uses=nc.uses,
            action_servers=nc.action_servers,
            action_clients=nc.action_clients,
            nodelet_manager=nodelet_manager
        )

    def to_dict(self) -> Dict[str, Any]:
        dict_ = super().to_dict()
        dict_['nodelet_manager'] = self.nodelet_manager.fullname
        dict_['nodekind'] = 'nodelet'
        return dict_

    @classmethod
    def from_dict(cls, dict_: Dict[str, Any]) -> 'NodeletSummary':
        nc = NodeSummary.from_dict(dict_)
        return NodeletSummary(
            name=nc.name,
            fullname=nc.fullname,
            namespace=nc.namepsace,
            kind=nc.kind,
            package=nc.package,
            filename=nc.filename,
            provenance=nc.provenance,
            reads=nc.reads,
            writes=nc.writes,
            pubs=nc.pubs,
            subs=nc.subs,
            provides=nc.provides,
            uses=nc.uses,
            action_servers=nc.action_servers,
            action_clients=nc.action_clients,
            nodelet_manager=dict_.get('nodelet_manager', '')
        )


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeletManagerSummary(NodeletSummary):
    nodelets: Collection[str]

    def __attrs_post_init__(self) -> None:
        super().__attrs_post_init__()
        object.__setattr__(self, 'nodelets', frozenset(self.writes))

    @classmethod
    def merge(cls, lhs: 'NodeletManagerSummary', rhs: 'NodeletManagerSummary') -> 'NodeletSummary':
        nc = NodeSummary.merge(lhs, rhs)
        nodelets = NodeSummary._merge_collections(lhs.nodelets, rhs.nodelets)
        return NodeletManagerSummary(
            name=nc.name,
            fullname=nc.fullname,
            namespace=nc.namepsace,
            kind=nc.kind,
            package=nc.package,
            filename=nc.filename,
            provenance=nc.provenance,
            reads=nc.reads,
            writes=nc.writes,
            pubs=nc.pubs,
            subs=nc.subs,
            provides=nc.provides,
            uses=nc.uses,
            action_servers=nc.action_servers,
            action_clients=nc.action_clients,
            nodelets=nodelets
        )

    @classmethod
    def from_dict(cls, dict_: Dict[str, Any]) -> 'NodeletManagerSummary':
        nc = NodeSummary.from_dict(dict_)
        return NodeletManagerSummary(
            name=nc.name,
            fullname=nc.fullname,
            namespace=nc.namepsace,
            kind=nc.kind,
            package=nc.package,
            filename=nc.filename,
            provenance=nc.provenance,
            reads=nc.reads,
            writes=nc.writes,
            pubs=nc.pubs,
            subs=nc.subs,
            provides=nc.provides,
            uses=nc.uses,
            action_servers=nc.action_servers,
            action_clients=nc.action_clients,
            nodelets=dict_.get('nodelets', [])
        )

    def to_dict(self) -> Dict[str, Any]:
        dict_ = NodeSummary.to_dict()
        dict_['nodekind'] = 'nodelet_manager'
        dict_['nodelets'] = list(self.nodelets)


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
        summaries = [NodeSummary.from_dict(s) for s in arr if 'nodekind' not in s]
        summaries.extend(
            NodeletManagerSummary.from_dict(s) for s in arr if s.get('nodekind', None) == 'nodelet_manager'
        )
        summaries.extend(
            NodeletSummary.from_dict(s) for s in arr if s.get('nodekind', None) == 'nodelet'
        )
        return SystemSummary(node_to_summary={summary.name: summary for summary in summaries})

    @property
    def unresolved(self) -> Iterator[NodeSummary]:
        for n in self._node_to_summary.values():
            if n.provenance == Provenance.PLACEHOLDER:
                yield n

    @classmethod
    def merge(cls, lhs: 'SystemSummary', rhs: 'SystemSummary') -> 'SystemSummary':
        node_summaries: Dict[str, NodeSummary] = {}
        for key, summary in lhs.items():
            if key not in rhs:
                node_summaries[key] = summary
            else:
                rhs = rhs[key]
                if isinstance(summary, NodeletSummary):
                    if isinstance(rhs, NodeletSummary):
                        assert isinstance(summary, NodeletSummary)
                        assert isinstance(rhs, NodeletSummary)
                        node_summaries[key] = NodeletSummary.merge(summary, rhs)
                    else:
                        logger.error(f"{rhs.fullname} is not a nodelet in rhs")
                        node_summaries[key] = summary

                elif isinstance(summary, NodeletManagerSummary):
                    if isinstance(summary, NodeletManagerSummary):
                        assert isinstance(summary, NodeletManagerSummary)
                        assert isinstance(rhs, NodeletManagerSummary)
                        node_summaries[key] = NodeletManagerSummary.merge(summary, rhs)
                    else:
                        logger.error(f"{rhs.fullname} is not a nodelet manager in rhs")
                        node_summaries[key] = summary
                else:
                    node_summaries[key] = NodeSummary.merge(summary, rhs)
        for key, summary in rhs.items():
            if key not in lhs:
                node_summaries[key] = summary
        return SystemSummary(node_to_summary=node_summaries)
