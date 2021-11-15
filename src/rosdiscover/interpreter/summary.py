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
    _pubs: Collection[Topic]
    _subs: Collection[Topic]
    # The tuple is (name, dynamic) where name is the name of the parameter
    # and dynamic is whether the node reacts to updates to the parameter via reconfigure
    _reads: Collection[Tuple[str, bool]]
    _writes: Collection[str]
    _uses: Collection[Service]
    _provides: Collection[Service]
    _action_servers: Collection[Action]
    _action_clients: Collection[Action]

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, '_pubs', frozenset(self._pubs))
        object.__setattr__(self, '_subs', frozenset(self._subs))
        object.__setattr__(self, '_reads', frozenset(self._reads))
        object.__setattr__(self, '_writes', frozenset(self._writes))
        object.__setattr__(self, '_uses', frozenset(self._uses))
        object.__setattr__(self, '_provides', frozenset(self._provides))
        object.__setattr__(self, '_action_servers', frozenset(self._action_servers))
        object.__setattr__(self, '_action_clients', frozenset(self._action_clients))

    @classmethod
    def _merge_collections(cls, s1: Collection[Any], s2: Collection[Any]) -> Collection[Any]:
        s: Set[Any] = set()
        s.update(s1)
        s.update(s2)
        return s

    @classmethod
    def merge(cls, lhs: 'NodeSummary', rhs: 'NodeSummary') -> 'NodeSummary':

        reads = cls._merge_collections(lhs._reads, rhs._reads)
        writes = cls._merge_collections(lhs._writes, rhs._writes)
        pubs = cls._merge_collections(lhs._pubs, rhs._pubs)
        subs = cls._merge_collections(lhs._subs, rhs._subs)
        uses = cls._merge_collections(lhs._uses, rhs._uses)
        provides = cls._merge_collections(lhs._provides, rhs._provides)
        actions_servers = cls._merge_collections(lhs._action_servers, rhs._action_servers)
        action_clients = cls._merge_collections(lhs._action_clients, rhs._action_clients)

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
        pubs = [t.to_dict() for t in self._pubs]
        subs = [t.to_dict() for t in self._subs]
        provides = [s.to_dict() for s in self._provides]
        uses = [s.to_dict() for s in self._uses]
        action_servers = [a.to_dict() for a in self._action_servers]
        action_clients = [a.to_dict() for a in self._action_clients]
        reads = [{'name': n, 'dynamic': d} for (n, d) in self._reads]
        return {'name': self.name,
                'fullname': self.fullname,
                'namespace': self.namespace,
                'kind': self.kind,
                'package': self.package,
                'filename': self.filename,
                'provenance': self.provenance.value,
                'reads': reads,
                'writes': list(self._writes),
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

    @property
    def pubs(self) -> Collection[Topic]:
        return self._pubs

    @property
    def subs(self) -> Collection[Topic]:
        return self._subs

    @property
    def provides(self) -> Collection[Service]:
        return self._provides

    @property
    def uses(self) -> Collection[Service]:
        return self._uses

    @property
    def reads(self) -> Collection[Tuple[str, bool]]:
        return self._reads

    @property
    def writes(self) -> Collection[str]:
        return self._writes

    @property
    def action_clients(self) -> Collection[Action]:
        return self._action_clients

    @property
    def action_servers(self) -> Collection[Action]:
        return self._action_servers


MERGE_NODELETS_TO_NODELET_MANAGERS = True


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeletSummary(NodeSummary):
    """Summarises the architectural effects of a given node."""
    nodelet_manager: str

    @classmethod
    def merge(cls, lhs: 'NodeSummary', rhs: 'NodeSummary') -> 'NodeletSummary':
        assert isinstance(lhs, NodeletSummary)
        assert isinstance(rhs, NodeletSummary)
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

    @property
    def pubs(self) -> Collection[Topic]:
        return self._pubs if not MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def subs(self) -> Collection[Topic]:
        return self._subs if not MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def provides(self) -> Collection[Service]:
        return self._provides if not MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def uses(self) -> Collection[Service]:
        return self._uses if not MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def reads(self) -> Collection[Tuple[str, bool]]:
        return self._reads if not MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def writes(self) -> Collection[str]:
        return self._writes if not MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def action_clients(self) -> Collection[Action]:
        return self._action_clients if not MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def action_servers(self) -> Collection[Action]:
        return self._action_servers if not MERGE_NODELETS_TO_NODELET_MANAGERS else {}


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeletManagerSummary(NodeSummary):
    nodelets: Collection[NodeletSummary]

    def __attrs_post_init__(self) -> None:
        super().__attrs_post_init__()
        object.__setattr__(self, 'nodelets', frozenset(self.writes))

    @classmethod
    def merge(cls, lhs: 'NodeSummary', rhs: 'NodeSummary') -> 'NodeletManagerSummary':
        assert isinstance(lhs, NodeletManagerSummary)
        assert isinstance(rhs, NodeletManagerSummary)
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
            nodelets=list(NodeletSummary.from_dict(n) for n in dict_.get('nodelets', []))
        )

    def to_dict(self) -> Dict[str, Any]:
        dict_ = NodeSummary.to_dict()
        dict_['nodekind'] = 'nodelet_manager'
        dict_['nodelets'] = list(n.to_dict() for n in self.nodelets)

    @property
    def pubs(self) -> Collection[Topic]:
        return list(p for n in self.nodelets for p in n.pubs) if MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def subs(self) -> Collection[Topic]:
        return list(p for n in self.nodelets for p in n.subs) if MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def provides(self) -> Collection[Service]:
        return list(p for n in self.nodelets for p in n.provides) if MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def uses(self) -> Collection[Service]:
        return list(p for n in self.nodelets for p in n.uses) if MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def reads(self) -> Collection[Tuple[str, bool]]:
        return list(p for n in self.nodelets for p in n.reads) if MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def writes(self) -> Collection[str]:
        return list(p for n in self.nodelets for p in n.writes) if MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def action_clients(self) -> Collection[Action]:
        return list(p for n in self.nodelets for p in n.action_clients) if MERGE_NODELETS_TO_NODELET_MANAGERS else {}

    @property
    def action_servers(self) -> Collection[Action]:
        return list(p for n in self.nodelets for p in n.action_servers) if MERGE_NODELETS_TO_NODELET_MANAGERS else {}


@attr.s(frozen=True, slots=True, auto_attribs=True)
class SystemSummary(Mapping[str, NodeSummary]):
    """Summarises the architectural effects of all nodes in a system.
    Provides a mapping from node names to the architectural effects of those
    nodes."""
    _node_to_summary: Mapping[str, NodeSummary]

    def __len__(self) -> int:
        length = len(self._node_to_summary)
        # Because nodelet managers contain nodelets include them too
        for n in self._node_to_summary.values():
            if isinstance(n, NodeletManagerSummary):
                assert isinstance(n, NodeletManagerSummary)
                length += len(n.nodelets)
        return length

    def __iter__(self) -> Iterator[str]:
        # Because nodelet managers contain nodelets include them too
        for key, item in self._node_to_summary.items():
            yield key
            if isinstance(item, NodeletManagerSummary):
                assert isinstance(item, NodeletManagerSummary)
                for nodelet in item.nodelets:
                    yield nodelet.fullname

    def __getitem__(self, name: str) -> NodeSummary:
        item = self._node_to_summary.get(name, None)
        # Because nodelet managers contain nodelets include them too
        if not item:
            items = [n.fullname for nm in self._node_to_summary.values() if isinstance(nm, NodeletManagerSummary) for
                     n in nm.nodelets]
            if len(items) == 1:
                item = items[0]
            else:
                raise KeyError(f'More than one nodelet with name {name}')
        if item:
            return item
        raise KeyError(f"'{name}' not found in summary")

    def to_dict(self) -> List[Dict[str, Any]]:
        return [n.to_dict() for n in self.values()]

    @classmethod
    def from_dict(cls, arr: Collection[Any]) -> 'SystemSummary':
        summaries = [NodeSummary.from_dict(s) for s in arr if 'nodekind' not in s]
        summaries.extend(
            NodeletManagerSummary.from_dict(s) for s in arr if s.get('nodekind', None) == 'nodelet_manager'
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
                # rhs = rhs[key]
                # if isinstance(summary, NodeletSummary):
                #     if isinstance(rhs, NodeletSummary):
                #         assert isinstance(summary, NodeletSummary)
                #         assert isinstance(rhs, NodeletSummary)
                #         node_summaries[key] = NodeletSummary.merge(summary, rhs)
                #     else:
                #         logger.error(f"{rhs.fullname} is not a nodelet in rhs")
                #         node_summaries[key] = summary

                if isinstance(summary, NodeletManagerSummary):
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
