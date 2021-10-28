# -*- coding: utf-8 -*-
"""
This module is used to generate an Acme description from a set of nodes parsed
from a launch file.

The main class provided by this module is :class:`AcmeGenerator`
"""
__all__ = ("AcmeGenerator", )
import json
import os
import subprocess
import tempfile
from subprocess import PIPE
from typing import Any, Collection, Dict, Iterable, List, Optional, Sequence, Set, Tuple

import attr
from loguru import logger

from ..core import Action, Service, Topic
from ..interpreter import NodeSummary

# Constants for Acme generation
from ..interpreter.context import Provenance

TOPIC_CONNECTOR = """   connector {conn_name} : TopicConnectorT = new TopicConnectorT extended with {{
    {roles}
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """
SERVICE_CONNECTOR = """  connector {conn_name} : ServiceConnT = new ServiceConnT extended with {{
    {roles}
    }};
   """
ACTION_CONNECTOR = """  connector {conn_name} : ActionServerConnT = new ActionServerConnT extended with {{
    {roles}
    }};
   """
NODE_COMPONENT = """   component {comp_name} : ROSNodeCompT = new ROSNodeCompT extended with {{
        {ports}
        property name = "{node_name}";
        property launchedBy = "{filename}";
    }};
    """
NODE_PLACEHOLDER_COMPONENT = """   component {comp_name} : ROSNodeCompT, PlaceholderT = new ROSNodeCompT, PlaceholderT extended with {{
        {ports}
        property name = "{node_name}";
        property placeholder = true;
        property launchedBy = "{filename}";
    }};
"""
ATTACHMENT = "  attachment {comp}.{port} to {conn}.{role};"
SERVICE_ATTACHMENT = "  attachment {qualified_port} to {conn}.{role};"
SUBSCRIBER_ROLE = """    role {role_name} : ROSTopicSubscriberRoleT = new ROSTopicSubscriberRoleT;
    """
ADVERTISER_ROLE = """     role {role_name} : ROSTopicAdvertiserRoleT = new ROSTopicAdvertiserRoleT;
    """
PROVIDER_ROLE = """     role {role_name} : ROSServiceResponderRoleT = new ROSServiceResponderRoleT;
    """
CLIENT_ROLE = """       role {role_name} : ROSServiceCallerRoleT = new ROSServiceCallerRoleT;
    """
ACTION_CLIENT_ROLE = """      role {role_name} : ROSActionCallerRoleT = new ROSActionCallerRoleT;
    """
ACTION_SERVER_ROLE = """      role {role_name} : ROSActionResponderRoleT = new ROSActionResponderRoleT;
    """
TOPIC_PORT = """     port {port_name} : ${port_type} = new ${port_type} extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """
SUBSCRIBER_PORT = """     port {port_name} : TopicSubscribePortT = new TopicSubscribePortT extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """
ADVERTISER_PORT = """     port {port_name} : TopicAdvertisePortT = new TopicAdvertisePortT extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """
PROVIDER_PORT = """     port {port_name} : ServiceProviderPortT = new ServiceProviderPortT extended with {{
        property svc_type : string = "{svc_type}";
        property name : string = "{service}";
        property args : string = "";
    }};
    """
REQUIRER_PORT = """     port {port_name} : ServiceClientPortT = new ServiceClientPortT extended with {{
        property name : string = "{service}";
        property svc_type : string = "{svc_type}";
        property persistency : ServicePersistencyT = {persistence};
    }};
    """
ACTION_CLIENT_PORT = """    port {port_name} : ActionClientPortT = new ActionClientPortT extended with {{
        property action_type : string = "{action_type}";
    }};
    """
ACTION_SERVER_PORT = """    port {port_name}: ActionServerPortT = new ActionServerPortT extended with {{
        property action_type : string = "{action_type}";
    }};
    """


def update_service_conn(conns, service, port_qualified, is_provider) -> None:
    if service in conns:
        s = conns[service]
    else:
        s = {"name": service, 'callers': set(), 'providers': set()}
        conns[service] = s
    if is_provider:
        s['providers'].add(port_qualified)
    else:
        s['callers'].add(port_qualified)


def update_action_conn(conns, action, port_qualified, is_server) -> None:
    a = {}
    if action in conns:
        a = conns[action]
    else:
        a = {'name': action, "clients": set(), 'servers': set()}
        conns[action] = a
    if is_server:
        a['servers'].add(port_qualified)
    else:
        a['clients'].add(port_qualified)


@attr.s(slots=True, auto_attribs=True)
class _TopicInformation:
    details: Topic
    pubs: Set[str] = attr.ib(factory=set)
    subs: Set[str] = attr.ib(factory=set)


@attr.s(slots=True, auto_attribs=True)
class _ServiceInformation:
    details: Service
    provs: Set[str] = attr.ib(factory=set)
    reqs: Set[str] = attr.ib(factory=set)


@attr.s(slots=True, auto_attribs=True)
class _ActionInformation:
    details: Action
    servers: Set[str] = attr.ib(factory=set)
    clients: Set[str] = attr.ib(factory=set)


class AcmeGenerator:
    def __init__(self,
                 nodes: Iterable[NodeSummary],
                 acme_file: str,
                 jar: Optional[str],
                 things_to_ignore: Optional[Collection[str]],
                 ) -> None:
        self.__nodes: Sequence[NodeSummary] = list(nodes)
        self.__acme_file = acme_file
        self.__generate_dangling_connectors = False
        self.__acme_jar = jar if jar is not None else 'lib/acme.standalone-ros.jar'
        self.__to_ignore = things_to_ignore if things_to_ignore is not None else []

    def get_components_and_connectors(self) \
            -> Tuple[List[NodeSummary],
                     Dict[str, _TopicInformation],
                     Dict[str, _ServiceInformation],
                     Dict[str, _ActionInformation]]:
        components: List[NodeSummary] = []
        topics: Dict[str, _TopicInformation] = {}
        services: Dict[str, _ServiceInformation] = {}
        actions: Dict[str, _ActionInformation] = {}

        for node in self.__nodes:
            for pub in [t for t in node.pubs if
                        not t.implicit and not self._ignore(t.name)]:
                if pub.name in topics:
                    topic = topics[pub.name]
                else:
                    topic = _TopicInformation(details=pub)
                    topics[pub.name] = topic
                topic.pubs.add(node.name)
            for sub in [t for t in node.subs if
                        not t.implicit and t.name not in self.__to_ignore]:
                if sub.name in topics:
                    topic = topics[sub.name]
                else:
                    topic = _TopicInformation(details=sub)
                    topics[sub.name] = topic
                topic.subs.add(node.name)
            for service_object in [s for s in node.provides if not self._ignore(s.name)]:
                service_name = service_object.name
                if service_name in services:
                    service = services[service_name]
                else:
                    service = _ServiceInformation(details=service_object)
                    services[service_name] = service
                service.provs.add(node.name)
            for call in [s for s in node.uses if not self._ignore(s.name)]:
                if call.name in services:
                    service = services[call.name]
                else:
                    service = _ServiceInformation(details=call)
                    services[call.name] = service
                service.reqs.add(node.name)
            for action_object in [a for a in node.action_servers if
                                  not self._ignore(a.name)]:
                action_name = action_object.name
                if action_name in actions:
                    action = actions[action_name]
                else:
                    action = _ActionInformation(details=action_object)
                    actions[action_name] = action
                action.servers.add(node.name)
            for action_object in [a for a in node.action_clients if
                                  not self._ignore(a.name)]:
                action_name = action_object.name
                if action_name in actions:
                    action = actions[action_name]
                else:
                    action = _ActionInformation(details=action_object)
                    actions[action_name] = action
                action.clients.add(node.name)
            components.append(node)
        return components, topics, services, actions

    def _ignore(self, name: str) -> bool:
        ignore: bool = False
        for i in self.__to_ignore:
            if i.startswith('*') and name.endswith(i[1:]):
                ignore = True
                break
            elif i == name:
                ignore = True
                break
        return ignore

    @staticmethod
    def to_acme_name(name: str) -> str:
        return name.replace("/", "_").replace('.', "_")

    def generate_acme(self) -> str:
        components, topics, services, actions = \
            self.get_components_and_connectors()
        topics = {key: info for key, info in topics.items() if not self._ignore(key)}
        services = {key: info for key, info in services.items() if not self._ignore(key)}
        actions = {key: info for key, info in actions.items() if not self._ignore(key)}
        system_name = "RobotSystem" if self.__acme_file is None else self.to_acme_name(
            '_'.join(self.__acme_file.split(".")[:-1])
        )
        # system_name = os.path.basename(os.path.normpath(self.__launch_files)).split('.')[0]

        acme = f"import families/ROSFam.acme;\nsystem {system_name} : ROSFam = new ROSFam extended with {{\n"
        attachments: List[str] = []
        component_strs: List[str] = []
        service_conns: Dict[str, dict] = {}
        action_conns: Dict[str, dict] = {}
        attachments_to_topic: Dict[str, List[str]] = {}
        for c in components:
            ports = []
            comp_name = self.to_acme_name(c.name)

            for pub in [t for t in c.pubs if not t.implicit and not self._ignore(t.name)]:
                if pub.name not in attachments_to_topic:
                    attachments_to_topic[pub.name] = []
                pname = f'{self.to_acme_name(pub.name)}_pub'
                port = ADVERTISER_PORT.format(port_name=pname, msg_type=pub.format, topic=pub.name)
                ports.append(port)
                attachments_to_topic[pub.name].append(
                    ATTACHMENT.format(comp=comp_name,
                                      port=pname,
                                      conn=f"{AcmeGenerator.to_acme_name(pub.name)}_conn",
                                      role=f"{comp_name}_pub"))
            for subscriber in [t for t in c.subs if not t.implicit and not self._ignore(t.name)]:
                if subscriber.name not in attachments_to_topic:
                    attachments_to_topic[subscriber.name] = []
                pname = f"{AcmeGenerator.to_acme_name(subscriber.name)}_sub"
                port = SUBSCRIBER_PORT.format(port_name=pname, msg_type=subscriber.format,
                                              topic=subscriber.name)
                ports.append(port)
                attachments_to_topic[subscriber.name].append(
                    ATTACHMENT.format(comp=comp_name,
                                      port=pname,
                                      conn=f"{self.to_acme_name(subscriber.name)}_conn",
                                      role=f"{comp_name}_sub"))
            for provider in [s for s in c.provides if not self._ignore(s.name)]:
                name = provider.name
                fmt = provider.format
                pname = f"{AcmeGenerator.to_acme_name(name)}_svc"
                port = PROVIDER_PORT.format(port_name=pname, svc_type=fmt, service=name)
                ports.append(port)
                update_service_conn(service_conns, name, f"{comp_name}.{pname}", True)

            for caller in [s for s in c.uses if not self._ignore(s.name)]:
                service_name = caller.name
                pname = f"{AcmeGenerator.to_acme_name(service_name)}_call"
                fmt = caller.format
                port = REQUIRER_PORT.format(port_name=pname, svc_type=fmt,
                                            service=service_name, persistence="non-persistent")
                ports.append(port)
                update_service_conn(service_conns, service_name,
                                    f"{comp_name}.{pname}",
                                    False)

            for action in [a for a in c.action_servers if not self._ignore(a.name)]:
                name = action.name
                fmt = action.format
                pname = f"{AcmeGenerator.to_acme_name(name)}_srvr"
                port = ACTION_SERVER_PORT.format(port_name=pname,
                                                 action_type=fmt)
                ports.append(port)
                update_action_conn(action_conns,
                                   name,
                                   f"{comp_name}.{pname}",
                                   True)

            for action in [a for a in c.action_clients if not self._ignore(a.name)]:
                name = action.name
                fmt = action.format
                pname = f"{AcmeGenerator.to_acme_name(name)}_cli"
                port = ACTION_CLIENT_PORT.format(port_name=pname,
                                                 action_type=fmt)
                ports.append(port)
                update_action_conn(action_conns,
                                   name,
                                   f"{comp_name}.{pname}",
                                   False)
            component_template: str = NODE_COMPONENT \
                if c.provenance != Provenance.PLACEHOLDER else NODE_PLACEHOLDER_COMPONENT
            comp = component_template.format(comp_name=comp_name,
                                             ports='\n'.join(ports),
                                             node_name=c.name,
                                             filename=c.filename)
            component_strs.append(comp)

        acme = acme + "\n".join(component_strs)

        connector_strs: List[str] = []
        self._process_topics(topics, connector_strs, attachments, attachments_to_topic)
        self._process_services(service_conns, attachments, connector_strs)
        self._process_actions(action_conns, attachments, connector_strs)

        acme = acme + "\n".join(connector_strs)
        acme = acme + "\n".join(attachments) + "}"
        self.generate_acme_file(acme)
        return acme

    def _process_actions(self,
                         action_conns: Dict[str, Any],
                         attachments: List[str],
                         connector_strs: List[str]) -> None:
        for a in action_conns.keys():
            # only create a connector for actions that are connected
            has_actions_srvrs = len(action_conns[a]['servers']) != 0
            has_action_clnts = len(action_conns[a]['clients']) != 0
            if self.__generate_dangling_connectors or (has_actions_srvrs and has_action_clnts):
                roles = []
                cname = f"{AcmeGenerator.to_acme_name(a)}_conn"
                for cl in action_conns[a]['clients']:
                    rname = AcmeGenerator.to_acme_name(cl)
                    role = ACTION_CLIENT_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(
                        SERVICE_ATTACHMENT.format(qualified_port=cl, conn=cname, role=rname))
                for sub in action_conns[a]['servers']:
                    rname = AcmeGenerator.to_acme_name(sub)
                    role = ACTION_SERVER_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(
                        SERVICE_ATTACHMENT.format(qualified_port=sub, conn=cname, role=rname))
                connector_strs.append(
                    ACTION_CONNECTOR.format(conn_name=cname, roles="\n".join(roles)))

    def _process_services(self,
                          service_conns: Dict[str, Any],
                          attachments: List[str],
                          connector_strs: List[str]) -> None:
        for sub in service_conns.keys():
            # Only create a connector for services that are connected
            has_providers = len(service_conns[sub]['providers']) != 0
            has_callers = len(service_conns[sub]['callers']) != 0
            if self.__generate_dangling_connectors or (has_providers and has_callers):
                roles = []
                cname = f"{AcmeGenerator.to_acme_name(sub)}_conn"
                for p in service_conns[sub]['providers']:
                    rname = AcmeGenerator.to_acme_name(p)
                    role = PROVIDER_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(
                        SERVICE_ATTACHMENT.format(qualified_port=p, conn=cname, role=rname))
                for p in service_conns[sub]['callers']:
                    rname = AcmeGenerator.to_acme_name(p)
                    role = CLIENT_ROLE.format(role_name=rname)
                    roles.append(f"{role}\n")
                    attachments.append(SERVICE_ATTACHMENT.format(qualified_port=p,
                                                                 conn=cname,
                                                                 role=rname))
                connector_strs.append(
                    SERVICE_CONNECTOR.format(conn_name=cname, roles="\n".join(roles)))

    def _process_topics(self,
                        topics: Dict[str, _TopicInformation],
                        connector_strs: List[str],
                        attachments: List[str],
                        attachments_to_topic: Dict[str, List[str]]) -> None:
        for t in topics.keys():
            if len(topics[t].pubs) + len(topics[t].subs) > 1:
                roles = []
                for p in topics[t].pubs:
                    rname = f"{AcmeGenerator.to_acme_name(p)}_pub"
                    role = ADVERTISER_ROLE.format(role_name=rname)
                    roles.append(role)
                for sub in topics[t].subs:
                    rname = f"{AcmeGenerator.to_acme_name(sub)}_sub"
                    role = SUBSCRIBER_ROLE.format(role_name=rname)
                    roles.append(role)
                cname = AcmeGenerator.to_acme_name(topics[t].details.name) + "_conn"
                conn = TOPIC_CONNECTOR.format(conn_name=cname, roles="\n".join(roles),
                                              msg_type=topics[t].details.format,
                                              topic=topics[t].details.name)
                connector_strs.append(conn)

                for a in attachments_to_topic[t]:
                    attachments.append(a)

    @staticmethod
    def check_acme_file(filename: str) -> Tuple[bytes, bytes]:
        process = subprocess.Popen(list(['java', '-jar', 'lib/acme.standalone-ros.jar', filename]))
        (output, err) = process.communicate()
        process.wait()
        return output, err

    def check_acme_string(self, acme: str) -> Tuple[bytes, bytes]:
        f, filename = tempfile.mkstemp()
        try:
            with open(filename, 'w') as fi:
                fi.write(acme)
                fi.close()
            return self.check_acme_file(filename)
        finally:
            os.unlink(filename)

    def generate_acme_file(self, acme: str):
        if self.__acme_file is not None:
            logger.info(f"Writing Acme to {self.__acme_file}")
            with open(self.__acme_file, 'w') as f:
                f.write(acme)

    def _output(self, message: str):
        logger.info(message)
        print(message)

    def check_acme(self):
        self._check_acme(self.__acme_file)

    def _check_acme(self, acme_file: str):
        (_, jf) = tempfile.mkstemp(suffix=".json")
        try:
            logger.debug("Running Acme checker")
            self._output("Checking architecture...")
            run = subprocess.run(list(["java", "-jar", self.__acme_jar, "-j", jf, acme_file]),
                                 stdout=PIPE, stderr=PIPE)
            if run.returncode == 0:
                logger.debug("Checking ran successfully")
                logger.debug(run.stdout)
                with open(jf, 'r') as j:
                    checks = json.load(j)
                if len(checks["errors"]) == 0:
                    self._output("Robot architecture has no errors")
                else:
                    self._output("The following problems were found with the robot architecture:")
                    for e in checks["errors"]:
                        if 'causes' not in e.keys() or len(e['causes']) == 0:
                            self._output(f"    {e['error']}")
                        else:
                            for c in e["causes"]:
                                self._output(f"    {c}")

            else:
                logger.error("Could not run the checker")
                logger.error(run.stderr)
        finally:
            os.remove(jf)
