"""
This module is used to generate an Acme description from a set of nodes parsed
from aa launch file. 

The main class within this module is :clas:`AcmeGenerator`
"""
import logging
import copy
import os

logger = logging.getLogger(__name__) # type: logging.Logger
logger.setLevel(logging.DEBUG)

# Constants for Acme generation
TOPIC_CONNECTOR="""   connector {conn_name} : TopicConnectorT = new TopicConnectorT extended with {{
    {roles}
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """
SERVICE_CONNECTOR="""  connector {conn_name} : ServiceConnT = new ServiceConnT extended with {{
    {roles}
    }};
   """
NODE_COMPONENT="""   component {comp_name} : ROSNodeCompT = new ROSNodeCompT extended with {{
        {ports}
        property name = "{node_name}";
    }};
    """

SUBSCRIBER_ROLE="""    role {role_name} : ROSTopicSubscriberRoleT = new ROSTopicSubscriberRoleT;
    """

ADVERTISER_ROLE="""     role {role_name} : ROSTopicAdvertiserRoleT = new ROSTopicAdvertiserRoleT;
    """

PROVIDER_ROLE="""     role {role_name} : ROSServiceCallRoleT = new ROSServiceCallRoleT;
    """

CLIENT_ROLE="""       role {role_name} : ROSServiceProviderRoleT = new ROSServiceProviderRoleT;
    """

TOPIC_PORT="""     port {port_name} : ${port_type} = new ${port_type} extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """

SUBSCRIBER_PORT="""     port {port_name} : TopicSubscribePortT = new TopicSubscribePortT extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """
ADVERTISER_PORT="""     port {port_name} : TopicAdvertisePortT = new TopicAdvertisePortT extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }};
    """

PROVIDER_PORT="""     port {port_name} : ServiceProviderPortT = new ServiceProviderPortT extended with {{
        property svc_type : string = "{svc_type}"
        property name : string = "{service}"
        property args : string = "";
    }};
    """

REQUIRER_PORT=""""     port {port_name} : ServiceClientPortT = new ServiceClientPortT extended with {{
        property svc_type : string = "{svc_type}"
        property persistency : boolean = {persistence}
    }};
    """

class AcmeGenerator(object):
    def __init__(self,
                 nodes, # type: Iterator[NodeSummary]
                 launch_file
                 ):    # type: (...) -> None
        self.__nodes = nodes
        self.__launch_file = launch_file

    def get_components_and_connectors(self):
        # type: () ->Tuple[Array[NodeSummary],Dict[str,Dict],Dict[str,Dict]]
        components = []
        topics = {}
        services = {}

        for node in self.__nodes:
            for pub in node()['pubs']:
                topic={}
                if pub["name"] in topics:
                    topic = topics[pub["name"]]
                else:
                    topic = {'details' : pub, "pubs" : [], "subs" : []}
                    topics[pub["name"]] = topic
                topic["pubs"].append(node()["name"])
            for sub in node()['subs']:
                topic={}
                if sub["name"] in topics:
                    topic = topics[sub["name"]]
                else:
                    topic = {'details' : sub, "pubs" : [], "subs" : []}
                    topics[sub["name"]] = topic
                topic["subs"].append(node()["name"])
            for prov in node()['provides']:
                service={}
                if prov["name"] in services:
                    service = services[prov["name"]]
                else:
                    service = {'details' : prov, "provs" : [], "reqs" : []}
                    services[prov["name"]] = service
                service["provs"].append(node()["name"])
            for call in node()['requires']:
                service={}
                if call["name"] in services:
                    service = services[call["name"]]
                else:
                    service = {'details' : call, "provs": [], "reqs" : []}
                    services[call["name"]] = service
                service["reqs"].append(node()["name"])
            components.append(node())
        return components, topics, services

    def update_service_conn(self, conns, service, port_qualified, is_provider):
        s = {}
        if service in conns:
            s = conns[service]
        else:
            s={}
            s.name = service
            s.callers= {}
            s.providers = {}
            conns[service] = s
        if is_provider:
            s.providers.add(port_qualified)
        else:
            s.callers.add(port_qualified)

    def to_acme_name(self, name):
        return name.replace("/","_")

    def generate_acme(self):
        # type: () -> str
        components, topics, services = self.get_components_and_connectors()

        system_name = os.path.basename(os.path.normpath(self.__launch_file)).split('.')[0]

        acme = "import families/ROSFam.acme;\nsystem %s : ROSFam = new ROSFam extended with {\n" %system_name;
        attachments = []
        component_strs = []
        service_conns={}

        ATTACHMENT="""  attachment {comp}.{port} to {conn}.{role};"""
        SERVICE_ATTACHMENT = """  attachment {qualified_port} to {conn}.{role};"""
        for c in components:
            ports = []
            comp_name = self.to_acme_name(c['name'])
            for p in c['pubs']:
                pname = self.to_acme_name(p['name']) + "_pub"
                port = ADVERTISER_PORT.format(port_name=pname, msg_type=p['format'], topic=p['name'])
                ports.append(port)
                attachments.append(ATTACHMENT.format(comp=comp_name, port=pname, conn="%s_conn" %self.to_acme_name(p['name']), role="%s_pub" %comp_name))
            for s in c['subs']:
                pname = self.to_acme_name(s['name']) + "_sub"
                port = SUBSCRIBER_PORT.format(port_name=pname, msg_type=s['format'], topic=s['name'])
                ports.append(port)
                attachments.append(ATTACHMENT.format(comp=comp_name, port=pname, conn="%s_conn" %self.to_acme_name(p['name']), role="%s_sub" %comp_name))
            for s in c['provides']:
                pname=self.to_acme_name(s['name']) + "_svc"
                port = PROVIDER_PORT.format(port_name=pname, svc_type=s['format'], service=s['name'])
                ports.append(port)
                self.update_service_conn(service_conns,s['name'], "%s.%s" %(comp_name, pname), True)
                #attach = attach + "  attachment %s.%s to %s.%s;\n" %(comp_name,pname,"%s_conn" %s['name'].replace("/","_"), "%s_prov" %comp_name)
            for s in c['calls']:
                pname=self.to_acme_name(s['name']) + "_call"
                port = REQUIRER_PORT.format(port_name=pname, svc_type=s['format'], service=s['name'])
                ports.append(port)
                self.update_service_conn(service_conns,s['name'], "%s.%s" %(comp_name, pname), False)

                #attach = attach + "  attachment %s.%s to %s.%s;" %(comp_name,pname, "%s_conn" %s['name'].replace("/","_"), "%s_call" %comp_name)


            comp = NODE_COMPONENT.format(comp_name=comp_name, ports="\n".join(ports), node_name=c['name'])
            component_strs.append(comp)
        acme = acme + "\n".join(component_strs)

        connector_strs = []
        for t in topics:
            roles = []
            for p in topics[t]["pubs"]:
                rname= p + "_pub"
                role = ADVERTISER_ROLE.format(role_name=rname)
                roles.append(role)
            for s in topics[t]["subs"]:
                rname= s + "_sub"
                role = SUBSCRIBER_ROLE.format(role_name=rname)
                roles.append(role)
            cname = self.to_acme_name(topics[t]["details"]['name']) + "_conn"
            conn = TOPIC_CONNECTOR.format(conn_name=cname, roles="\n".join(roles), msg_type=topics[t]["details"]['format'], topic=topics[t]["details"]['name'])
            connector_strs.append(conn)

        for s in service_conns:
            # Only create a connector for services that are connected
            if len(s.providers) != 0 and len(s.callers) != 0:
                roles = []
                cname="%s_conn" %self.to_acme_name(s)
                for p in service_conns[s].providers:
                    rname = self.to_acme_name(p)
                    role=PROVIDER_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualitifed_port=p,conn=cname,role=rname))
                for p in service_conns[s].callers:
                    rname = self.to_acme_name(p)
                    role=CLIENT_ROLE.format(role_name=rname)
                    roles = "%s%s\n" %(roles,role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualitifed_port=p,conn=cname,role=rname))
                connector_strs.append(SERVICE_CONNECTOR.format(conn_name=cname, roles="\n".join(roles)))
    
        acme = acme + "\n".join(connector_strs)
        acme = acme + "\n".join(attachments) + "}"
        return acme