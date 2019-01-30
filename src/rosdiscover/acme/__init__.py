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
    }}
    """

NODE_COMPONENT="""   component {comp_name} : ROSNodeCompT = new ROSNodeCompT extended with {{
        {ports}
        property name = "{node_name}";
    }}
    """

SUBSCRIBER_ROLE="""    role {role_name} : ROSTopicSubscriberRoleT = new ROSTopicSubscriberRoleT;
    """

ADVERTISER_ROLE="""     role {role_name} : ROSTopicAdvertiserRoleT = new ROSTopicAdvertiserRoleT;
    """

TOPIC_PORT="""     port {port_name} : ${port_type} = new ${port_type} extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }}
    """

SUBSCRIBER_PORT="""     port {port_name} : TopicSubscribePortT = new TopicSubscribePortT extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }}
    """
ADVERTISER_PORT="""     port {port_name} : TopicAdvertisePortT = new TopicAdvertisePortT extended with {{
        property msg_type = "{msg_type}";
        property topic = "{topic}";
    }}
    """

class AcmeGenerator(object):
    def __init__(self,
                 nodes, # type: Iterator[NodeSummary]
                 launch_file
                 ):    # type: (...) -> None
        self.__nodes = nodes
        self.__launch_file = launch_file

    def get_components_and_connectors(self):
        # type: () ->Tupe[Array[NodeSummary],Dict[str,Dict]]
        components = []
        topics = {}

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

            components.append(node())
        return components,topics


    def generate_acme(self):
        components, topics = self.get_components_and_connectors()

        system_name = os.path.basename(os.path.normpath(self.__launch_file)).split('.')[0]

        acme = "import families/ROSFam.acme;\nsystem %s : ROSFam = new ROSFam extended with {\n" %system_name;
        attach = ""
        for c in components:
            ports = ""
            comp_name = c['name'].replace("/","_")
            for p in c['pubs']:
                pname = p['name'].replace("/","_") + "_pub"
                port = ADVERTISER_PORT.format(port_name=pname, msg_type=p['format'], topic=p['name'])
                ports = ports + port + ";\n";
                attach = attach + "  attachment %s.%s to %s.%s;\n" %(comp_name,pname,"%s_conn" %p['name'].replace("/","_"), "%s_pub" %comp_name)
            for s in c['subs']:
                pname = s['name'].replace("/","_") + "_sub"
                port = SUBSCRIBER_PORT.format(port_name=pname, msg_type=s['format'], topic=s['name'])
                ports = ports + port + ";\n";
                attach = attach + "  attachment %s.%s to %s.%s;\n" %(comp_name,pname,"%s_conn" %s['name'].replace("/","_"), "%s_sub" %comp_name)


            comp = NODE_COMPONENT.format(comp_name=comp_name, ports=ports, node_name=c['name'])
            acme = acme + comp + ";\n"

        for t in topics:
            roles = ""
            for p in topics[t]["pubs"]:
                rname= p + "_pub"
                role = ADVERTISER_ROLE.format(role_name=rname)
                roles = roles + role + "\n"
            for s in topics[t]["subs"]:
                rname= s + "_sub"
                role = SUBSCRIBER_ROLE.format(role_name=rname)
                roles = roles + role + "\n"
            cname = topics[t]["details"]['name'].replace("/","_") + "_conn"
            conn = TOPIC_CONNECTOR.format(conn_name=cname, roles=roles, msg_type=topics[t]["details"]['format'], topic=topics[t]["details"]['name'])
            acme = acme + conn + ";\n"

        acme = acme + attach + "}"
        return acme