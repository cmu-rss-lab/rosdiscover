"""
This module is used to generate an Acme description from a set of nodes parsed
from aa launch file. 

The main class within this module is :clas:`AcmeGenerator`
"""
import logging
import copy
import os
import tempfile
import subprocess
import json


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

ACTION_CONNECTOR="""  connector {conn_name} : ActionServerConnT = new ActionServerConnT extended with {{
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

ACTION_CLIENT_ROLE="""      role {role_name} : ROSActionCallerRoleT = new ROSActionCallRoleT;
    """

ACTION_SERVER_ROLE="""      role {role_name} : ROSActionResponderRoleT = new ROSActionResponderRoleT;
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
        property svc_type : string = "{svc_type}";
        property name : string = "{service}";
        property args : string = "";
    }};
    """

REQUIRER_PORT="""     port {port_name} : ServiceClientPortT = new ServiceClientPortT extended with {{
        property svc_type : string = "{svc_type}";
        property persistency : boolean = {persistence}
    }};
    """

ACTION_CLIENT_PORT="""    port {port_name} : ActionClientPortT = new ActionClientPortT extended with {{
        property action_type : string = "{action_type}";
    }};
    """

ACTION_SERVER_PORT="""    port {port_name}: ActionServerPortT = new ActionServerPortT extended with {{
        property action_type : string = "{action_type}";
    }};
    """

class AcmeGenerator(object):

    def __init__(self, nodes, acme_file, jar):    # type: (...) -> None
        self.__nodes = nodes
        self.__acme_file = acme_file
        self.__generate_dangling_connectors = False
        self.__acme_jar = jar if jar is not None else 'lib/acme.standalone-ros.jar'

    def get_components_and_connectors(self):
        # type: () ->Tuple[Array[NodeSummary],Dict[str,Dict],Dict[str,Dict]]
        components = []
        topics = {}
        services = {}
        actions = {}

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
            # for call in node()['requires']:
            #     service={}
            #     if call["name"] in services:
            #         service = services[call["name"]]
            #     else:
            #         service = {'details' : call, "provs": [], "reqs" : []}
            #         services[call["name"]] = service
            #     service["reqs"].append(node()["name"])
            for ac in node()["action-servers"]:
                action={}
                if ac["name"] in actions:
                    action = actions[ac["name"]]
                else:
                    action={'details' : ac, "servers": [], "clients": []}
                    actions[ac["name"]] = action
                action["servers"].append(node()["name"])
            for ac in node()["action-clients"]:
                action={}
                if ac["name"] in actions:
                    action = actions[ac["name"]]
                else:
                    action={'details' : ac, "servers": [], "clients": []}
                    actions[ac["name"]] = action
                action["clients"].append(node()["name"])
            components.append(node())
        return components, topics, services, actions

    def update_service_conn(self, conns, service, port_qualified, is_provider):
        s = {}
        if service in conns:
            s = conns[service]
        else:
            s={"name" : service, 'callers': set(), 'providers' : set()}
            # s.name = service
            # s.callers= {}
            # s.providers = {}
            conns[service] = s
        if is_provider:
            s['providers'].add(port_qualified)
        else:
            s['callers'].add(port_qualified)

    def update_action_conn(self, conns, action, port_qualified, is_server):
        a = {}
        if action in conns:
            a = conns[action]
        else:
            a={'name' : action, "clients" : set(), 'servers' : set()}
            conns[action] = a
        if is_server:
            a['servers'].add(port_qualified)
        else:
            a['clients'].add(port_qualified)

    def to_acme_name(self, name):
        return name.replace("/","_")

    def generate_acme(self):
        # type: () -> str
        components, topics, services, actions = self.get_components_and_connectors()

        system_name = "RobotSystem" if self.__acme_file is None else '_'.join(self.__acme_file.split(".")[:-1])
        # system_name = os.path.basename(os.path.normpath(self.__launch_files)).split('.')[0]

        acme = "import families/ROSFam.acme;\nsystem %s : ROSFam = new ROSFam extended with {\n" %system_name;
        attachments = []
        component_strs = []
        service_conns={}
        action_conns={}
        attachments_to_topic = {}
        ATTACHMENT="""  attachment {comp}.{port} to {conn}.{role};"""
        SERVICE_ATTACHMENT = """  attachment {qualified_port} to {conn}.{role};"""
        for c in components:
            ports = []
            comp_name = self.to_acme_name(c['name'])

            for p in c['pubs']:
                if p['name'] not in attachments_to_topic.keys():
                    attachments_to_topic[p['name']] = []
                pname = self.to_acme_name(p['name']) + "_pub"
                port = ADVERTISER_PORT.format(port_name=pname, msg_type=p['format'], topic=p['name'])
                ports.append(port)
                attachments_to_topic[p['name']].append(ATTACHMENT.format(comp=comp_name, port=pname,
                    conn="%s_conn" %self.to_acme_name(p['name']), role="%s_pub" %comp_name))
            for s in c['subs']:
                if s['name'] not in attachments_to_topic.keys():
                    attachments_to_topic[s['name']] = []
                pname = self.to_acme_name(s['name']) + "_sub"
                port = SUBSCRIBER_PORT.format(port_name=pname, msg_type=s['format'], topic=s['name'])
                ports.append(port)
                attachments_to_topic[s['name']].append(ATTACHMENT.format(comp=comp_name, port=pname,
                    conn="%s_conn" %self.to_acme_name(s['name']), role="%s_sub" %comp_name))
            for s in c['provides']:
                pname=self.to_acme_name(s['name']) + "_svc"
                port = PROVIDER_PORT.format(port_name=pname, svc_type=s['format'], service=s['name'])
                ports.append(port)
                self.update_service_conn(service_conns,s['name'], "%s.%s" %(comp_name, pname), True)
                #attach = attach + "  attachment %s.%s to %s.%s;\n" %(comp_name,pname,"%s_conn" %s['name'].replace("/","_"), "%s_prov" %comp_name)
            # for s in c['calls']:
            #     pname=self.to_acme_name(s['name']) + "_call"
            #     port = REQUIRER_PORT.format(port_name=pname, svc_type=s['format'], service=s['name'])
            #     ports.append(port)
            #     self.update_service_conn(service_conns,s['name'], "%s.%s" %(comp_name, pname), False)

                #attach = attach + "  attachment %s.%s to %s.%s;" %(comp_name,pname, "%s_conn" %s['name'].replace("/","_"), "%s_call" %comp_name)
            for a in c['action-servers']:
                pname=self.to_acme_name(a['name']) + "_srvr";
                port = ACTION_SERVER_PORT.format(port_name=pname, action_type=a['name'])
                ports.append(port)
                self.update_action_conn(action_conns,a['name'], "%s.%s" %(comp_name,pname), True)
            for a in c['action-clients']:
                pname=self.to_acme_name(a['name']) + "_cli";
                port = ACTION_CLIENT_PORT.format(port_name=pname, action_type=a['name'])
                ports.append(port)
                self.update_action_conn(action_conns,a['name'], "%s.%s" %(comp_name,pname), False)

            comp = NODE_COMPONENT.format(comp_name=comp_name, ports="\n".join(ports), node_name=c['name'])
            component_strs.append(comp)
        acme = acme + "\n".join(component_strs)

        connector_strs = []
        for t in topics:

            if len(topics[t]["pubs"]) + len(topics[t]["subs"]) > 1:
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

                for a in attachments_to_topic[t]:
                    attachments.append(a)

        for s in service_conns:
            # Only create a connector for services that are connected
            if self.__generate_dangling_connectors or (len(service_conns[s]['providers']) != 0 and len(service_conns[s]['callers']) != 0):
                roles = []
                cname="%s_conn" %self.to_acme_name(s)
                for p in service_conns[s]['providers']:
                    rname = self.to_acme_name(p)
                    role=PROVIDER_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualitifed_port=p,conn=cname,role=rname))
                for p in service_conns[s]['callers']:
                    rname = self.to_acme_name(p)
                    role=CLIENT_ROLE.format(role_name=rname)
                    roles = "%s%s\n" %(roles,role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualitifed_port=p,conn=cname,role=rname))
                connector_strs.append(SERVICE_CONNECTOR.format(conn_name=cname, roles="\n".join(roles)))
        for a in action_conns:
            # only create a connector for actions that are connected
            if self.__generate_dangling_connectors or (len(action_conns[a]['servers']) != 0 and len(action_conns[a]['clients']) != 0):
                roles = []
                cname="%s_conn" %self.to_acme_name(a)
                for c in action_conns[a]['clients']:
                    rname = self.to_acme_name(c)
                    role=ACTION_CLIENT_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualified_port=c,conn=cname,role=rname))
                for s in action_conns[s]['servers']:
                    rname = self.to_acme_name(s)
                    role=ACTION_SERVER_ROLE.format(role_name=rname)
                    roles.append(role)
                    attachments.append(SERVICE_ATTACHMENT.format(qualified_port=c,conn=cname,role=rname))
                connector_strs.append(ACTION_CONNECTOR.format(conn_name=cname, roles="\n".join(roles)))
        acme = acme + "\n".join(connector_strs)
        acme = acme + "\n".join(attachments) + "}"
        self.generate_acme_file(acme)
        return acme

    def check_acme_file(self,filename):
        process = Popen(['java', '-jar', 'lib/acme.standalone-ros.jar', filename])
        (output,err) = process.communicate()
        exit_code = process.wait()
        return output, err

    def check_acme_string(self,acme):
        (f,filename) = tempfile.mkstemp()
        #filename = f.name
        try:
            f.write(acme)
            f.close()
            return self.check_acme_file(filename)
        finally:
            os.unlink(filename)
            

    def generate_acme_file(self, acme):
        if self.__acme_file is not None:
            logger.info(f"Writing Acme to {self.__acme_file}")
            with open(self.__acme_file, 'w') as f:
                f.write(acme)

    def check_acme(self, acme):
        (_, name) = tempfile.mkstemp(suffix='.acme')
        logger.debug(f"Writing Acme to {name}")
        with open(name, 'w') as f:
            f.write(acme)
        self._check_acme(name)
        os.remove(name)

    def check_acme(self):
        self._check_acme(self.__acme_file)

    def _check_acme(self, __acme_file):
        (_, jf) = tempfile.mkstemp(suffix=".json")
        try:
            logger.debug("Running Acme checker")
            run = subprocess.run(["java", "-jar", self.__acme_jar, "-j", jf, __acme_file], capture_output=True)
            if run.returncode == 0:
                logger.debug("Checking ran successfully")
                logger.info(run.stdout)
                with open(jf, 'r') as j:
                    checks = json.load(j)
                if len(checks["errors"]) == 0:
                    print("Robot architecture has no errors")
                else:
                    print("The following problems were found with the robot architecture:")
                    for e in checks["errors"]:
                        if 'causes' not in e.keys() or len(e['causes']) == 0:
                            print(f"    {e['error']}")
                        else:
                            for c in e["causes"]:
                                print(f"    {c}")

            else:
                logger.error("Could not run the checker")
                logger.error(run.stderr)
        finally:
            os.remove(jf)


"""
rosdiscover acme /ros_ws/src/turtlebot_simulator/turtlebot_stage/launch/turtlebot_in_stage.launch --workspace /ros_ws --acme generated.acme
"""
