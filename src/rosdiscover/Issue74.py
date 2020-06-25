import time
import collections
import sys

sys.path.append("/interpreter")
from .interpreter import NodeSummary


from loguru import logger

import roswire

f = open("Arch.yml", "w")

# enable logging
logger.enable('roswire')


# first, we create a new ROSWire session
rsw = roswire.ROSWire()

def toString(l): 
    s = ''
    l1 = list(l)
    if len(l1) == 0: 
        return '[]'
    for i in l1:
        (fmt, topic) = i
        s = s + "\n  - format: " + fmt + "\n    name: " + topic + "\n" 
    return s

# we then launch a container for a given robot.
# once the end of the given context is reached, including the event where an
# exception occurs, the container will be automatically destroyed.
image = 'therobotcooperative/turtlebot3'
sources = ['/opt/ros/kinetic/setup.bash', '/ros_ws/devel/setup.bash']
environment = {'TURTLEBOT3_MODEL': 'burger'}
with rsw.launch(image, sources, environment=environment) as system:

    # we then launch ROS inside the container
    # again, once the end of the context is reached, ROS is killed.
    with system.roscore() as ros:

        # let's bring up the application
        ros.roslaunch('turtlebot3_house.launch',
                      package='turtlebot3_gazebo',
                      args={'gui': 'false'})

        # we need to wait for the nodes to finish their initialisation
        time.sleep(30)
        
        node_names = list(ros.nodes)

        # get an overview of the system state
        state = ros.state

        # get topic types
        topic_to_type = ros.topic_to_type

        # get service formats
        
        service_to_format = {}
        for service_name in state.services:
            service = ros.services[service_name]
            service_to_format[service_name] = service.format.name
        

nodeSummaryDict = {}
for n in node_names: #create the dictionary of NodeSummary objects
    
    p = []
    for key in state.publishers:
        pubs = state.publishers[key]
        for i in pubs:
            if i == n: 
                p.append((topic_to_type[key], key))
       
    s = []

    for key in state.subscribers:
        subs = state.subscribers[key]
        for i in subs:
            if i == n:
                s.append((topic_to_type[key], key))
    
    serv = []
    
    for key in service_to_format: 
        path = n + '/'
        if path in key:
            serv.append((service_to_format[key], key))

    obj = NodeSummary('', n, '/', '', '', False, '', False, p, s, [], [], [], serv, [], [])
    nodeSummaryDict.update( {n : obj} )
    

for i in nodeSummaryDict: #add to Arch.yml file
    obj = nodeSummaryDict[i] 
    f.write(f"- action-clients: {toString(obj.action_clients)}\n  action-servers: {toString(obj.action_servers)}\n  filename: {obj.filename}\n  fullname: {obj.fullname}\n  kind: {obj.kind}\n  name: {obj.name}\n  namespace: {obj.namespace}\n  nodelet: {obj.nodelet}\n  package: {obj.package}\n  placeholder: {obj.placeholder}\n  provides: {toString(obj.provides)}\n  pubs: {toString(obj.pubs)}\n  reads: {toString(obj.reads)}\n  subs: {toString(obj.subs)}\n  uses: {toString(obj.uses)}\n  writes: {toString(obj.writes)}\n\n")


f.close()
