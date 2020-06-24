import yaml 

import os, sys


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


from launch_ros.actions import Node
from launch.launch_description_sources import get_launch_description_from_python_launch_file



description = get_launch_description_from_python_launch_file('robot.launch.py')
nodes = description.entities


def toYaml(nodes):
    to_yaml = {}
    no_name = 0
    
    for node in nodes:
        if isinstance(node, IncludeLaunchDescription):
            description1 = node._IncludeLaunchDescription__launch_description_source.try_get_launch_description_without_context()
            if description1 == None:
                continue
            nodes1 = description1.entities
            to_yaml.update(toYaml(nodes1))
        
        if isinstance(node, Node):
            dic = {}
            if node._Node__node_name != None:
                name = node._Node__node_name
                name_entry = name + 'name'
                dic.update({name+'name': node._Node__node_name})
            else:
                name = 'unknown' + str(no_name)
                name_entry = name
                no_name+=1
                dic.update({'name': name})
            if node._Node__node_namespace != None:
                namespace_entry = name + 'namespace'
                dic.update({'namespace': node._Node__node_namespace})
            if node._Node__node_executable != None:
                exec_entry = name + 'executable'
                dic.update({'executable': node._Node__node_executable})
            if node._Node__package != None:
                pack_entry = name + 'package'
                dic.update({'package': node._Node__package})

            to_yaml.update({name_entry: dic})
    return to_yaml 

with open('Arch.yml', 'w') as f:
    print(yaml.dump(toYaml(nodes), f, default_flow_style=False))

