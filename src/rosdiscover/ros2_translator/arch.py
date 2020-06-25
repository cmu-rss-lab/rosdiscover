import json
import argparse
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import get_launch_description_from_python_launch_file


def toyaml(nodes):
    to_yaml = {}
    no_name = 0
    for node in nodes:
        if isinstance(node, IncludeLaunchDescription):
            description_from_node = node._IncludeLaunchDescription__launch_description_source.try_get_launch_description_without_context()
            if description_from_node is None:
                continue
            nodes1 = description_from_node.entities
            to_yaml.update(toyaml(nodes1))
        if isinstance(node, Node):
            '''
            Note: The code directly accesses variables since there are no getters for
            these specific variables in launch.
            This code might break with changed implementations of launch
            '''
            dic = {}
            if node._Node__node_name:
                name_entry = node._Node__node_name
                dic['name'] = node._Node__node_name
            else:
                name_entry = 'unknown' + str(no_name)
                no_name += 1
                dic['name'] = name_entry
            if node._Node__node_namespace:
                dic['namespace'] = node._Node__node_namespace
            if node._Node__node_executable:
                dic['executable'] = node._Node__node_executable
            if node._Node__package:
                dic['package'] = node._Node__package
            to_yaml[name_entry] = dic
    return to_yaml


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Find nodes')
    parser.add_argument('--launchfile', type=str)
    args = parser.parse_args()
    description = get_launch_description_from_python_launch_file(args.launchfile)
    nodes = description.entities
    with open('Arch.yml', 'w') as f:
        f.write(json.dumps(toyaml(nodes), indent=4, separators=(". ", " = ")))
