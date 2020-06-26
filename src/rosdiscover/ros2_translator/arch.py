from typing import Any, Dict, List
import argparse
import json
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
NodeDescription = Dict[str, Any]
class NodeDescriptionExtractor:


    def __init__(self) -> None:
        self.__unnamed_nodes = 0

    def _node_to_description(self, node: Node) -> NodeDescription:
        # Note: The code directly accesses variables since there are no getters for
        # these specific variables in launch.
        # This code might break with changed implementations of launch
        output: NodeDescription = {}
        if node._Node__node_name:
            name_entry = node._Node__node_name
            output['name'] = node._Node__node_name
        else:
            name_entry = f'unknown{self.__unnamed_nodes}'
            self.__unnamed_nodes += 1
            output['name'] = name_entry
        if node._Node__node_namespace:
            output['namespace'] = node._Node__node_namespace
        if node._Node__node_executable:
            output['executable'] = node._Node__node_executable
        if node._Node__package:
            output['package'] = node._Node__package
        return output

    def _extract_descriptions(self,
                              entity: LaunchDescriptionEntity,
                              context: LaunchContext
                              ) -> List[NodeDescription]:
        descriptions: List[NodeDescription] = []
        if isinstance(entity, LaunchDescription) or isinstance(entity, IncludeLaunchDescription):  # noqa
            for sub_entity in entity.visit(context):
                descriptions += self._extract_descriptions(sub_entity, context)
        elif isinstance(entity, Node):
            descriptions = [self._node_to_description(entity)]
        else:
            print(f'UNKNOWN ENTITY: {type(entity)}')
        return descriptions

    def extract(self, filename: str) -> List[NodeDescription]:
        context = LaunchContext()
        launch_description_source = \
            PythonLaunchDescriptionSource(args.launchfile)
        launch_description = LaunchDescription([
            IncludeLaunchDescription(launch_description_source)
        ])
        return self._extract_descriptions(launch_description, context)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Find nodes')
    parser.add_argument('launchfile', type=str)
    parser.add_argument('--output', type=str, default='arch.json')
    args = parser.parse_args()
    extractor = NodeDescriptionExtractor()
    output = extractor.extract(args.launchfile)
    with open(args.output, 'w') as f:
        f.write(json.dumps(output, indent=2))
