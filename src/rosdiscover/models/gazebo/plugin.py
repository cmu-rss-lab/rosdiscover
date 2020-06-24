# -*- coding: utf-8 -*-
import xml.etree.ElementTree as ET


def gazebo_plugin():
    pass


# name, filename, element


"""
Example definition
------------------

.. code:: xml

    <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_lds_lfcd_controller">
      <topicName>scan</topicName>
      <frameName>base_scan</frameName>
    </plugin>
"""
@gazebo_plugin('libgazebo_ros_laser.so')
def plugin_libgazebo_ros_laser(xml_definition: ET.Element) -> None:
    pass


class ModelPlugin(abc.ABC):
    @abc.abstractmethod
    def load(self, graph: GraphContext) -> None:
        ...


class GazeboPlugin(ModelPlugin):
    pass


class LibGazeboROSLaserPlugin(GazeboPlugin):
    filename = 'libgazebo_ros_laser.so'
