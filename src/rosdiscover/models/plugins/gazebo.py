# -*- coding: utf-8 -*-
"""
This file provides model plugins that represent various Gazebo plugins.
"""
__all__ = ('GazeboPlugin',)

from typing import Mapping, Type
import abc
import xml.etree.ElementTree as ET  # noqa

from loguru import logger
from roswire.name import namespace_join
import attr

from ...interpreter import Interpreter, ModelPlugin


class GazeboPlugin(ModelPlugin):
    """Represents the architectural effects of a Gazebo plugin."""
    @classmethod
    def from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        name = xml.attrib['name']
        filename = xml.attrib['filename']
        logger.debug(f'loading gazebo plugin [{name}] from file [{filename}] '
                     f'via XML: {ET.tostring(xml).decode("utf-8")}')

        # TODO locate the class for the plugin based on filename
        filename_to_cls: Mapping[str, Type[GazeboPlugin]] = {
            'libgazebo_ros_laser.so': LibGazeboROSLaserPlugin,
            'libgazebo_ros_diff_drive.so': LibGazeboROSDiffDrivePlugin,
            'libgazebo_ros_imu.so': LibGazeboROSIMUPlugin
        }
        cls = filename_to_cls[filename]
        plugin = cls.build_from_xml(xml)
        logger.debug(f'loaded gazebo plugin [{name}] from file [{filename}]: '
                     f'{plugin}')
        return plugin

    @classmethod
    @abc.abstractmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        ...


@attr.s(frozen=True, slots=True)
class LibGazeboROSIMUPlugin(GazeboPlugin):
    """
    Example
    -------

    .. code:: xml

        <plugin filename="libgazebo_ros_imu.so" name="imu_plugin">
          <alwaysOn>true</alwaysOn>
          <bodyName>imu_link</bodyName>
          <frameName>imu_link</frameName>
          <topicName>imu</topicName>
          <serviceName>imu_service</serviceName>
          <gaussianNoise>0.0</gaussianNoise>
          <updateRate>200</updateRate>
          <imu>
            <noise>
              <type>gaussian</type>
              <rate>
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </rate>
              <accel>
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </accel>
            </noise>
          </imu>
        </plugin>
    """
    filename = 'libgazebo_ros_imu.so'
    topic_name: str = attr.ib()
    service_name: str = attr.ib()
    robot_namespace: str = attr.ib()

    def load(self, interpreter: Interpreter) -> None:
        gazebo = interpreter.nodes['/gazebo']
        namespace = self.robot_namespace

        if self.topic_name:
            topic_name = namespace_join(namespace, self.topic_name)
            gazebo.pub(topic_name, 'sensor_msgs/Imu')
            service_name = namespace_join(namespace, self.service_name)
            gazebo.provide(service_name, 'std_srvs/Empty')

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        topic_name: str = '/default_imu'
        xml_topic_name = xml.find('topicName')
        if xml_topic_name is not None and xml_topic_name.text is not None:
            topic_name = xml_topic_name.text

        service_name: str = '/calibrate'
        xml_service_name = xml.find('serviceName')
        if xml_service_name is not None and xml_service_name.text is not None:
            service_name = xml_service_name.text

        robot_namespace: str = ''
        xml_robot_ns = xml.find('robotNamespace')
        if xml_robot_ns is not None and xml_robot_ns.text is not None:
            robot_namespace = xml_robot_ns.text

        return LibGazeboROSIMUPlugin(topic_name=topic_name,
                                     service_name=service_name,
                                     robot_namespace=robot_namespace)


@attr.s(frozen=True, slots=True)
class LibGazeboROSDiffDrivePlugin(GazeboPlugin):
    """
    Example
    -------

    .. code:: xml

        <plugin filename="libgazebo_ros_diff_drive.so" name="turtlebot3_burger_controller">
          <commandTopic>cmd_vel</commandTopic>
          <odometryTopic>odom</odometryTopic>
          <odometryFrame>odom</odometryFrame>
          <odometrySource>world</odometrySource>
          <publishOdomTF>true</publishOdomTF>
          <robotBaseFrame>base_footprint</robotBaseFrame>
          <publishWheelTF>false</publishWheelTF>
          <publishTf>true</publishTf>
          <publishWheelJointState>true</publishWheelJointState>
          <legacyMode>false</legacyMode>
          <updateRate>30</updateRate>
          <leftJoint>wheel_left_joint</leftJoint>
          <rightJoint>wheel_right_joint</rightJoint>
          <wheelSeparation>0.160</wheelSeparation>
          <wheelDiameter>0.066</wheelDiameter>
          <wheelAcceleration>1</wheelAcceleration>
          <wheelTorque>10</wheelTorque>
          <rosDebugLevel>na</rosDebugLevel>
        </plugin>
    """
    filename = 'libgazebo_ros_diff_drive.so'
    command_topic: str = attr.ib()
    odometry_topic: str = attr.ib()

    def load(self, interpreter: Interpreter) -> None:
        gazebo = interpreter.nodes['/gazebo']
        gazebo.pub(self.command_topic, 'geometry_msgs/Twist')
        gazebo.pub(self.odometry_topic, 'nav_msgs/Odometry')

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        xml_command_topic = xml.find('commandTopic')
        xml_odometry_topic = xml.find('odometryTopic')

        assert xml_command_topic is not None
        assert xml_command_topic.text is not None
        assert xml_odometry_topic is not None
        assert xml_odometry_topic.text is not None

        command_topic: str = xml_command_topic.text
        odometry_topic: str = xml_odometry_topic.text
        return LibGazeboROSDiffDrivePlugin(command_topic, odometry_topic)


@attr.s(frozen=True, slots=True)
class LibGazeboROSLaserPlugin(GazeboPlugin):
    """
    Example
    -------

    .. code:: xml

        <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_lds_lfcd_controller">
          <topicName>scan</topicName>
          <frameName>base_scan</frameName>
        </plugin>
    """
    filename = 'libgazebo_ros_laser.so'
    topic_name: str = attr.ib()
    frame_name: str = attr.ib()
    robot_namespace: str = attr.ib()

    def load(self, interpreter: Interpreter) -> None:
        gazebo = interpreter.nodes['/gazebo']
        topic_name = namespace_join(self.robot_namespace, self.topic_name)
        gazebo.pub(topic_name, 'sensor_msgs/LaserScan')

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        xml_topic_name = xml.find('topicName')
        xml_frame_name = xml.find('frameName')
        xml_robot_ns = xml.find('robotNamespace')

        assert xml_topic_name is not None and xml_topic_name.text is not None
        assert xml_frame_name is not None and xml_frame_name.text is not None

        topic_name: str = xml_topic_name.text
        frame_name: str = xml_frame_name.text
        robot_namespace: str = '/'
        if xml_robot_ns is not None and xml_robot_ns.text is not None:
            robot_namespace = xml_robot_ns.text
        return LibGazeboROSLaserPlugin(topic_name=topic_name,
                                       frame_name=frame_name,
                                       robot_namespace=robot_namespace)
