# -*- coding: utf-8 -*-
"""
This file provides model plugins that represent various Gazebo plugins.
"""
__all__ = ('GazeboPlugin',)

import abc
import xml.etree.ElementTree as ET  # noqa
from typing import Mapping, Type

import attr
from loguru import logger
from roswire.name import namespace_join

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
            'libgazebo_ros_imu.so': LibGazeboROSIMUPlugin,
            'libgazebo_ros_control.so': LibGazeboROSControlPlugin,
            'libhector_gazebo_ros_gps.so': LibGazeboROSGpsPlugin,
            'libgazebo_ros_camera.so': LibGazeboROSCameraPlugin,
            'libgazebo_ros_openni_kinect.so': LibGazeboROSOpenniKinectPlugin,
            'libfetch_gazebo_plugin.so': LibFetchGazeboPlugin  # Note, this should be generated
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


@attr.s(frozen=True, slots=True)
class LibGazeboROSControlPlugin(GazeboPlugin):
    """
    Example:

        .. code:: xml

        <plugin name="ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>/rbcar</robotNamespace>
            <robotParam>robot_description</robotParam>
            <!-- controlPeriod>0.003</controlPeriod -->
            <controlPeriod>0.001</controlPeriod>
            <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
        </plugin>
    """
    filename = "libgazebo_ros_control.so"
    topic_name: str = attr.ib()
    robot_namespace: str = attr.ib()

    def load(self, interpreter: Interpreter) -> None:
        gazebo = interpreter.nodes["/gazebo"]
        namespace = self.robot_namespace

        if self.topic_name:
            topic_name = namespace_join(namespace, self.topic_name)
            gazebo.pub(topic_name, "std_msgs/Boolean")

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        topic_name: str = "/default_stop"
        xml_topic_name = xml.find("eStopTopic")
        if xml_topic_name is not None and xml_topic_name.text is not None:
            topic_name = xml_topic_name.text

        robot_namespace: str = ''
        xml_robot_ns = xml.find("robotNamespace")
        if xml_robot_ns is not None and xml_robot_ns.text is not None:
            robot_namespace = xml_robot_ns.text

        return LibGazeboROSControlPlugin(topic_name=topic_name, robot_namespace=robot_namespace)


@attr.s(frozen=True, slots=True)
class LibGazeboROSGpsPlugin(GazeboPlugin):
    """
    Example
    -------

    .. code:: xml

        <plugin name="${prefix}_controller" filename="libhector_gazebo_ros_gps.so">
            <alwaysOn>1</alwaysOn>
            <updateRate>5</updateRate>
            <bodyName>${prefix}_base_link</bodyName> <!-- must be the link of the gps device, not the base_link or base_footprint -->
            <frameId>/${prefix}_base_link</frameId>
            <topicName>fix</topicName>
            <!-- Robotnik position at Fuente del Jarro -->
            <referenceLatitude>39.5080331</referenceLatitude>
            <referenceLongitude>-0.4619816</referenceLongitude>
            <!-- To set heading in ENU orientation (degrees) -->
            <referenceHeading>90</referenceHeading>
            <velocityTopicName>fix_velocity</velocityTopicName>
            <drift>0.0 0.0 0.0</drift>
            <!--<drift>0.0001 0.0001 0.0001</drift>-->
            <!--<drift>0.3 0.3 0.3</drift>-->
            <gaussianNoise>0.1 0.1 0.1</gaussianNoise>
            <!--<gaussianNoise>0.00001 0.00001 0.00001</gaussianNoise>-->
            <velocityDrift>0.00001 0.00001 0.00001</velocityDrift>
            <!--<velocityGaussianNoise>0.1 0.1 0.1</velocityGaussianNoise>-->
            <velocityGaussianNoise>0.00001 0.00001 0.00001</velocityGaussianNoise>
        </plugin>
    """
    filename = "libhector_gazebo_ros_gps.so"
    topic_name: str = attr.ib()
    velocity_topic_name: str = attr.ib()

    def load(self, interpreter: Interpreter) -> None:
        gazebo = interpreter.nodes["/gazebo"]
        gazebo.pub(self.topic_name, 'sensor_msgs/NavSatFix')
        gazebo.pub(self.velocity_topic_name, 'geometry_msgs/Vector3Stamped')

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        xml_topic = xml.find('topicName')
        xml_vel_topic = xml.find('velocityTopicName')

        assert xml_topic is not None
        assert xml_topic.text is not None
        assert xml_vel_topic is not None
        assert xml_vel_topic.text is not None

        topic_name: str = xml_topic.text
        vel_topic_name: str = xml_vel_topic.text

        return LibGazeboROSGpsPlugin(topic_name=topic_name, velocity_topic_name=vel_topic_name)


@attr.s(frozen=True, slots=True)
class LibGazeboROSCameraPlugin(GazeboPlugin):
    """
    Example
    -------

    .. code:: xml

      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>camera</cameraName>
        <frameName>camera_rgb_optical_frame</frameName>
        <imageTopicName>image</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    """
    filename = "libgazebo_ros_camera.so"
    camera_name: str = attr.ib()
    image_topic_name: str = attr.ib()
    camera_info_topic_name: str = attr.ib()
    frame_name: str = attr.ib()
    robot_namespace: str = attr.ib()

    def load(self, interpreter: Interpreter) -> None:
        gazebo = interpreter.nodes['/gazebo']
        image_topic_name = namespace_join(self.robot_namespace, namespace_join(self.camera_name, self.image_topic_name))
        camera_info_topic_name = namespace_join(self.robot_namespace,
                                                namespace_join(self.camera_name, self.camera_info_topic_name))
        for image_topic in [("", "sensor_msgs/Image"), ("/compressed", "sensor_msgs/CompressedImage"),
                            ("/compressedDepth", "sensor_msgs/CompressedImage"),
                            ("/theora", "theora_image_transport/Packet")]:
            gazebo.pub(image_topic_name + image_topic[0], image_topic[1])
        gazebo.pub(camera_info_topic_name, 'sensor_msgs/CameraInfo')

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        xml_camera_name = xml.find("cameraName")
        xml_topic_name = xml.find("imageTopicName")
        xml_camera_topic_name = xml.find('cameraInfoTopicName')
        xml_frame_name = xml.find('frameName')
        xml_robot_ns = xml.find('robotNamesapce')

        assert xml_topic_name is not None and xml_topic_name.text is not None
        assert xml_camera_topic_name is not None and xml_camera_topic_name.text is not None
        assert xml_frame_name is not None and xml_frame_name.text is not None
        assert xml_camera_name is not None and xml_camera_name.text is not None
        topic_name: str = xml_topic_name.text
        camera_topic_name: str = xml_camera_topic_name.text
        camera_name: str = xml_camera_name.text
        frame_name: str = xml_frame_name.text

        robot_ns = "/"
        if xml_robot_ns is not None and xml_robot_ns.text is not None:
            robot_ns = xml_robot_ns.text

        return LibGazeboROSCameraPlugin(camera_name=camera_name,
                                        image_topic_name=topic_name,
                                        camera_info_topic_name=camera_topic_name,
                                        frame_name=frame_name,
                                        robot_namespace=robot_ns)


@attr.s(frozen=True, slots=True)
class LibGazeboROSOpenniKinectPlugin(GazeboPlugin):
    """
    Example
    -------

    .. code:: xml
          <plugin name="head_camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <baseline>0.1</baseline>
        <alwaysOn>true</alwaysOn>
        <updateRate>15.0</updateRate>
        <cameraName>head_camera</cameraName>
        <imageTopicName>/head_camera/rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>/head_camera/rgb/camera_info</cameraInfoTopicName>
        <depthImageTopicName>/head_camera/depth_registered/image_raw</depthImageTopicName>
        <depthImageCameraInfoTopicName>/head_camera/depth_registered/camera_info</depthImageCameraInfoTopicName>
        <pointCloudTopicName>/head_camera/depth_registered/points</pointCloudTopicName>
        <frameName>head_camera_rgb_optical_frame</frameName>
        <pointCloudCutoff>0.35</pointCloudCutoff>
        <pointCloudCutoffMax>4.5</pointCloudCutoffMax>
        <CxPrime>0</CxPrime>
        <Cx>0</Cx>
        <Cy>0</Cy>
        <focalLength>0</focalLength>
        <hackBaseline>0</hackBaseline>
      </plugin>
    """
    filename = "libgazebo_ros_openni_kinect.so"
    camera_name: str = attr.ib()
    image_topic_name: str = attr.ib()
    camera_info_topic_name: str = attr.ib()
    depth_image_topic_name: str = attr.ib()
    depth_image_camera_info_topic_name: str = attr.ib()
    point_cloud_topic_name: str = attr.ib()

    def load(self, interpreter: 'Interpreter') -> None:
        gazebo = interpreter.nodes['/gazebo']

        gazebo.pub(self.image_topic_name, 'sensor_msgs/Image')
        gazebo.pub(self.camera_info_topic_name, 'sensor_msgs/CameraInfo')
        gazebo.pub(self.depth_image_topic_name, 'sensor_msgs/Image')
        gazebo.pub(self.depth_image_camera_info_topic_name, 'sensor_msgs/CameraInfo')
        gazebo.pub(self.point_cloud_topic_name, 'sensor_msgs/PointCloud2')

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        xml_camera_name = xml.find("cameraName")
        xml_image = xml.find('imageTopicName')
        xml_image_camera_info = xml.find('cameraInfoTopicName')
        xml_depth_image = xml.find('depthImageTopicName')
        xml_depth_image_camera_info = xml.find('depthImageCameraInfoTopicName')
        xml_point_cloud = xml.find('pointCloudTopicName')
        assert xml_camera_name is not None and xml_camera_name.text is not None
        camera_name = xml_camera_name.text

        image_t = xml_image.text \
            if xml_image is not None and xml_image.text is not None else 'ir/image_raw'
        image_ci_t = xml_image_camera_info.text \
            if xml_image_camera_info is not None and xml_image_camera_info.text is not None else 'ir/camera_info'
        depth_t = xml_depth_image.text \
            if xml_depth_image is not None and xml_depth_image.text is not None else 'depth/image_raw'
        depth_ci_t = xml_depth_image_camera_info.text \
            if xml_depth_image_camera_info is not None and xml_depth_image_camera_info.text is not None \
            else 'depth/camera_info'
        point_cloud_t = xml_point_cloud.text \
            if xml_point_cloud is not None and xml_point_cloud.text is not None else 'points'

        return LibGazeboROSOpenniKinectPlugin(camera_name=camera_name, image_topic_name=image_t,
                                              camera_info_topic_name=image_ci_t,
                                              depth_image_topic_name=depth_t,
                                              depth_image_camera_info_topic_name=depth_ci_t,
                                              point_cloud_topic_name=point_cloud_t)


# TODO: Generate this from source
@attr.s(frozen=True, slots=True)
class LibFetchGazeboPlugin(GazeboPlugin):
    """
    This is the gazebo plugin for the Fetch robot. It should really be generated.
    """
    filename = "libfectch_gazebo_plugin.so"
    joint_state_topic: str = attr.ib()

    def load(self, interpreter: 'Interpreter') -> None:
        gazebo = interpreter.nodes['/gazebo']
        gazebo.pub(self.joint_state_topic, 'sensor_msgs/JointState')

    @classmethod
    def build_from_xml(cls, xml: ET.Element) -> 'GazeboPlugin':
        return LibFetchGazeboPlugin(joint_state_topic="joint_states")
