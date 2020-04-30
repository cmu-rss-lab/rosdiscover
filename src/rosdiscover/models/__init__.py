"""
This module provides architectural models for various ROS packages.

Each file within this directory defines a single model for a particular ROS
node type. For now, models are implemented as Python functions that accept a
single argument, which is used to supply a :code:`NodeContext` for evaluating
the various architectural commands within the model definition (e.g.,
subscribing to a particular topic).
The :code:`model` decorator must be used to indicate that a particular
function is used to supply a model definition, as shown below.

.. code:: python

    from ..interpreter import model

    @register('amcl', 'amcl')
    def amcl(c):
        ...

Note that the module (i.e., file) responsible for defining a particular model
must be imported below in order for that model to be loaded. For example, to
ensure that the model provided by :code:`joint_state_publisher.py` is loaded,
the following :code:`import` statement must be added to this file:

.. code:: python

    from . import joint_state_publisher

"""
from . import joint_state_publisher
from . import robot_state_publisher
from . import map_server
from . import amcl
from . import move_base
from . import stage_ros
from . import velocity_smoother
from . import kobuki_safety_controller
from . import cmd_vel_mux
from . import diagnostic_aggregator
from . import rviz
from . import gzserver
from . import spawn_model
from . import turtlebot3_teleop_key
from . import pdw_turtlebot3_fake
from . import pdw_turtlebot3_teleop_key
from . import pdw_robot_state_publisher
from . import topic_tools_mux
from . import image_proc_nodelets
from . import depth_image_proc_nodelets
from . import placeholder
