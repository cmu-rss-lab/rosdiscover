rosdiscover
===========

.. image:: https://github.com/rosqual/rosdiscover/actions/workflows/tox.yml/badge.svg
    :target: https://github.com/rosqual/rosdiscover/actions/workflows/tox.yml

Description
-----------

rosdiscover is a system for extracting the run-time software architecture from ROS system code.
Current ROS tools like rosgraph or rosdoctor require the system to be running and the node
interconnections to be observed from the system. In contrast, rosdiscover analyzes the launch
files and source code
of ROS systems to derive the anticipated run-time architecture (the nodes that will be started,
the topic, service, and action connections between them, as well as parameters that are read and
written). This architecture information can be used by downstream tools to support various kinds
of architectural analysis, such as configuration error checking.

This project is a research project within the Institute for Software Research at Carnegie Mellon
University.

The currently supported output formats are a simple YML file listing the nodes in a similar way
to what might be reported by running :code:`rosnode list` and :code:`rosnode info` but without the
need to
run the system, and an architecture description language called `Acme <http://acme.able.cs.cmu
.edu>`_, used within our research group.

The system assumes that the ROS system is contained inside a Docker container, and uses `roswire
<https://github.com/ChrisTimperley/roswire>`_ to interact with the container.

We are developing support for ROS 1 and ROS 2, and both static and dynamic extraction.

Installation
------------

See below for two different methods of installing rosdiscover.
In general, the native installation should be preferred, but for some cases
(e.g., machines that run Mac OS or Windows), the Docker-based method is
ideal.

Both methods require that Docker is installed on your machine and that your
user belongs to the :code:`docker` group (i.e., :code:`sudo` isn't required
to run :code:`docker` commands). For instructions on installing Docker,
refer to: https://docs.docker.com/install/


Native Installation
...................

The ideal way to install :code:`rosdiscover` is to install to a virtual environment
running on your host machine. This method requires that your host machine is
running Python 3.6 or greater. If that isn't the case, the safest way to install
a newer version of Python on your machine is via `pyenv <https://github.com/pyenv/pyenv>`_,
which allows you to manage multiple installations of Python.

We strongly recommend that you install :code:`rosdiscover` inside a Python virtual
environment (via virtualenv or pipenv) to avoid interfering with the rest of
your system (i.e., to avoid Python’s equivalent of DLL hell).
To install roswire from source within a virtual environment using :code:`pipenv`:

.. code::

   $ git clone git@github.com:ChrisTimperley/rosdiscover rosdiscover
   $ cd rosdiscover
   $ pipenv shell

   (rosdiscover) $ pip install -e .


(Alternative) Docker Installation
.................................

**(WARNING: This approach is more complex than the native installation:
Where possible, you should try to stick to the native installation.)**

In some cases, it may not be possible to install :code:`rosdiscover` natively on
your machine (e.g., Mac OS or Windows machines). :code:`rosdiscover` can be
installed on such systems by building (or downloading) and using a Docker
image for :code:`rosdiscover`.

To build the Docker image for :code:`rosdiscover`:

.. code::

   $ docker build -t rosdiscover .

To run :code:`rosdiscover` commands via Docker, replace in all commands shown below
:code:`rosdiscover` with the following prefix:

.. code::

   $ docker run --rm \
       -v /var/run/docker.sock:/var/run/docker.sock \
       -it rosdiscover

where :code:`-v /var/run/docker.sock:/var/run/docker.sock` is used to mount the
host's Docker socket inside the :code:`rosdiscover` container.
Optionally, you may also want to use volume mounting to persist (and reuse) the cache:

**TODO: requires careful handling of users/permissions.**


Getting Started
------------------

ROSDiscover offers a number of commands for interacting with Docker images,
all of which accept the path to a YAML configuration file as one of their
parameters. Several example YAML configuration files can be found under the
:code:`example` directory at the root of this repository. Below is an example of
one of those configuration files, :code:`example/fetch.yml`, a configuration file
for the `Fetch mobile robot <https://github.com/TheRobotCooperative/TheRobotCooperative/tree/master/fetch>`_.

.. code:: yaml

   image: fetch
   sources:
   - /opt/ros/melodic/setup.bash
   - /ros_ws/devel/setup.bash
   launches:
   - /ros_ws/src/fetch_gazebo/fetch_gazebo/launch/pickplace_playground.launch

The :code:`image` property specifies the name of the Docker image that is used
to provide the robot. :code:`sources` gives an ordered list of the scripts that
must be sourced to setup the correct working environment for the robot;
in most cases, :code:`sources` will look as it does above, except the term
:code:`melodic` may be replaced with the name of another ROS distribution
(e.g., :code:`indigo` or :code:`kinetic`).
:code:`launches` gives an ordered list of the XML
launch files that should be used to launch the software for the robot.
For now, each element in this list is an absolute path to a launch file
inside the container. In the near future, support for relative paths
(e.g., name of package + name of launch file) and passing command-line
arguments will be added. There is also an additional :code:`environment` property,
exemplified below, which accepts a mapping from names of environment
variables to their respective values. This is useful in a small number of
cases (e.g., specifying :code:`TURTLEBOT3_MODEL` for TurtleBot3).

.. code:: yaml

   image: turtlebot3
   sources:
   - /opt/ros/kinetic/setup.bash
   - /ros_ws/devel/setup.bash
   environment:
     TURTLEBOT3_MODEL: burger
   launches:
   - filename: /ros_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_house.launch
     arg1: value
     arg2: value
     arg3: value
       

Commands
........

To see a complete list of commands that are supported by ROSDiscover,
run the following:

.. code::

   $ rosdiscover --help
   usage: rosdiscover [-h] {launch,rostopic,rosservice,acme} ...

   discovery of ROS architectures

   positional arguments:
     {launch,rostopic,rosservice,acme}
       launch              simulates the effects of a roslaunch.
       rostopic            simulates the output of rostopic for a given
                           configuration.
       rosservice          simulates the output of rosservice for a given
                           configuration.
       acme                generates Acme from a source file

   optional arguments:
     -h, --help            show this help message and exit

The :code:`launch` command is used to simulate the effects of a sequence of
:code:`roslaunch` calls for a robot application:

.. code::

   $ rosdiscover launch example/fetch.yml


Docker Development Setup (for Windows 10)
-----------------------------------------

If you are planning to develop on Windows 10, then you will need to mount
rosdiscover source directories into a Docker container. You can use your
favorite Python IDE in Windows, but run and test rosdiscover inside the
container.

We provide a Docker build file for setting up this development environment. To
run inside the image you need to mount (i) the source directory that is the top
of this repository as :code:`/code` in the container, (ii) the socket/port that the
host docker daemon connects to (so that rosdiscover can find other, (iii)
(recommended) a host folder that can be used to cache some of the rosdiscover
builds, so that there is no need to start from scratch with rosdiscover each
time.

To run rosdiscover on Windows 10, where the source code is mounted on
:code:`D:/rosdiscover`, and you want to store the cache on :code:`D:/cache`:

1. Ensure that the folders to mount are shared. This needs to be done through
   the Docker settings on your host. (This is done through the Dashboard or
   through settings on Windows Docker)
2. Build the development docker image:

   .. code::

      $ docker build -t build/rosdiscover-dev -f .\Dockerfile-dev .

3. Run the docker image:

   .. code::

      $ docker run --rm -v d:/rosdiscover:/code -v d:/cache:/root/.roswire -v //var/run/docker.sock:/var/run/docker.sock -it build/rosdiscover-dev

4. Once the shell has started and you are inside the container, you will need to install `rosdiscover` locally:

   .. code::

      bash-4.4# pip install -e .

You will then be able to  run `rosdiscover` from the command line.
