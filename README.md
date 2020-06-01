# rosdiscover


## Installation

See below for two different methods of installing rosdiscover.
In general, the native installation should be preferred, but for some cases
(e.g., machines that run Mac OS or Windows), the Docker-based method is
ideal.

Both methods require that Docker is installed on your machine and that your
user belongs to the `docker` group (i.e., `sudo` isn't required to run `docker`
commands). For instructions on installing Docker,
refer to: https://docs.docker.com/install/


### Native Installation

The ideal way to install `rosdiscover` is to install to a virtual environment
running on your host machine. This method requires that your host machine is
running Python 3.6 or greater. If that isn't the case, the safest way to install
a newer version of Python on your machine is via [pyenv](https://github.com/pyenv/pyenv),
which allows you to manage multiple installations of Python.

We strongly recommend that you install `rosdiscover` inside a Python virtual
environment (via virtualenv or pipenv) to avoid interfering with the rest of
your system (i.e., to avoid Python’s equivalent of DLL hell). 
To install roswire from source within a virtual environment using `pipenv`:

```
$ git clone git@github.com:ChrisTimperley/rosdiscover rosdiscover
$ cd rosdiscover
$ pipenv shell

(rosdiscover) $ pip install -e .

```

### (Alternative) Docker Installation

**(WARNING: This approach is more complex than the native installation:
Where possible, you should try to stick to the native installation.)**

In some cases, it may not be possible to install `rosdiscover` natively on
your machine (e.g., Mac OS or Windows machines). `rosdiscover` can be
installed on such systems by building (or downloading) and using a Docker
image for `rosdiscover`.

To build the Docker image for `rosdiscover`:

```
$ docker build -t rosdiscover .
```

To run `rosdiscover` commands via Docker, replace in all commands shown below
`rosdiscover` with the following prefix:

```
$ docker run --rm \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -it rosdiscover
```

where `-v /var/run/docker.sock:/var/run/docker.sock` is used to mount the
host's Docker socket inside the `rosdiscover` container. Optionally, you may
also want to use volume mounting to persist (and reuse) the cache:

**TODO: requires careful handling of users/permissions.**


## Getting Started

ROSDiscover offers a number of commands for interacting with Docker images,
all of which accept the path to a YAML configuration file as one of their
parameters. Several example YAML configuration files can be found under the
`example` directory at the root of this repository. Below is an example of
one of those configuration files, `example/fetch.yml`, a configuration file
for the [Fetch mobile robot](https://github.com/TheRobotCooperative/TheRobotCooperative/tree/master/fetch).

```yaml
image: fetch
sources:
- /opt/ros/melodic/setup.bash
- /ros_ws/devel/setup.bash
launches:
- /ros_ws/src/fetch_gazebo/fetch_gazebo/launch/pickplace_playground.launch
```

The `image` property specifies the name of the Docker image that is used
to provide the robot. `sources` gives an ordered list of the scripts that
must be sourced to setup the correct working environment for the robot;
in most cases, `sources` will look as it does above, except the term
`melodic` may be replaced with the name of another ROS distribution
(e.g., `indigo` or `kinetic`). `launches` gives an ordered list of the XML
launch files that should be used to launch the software for the robot.
For now, each element in this list is an absolute path to a launch file
inside the container. In the near future, support for relative paths
(e.g., name of package + name of launch file) and passing command-line
arguments will be added. There is also an additional `environment` property,
exemplified below, which accepts a mapping from names of environment
variables to their respective values. This is useful in a small number of
cases (e.g., specifying `TURTLEBOT3_MODEL` for TurtleBot3).

```yaml
image: turtlebot3
sources:
- /opt/ros/kinetic/setup.bash
- /ros_ws/devel/setup.bash
environment:
  TURTLEBOT3_MODEL: burger
launches:
- /ros_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_house.launch
```

### Commands

To see a complete list of commands that are supported by ROSDiscover,
run the following:

```
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
```

The `launch` command is used to simulate the effects of a sequence of `roslaunch`
calls for a robot application:

```
$ rosdiscover launch example/fetch.yml
```
