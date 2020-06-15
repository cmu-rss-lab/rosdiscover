.. -*-restructuredtext-*-

roswire
=======

.. image:: https://travis-ci.org/ChrisTimperley/roswire.svg?branch=master
    :target: https://travis-ci.org/ChrisTimperley/roswire
.. image:: https://badge.fury.io/py/roswire.svg
    :target: https://badge.fury.io/py/roswire
.. image:: https://img.shields.io/pypi/pyversions/roswire.svg
    :target: https://pypi.org/project/roswire
.. image:: https://gitq.com/badge.svg
    :target: https://gitq.com/ChrisTimperley/roswire

Read the `documentation <https://christimperley.github.io/roswire>`_ or check out the `forum <https://gitq.com/ChrisTimperley/roswire>`_
----------------------------------------------------------------------------------------------------------------------------------------

ROSWire is a Python library for static and dynamic analysis of
containerised `Robot Operating System (ROS) <https://ros.org>`_
applications.
Given a `Docker <https://docker.org>`_ image,
ROSWire provides an interface for statically querying the application
(e.g., by automatically discovering its types, packages, messages, service,
actions, etc.), as well as an interface for dynamically generating and
interacting with instances of that application in the form of Docker
containers (e.g., service calls, bag recording, topic publishing and
subscribing, catkin builds, etc.).


Features
--------

* **Does not require ROS to be installed on your machine.**
* **Supports most ROS distributions out of the box (e.g., Groovy, Indigo, Kinetic, Melodic).**
* **Package Discovery:** finds all ROS packages within a Docker image.
* **Definition Discovery:** finds and parses all message, service and
  action formats into readable data structures.
* **Message Serialisation:** converts ROS messages from YAML or binary
  to readable data structures and vice versa.
* **Bag Manipulation:** efficiently parses
  `rosbag <http://wiki.ros.org/rosbag>`_ files, which can then be inspected,
  manipulated, and saved to disk.
* **Bag Playback:** safely replay bag files inside containers.


Installation
------------

roswire requires Python 3.6+

To avoid interfering with the rest of your system (i.e., to avoid Python's
equivalent of DLL hell), we strongly recommend that
ROSWire is installed within a
`virtualenv <https://virtualenv.pypa.io/en/latest/>`_ or
`pipenv <https://pipenv.readthedocs.io/en/latest/>`_ (pipenv is preferred).

From within the virtual environment (i.e., the `virtualenv` or `pipenv`),
the latest stable release of ROSWire on `PyPI <https://pypi.org>`_
can be installed via:

.. code:: shell

   (roswire) $ pip install roswire

ROSWire can also be installed from source:

.. code:: shell

   $ git clone git@github.com:ChrisTimperley/roswire roswire
   $ cd roswire
   $ pipenv shell
   (roswire) $ pip install .
