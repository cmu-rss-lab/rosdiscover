#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/ros_ws/devel/setup.bash"

# first, we create a directory named after the robot to store its
# description and its associated meshes and materials
mkdir /ros_ws/src/fetch_gazebo/fetch_gazebo/robots/fetch

# next, we transform the Xacro file for the robot into a pure URDF file and
# store that file in the newly created robot directory
pushd /ros_ws/src/fetch_gazebo/fetch_gazebo/robots/fetch
xacro --inorder /ros_ws/src/fetch_gazebo/fetch_gazebo/robots/fetch.gazebo.xacro > fetch.gazebo.urdf

# we then need to transform all "package://" paths into "model://" paths to
# ensure gzweb compatibility
sed -i "s#package://#model://#g" fetch.gazebo.urdf

# if you take a look at the resulting paths, you'll notice that they all begin
# with "model://fetch_description/". This is problematic. The meshes and
# materials for the Fetch robot are not stored in the same package as its Xacro
# file. Because of this, GzWeb will fail to locate and build those meshes.
# We rectify the situation by first symlinking the meshes directory into the
# newly created robot directory.
ln -s /ros_ws/src/fetch_ros/fetch_description/meshes meshes

# We then update the URDF file to point to "fetch" (which is essentially
# a copy of the robot directory that we are creating)
sed -i "s#fetch_description#fetch#g" fetch.gazebo.urdf

# To advertise our new robot model directory to GzWeb, we need to add a
# model.config file at its root.
# (CT slightly suspects that the contents of this file don't actually matter.)
echo "<?xml version="1.0"?>
<model>
<name>fetch</name>
<version>1.0</version>
<sdf version="1.4">fetch.gazebo.urdf</sdf>
</model>" > model.config

# We also need to update the XML launch file for the Fetch simulator to use our
# new, pure URDF file
sed -i "s#fetch.gazebo.xacro#fetch/fetch.gazebo.urdf#g" /ros_ws/src/fetch_gazebo/fetch_gazebo/launch/include/fetch.launch.xml

# Workaround: We need to replace model:// paths with absolute paths
sed -i "s#model://#/ros_ws/src/fetch_gazebo/fetch_gazebo/robots/#g" /ros_ws/src/fetch_gazebo/fetch_gazebo/robots/fetch/fetch.gazebo.urdf

# Almost done!
# We now update the Gazebo model path to include the robot directory that we
# just created, as well as some existing models that are used for the simulated
# environment. (Note that each model directory on the path should contain a
# model.config file.)
#
# BEWARE: the value of this environment variable will not persist beyond the lifetime
# of the shell.
export GAZEBO_MODEL_PATH="/ros_ws/src/fetch_gazebo/fetch_gazebo/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_MODEL_PATH="/ros_ws/src/fetch_gazebo/fetch_gazebo/robots:${GAZEBO_MODEL_PATH}"

# Now that everything is in place, let's bake our models into the
# /opt/gzweb/http/client/assets directory:
/opt/gzweb/deploy.sh -m local 
