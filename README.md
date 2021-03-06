# Kinect

This provides some utilities for using the Kinect data (and the skeleton) with
Linux.

## Installation

Before compiling this ROS stack in the normal way (using catkin_make), it is
important to export the variables LIBFREENECT2 and NITE2 to indicate the
system where libfreenect2 and NITE2 are located.

Assuming that both libraries are located in /home/user/dev/kinect/, the
commands are the following:

```
echo "export LIBFREENECT2=/home/user/dev/kinect/libfreenect2" >> ~/.bashrc
echo "export NITE2=/home/user/dev/kinect/NiTE-Linux-x64-2.2" >> ~/.bashrc
```

In order to use the NiTE utilities (such as the skeleton), the NiTE2 trained
features need to be accessible from the path where the nodes are launched.  The
suggestion is to create a symbolic link to NiTE2 from the kinect_v2 package:

```
roscd kinect_v2
ln -s $NITE2/Samples/Bin/NiTE2
```
and then launch all the kinect related nodes from the root of the kinect_v2
package.


