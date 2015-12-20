# RTAI-ROS

This project adds [ROS](http://www.ros.org) support to [RTAI-Lab](https://www.rtai.org/?About_RTAI-Lab).
As with RTAI-Lab it allows to generate code directly from Matlab/Simulink that can be compiled and executed on the the RTAI real-time Linux.
Unlike RTAI-Lab it is no longer required to use special software like xrtailab to communicate with a running real-time process.
Instead you can use the tools provided by ROS.

Version 1.0 is fully compatible with RTAI-Lab tools. The development version is also fully compatible, but will not automatically start the host interface task to communicate with `xrtailab`.

RTAI-ROS was created as a part of a [master thesis](http://epub.jku.at/urn:nbn:at:at-ubl:1-5222)
 in mechatronics at the [Institute of Robotics](http://www.robotik.jku.at/) at the [Johannes Kepler University Linz](http://www.jku.at/).

See the [documentation](https://rhopfer.github.io/rtairos/) for installation and how to use it.
