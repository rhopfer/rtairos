# RTAI-ROS

This project adds [ROS](http://www.ros.org) support to [RTAI-Lab](https://www.rtai.org/?About_RTAI-Lab).
As with RTAI-Lab it allows to generate code directly from Matlab/Simulink that can be compiled and executed on the the RTAI real-time Linux.
Unlike RTAI-Lab it is no longer required to use special software like xrtailab to communicate with a running real-time process.
Instead you can use the tools provided by ROS.
The current version is also fully compatible with RTAI-Lab tools, but this will probably removed in future versions.

RTAI-ROS was created as a part of my master thesis in mechatronics at the [Institute of Robotics](http://www.robotik.jku.at/) at the [Johannes Kepler University Linz](http://www.jku.at/).

## Installation

1. Copy the whole directory to `$(MATLABROOT)/rtw/c/` of your Matlab installation.
2. In Matlab change to rtairos directory and run *setup*:
  ```
>> matlabroot
ans =
/opt/matlab
>> cd /opt/matlab/rtw/c/rtairos
>> setup
  ```
  This compiles all S-Functions of RTAI-ROS and adds the devices subdirectory to the search path.
  Make sure you have write permissions on *rtairos*.
   
3. In **rtairos.tmf** update the variables MATLAB_ROOT, LINUX_HOME, RTAIDIR, COMEDI_HOME, and ROS_HOME to your needs.
  If you use Windows for code generation remove also the following lines:
  ```
|>START_EXPAND_RULES<|%.o : |>EXPAND_DIR_NAME<|/%.c
    gcc -c $(CFLAGS) $<

|>END_EXPAND_RULES<|
  ```

## Use

The Matlab/Simulink blocks to connect with ROS are accessable via the Simulink Library Browser.
To generate code open the RTW configuration (Simulation → Configuration Parameters → Real-Time Workshop) of your model and set the system target file to **rtairos.tlc**.

For the moment there is only a german documentation extracted from the master thesis.
