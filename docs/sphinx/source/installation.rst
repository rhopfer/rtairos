Installation
============

1. Copy the ``rtairos`` directory to ``$(MATLABROOT)/rtw/c/``, where ``$(MATLABROOT)`` is the root of your Matlab installation.
2. In Matlab change to ``rtairos`` directory and run ``setup`` ::

     >> matlabroot
     ans =
     /opt/matlab
     >> cd /opt/matlab/rtw/c/rtairos
     >> setup

   This compiles all S-Functions of RTAI-ROS and adds the devices subdirectory to the search path.
   Make sure you have write permissions on ``rtairos``.

3. In ``rtairos.tmf`` change the variables ``MATLAB_ROOT``, ``LINUX_HOME``, ``RTAIDIR``, ``COMEDI_HOME``, and ``ROS_HOME`` to your needs.
   Be sure to set it for the target system where you want to compile your code.

   * During code generation the variable ``|>MATLAB_ROOT<|`` will be expanded by the TLC to the Matlab installation directory.
     On a linux system this is normally ``/opt/matlab`` (sometimes ``/usr/local/matlab``).
   * You can leave the variables ``LINUX_HOME``, ``RTAIDIR``, and ``COMEDI_HOME`` unchanged if ``rtai-config`` is in the search path and can be found during build.

   If you use Windows for code generation remove also the following lines::

     |>START_EXPAND_RULES<|%.o : |>EXPAND_DIR_NAME<|/%.c
         gcc -c $(CFLAGS) $<

     |>END_EXPAND_RULES<|


Uninstallation
--------------

To uninstall just remove the ``rtairos`` directory. Matlab normally removes invalid directories from search path.
To remove the ``rtairos`` directory manually from search path, run ::

    >> rmpath $(MATLABROOT)/rtw/c/rtairos
