Files
=====

* ``ros_block.c``: Helper functions used by ROS blocks
* ``rtairos_genfiles.tlc``: Additional script for the Target Language Compiler; creates the symbolic link on unix systems.
* ``rtairos.tlc``: Main script for the Target Language Compiler
* ``rtairos.tmf``: Makefile template
* ``rtmain.cpp``: Main program
* ``setup.m``: Matlab installation script
* ``devices/``
    * ``rtairos.mdl``: Simulink library
    * S-Functions for Comedi:
        * ``sfun_comedi_counter_read.c``: Read counter
        * ``sfun_comedi_data_read.c``: Read analog input
        * ``sfun_comedi_data_write.c``: Write analog output
        * ``sfun_comedi_dio_read.c``: Read digital input
        * ``sfun_comedi_dio_write.c``: Write digital output
    * ``sfun_rtai_fifo.c``: S-Function to write FIFOs
    * S-Functions for ROS:
        * ``sfun_ros_config.c``: Config block
        * ``sfun_ros_joint_state.c``: Joint state publisher
        * ``sfun_ros_joy.c``: Joystick subscriber
        * ``sfun_ros_log.c``: Logger
        * ``sfun_ros_publisher.c``: Publisher
        * ``sfun_ros_service.c``: Service
        * ``sfun_ros_subscriber.c``: Subscriber
        * ``sfun_ros_tf.c``: Transformation publisher
    * S-Functions for RTAI-Lab (see RTAI-Lab documentation):
        * ``sfun_rtai_automatic_log.c``
        * ``sfun_rtai_led.c``
        * ``sfun_rtai_log.c``
        * ``sfun_rtai_meter.c``
        * ``sfun_rtai_scope.c``
        * ``sfun_rtai_synchronoscope.c``
    * ``slblocks.m``: Configuration of the simulink library
* ``examples/``
    * ``rtaitest.mdl``: RTAI-Lab example
    * ``rosdemo.mdl``: Example model; see :ref:`Usage`
    * ``rosdemo.perspective``: A example perspective for rqt.
* ``include/``
    * ``ros_block.h``: Helper functions used by ROS blocks
    * ``ros_defines.h``: Defines and structs used by the main program and blocks.
