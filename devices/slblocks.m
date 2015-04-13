function blkStruct = slblocks
%SLBLOCKS Defines the block library for a specific Toolbox or Blockset.
% See: matlabroot/toolbox/simulink/blocks/slblocks.m

% Name of the subsystem which will show up in the SIMULINK Blocksets
% and Toolboxes subsystem.
blkStruct.Name = ['RTAI-ROS' sprintf('\n') 'Devices'];

% The function that will be called when the user double-clicks on
% this icon.
blkStruct.OpenFcn = 'rtairos';

% The argument to be set as the Mask Display for the subsystem.  You
% may comment this line out if no specific mask is desired.
blkStruct.MaskDisplay = '';

