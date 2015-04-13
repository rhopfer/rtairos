function setup

disp('Setup RTAI-ROS Bridge');

if ispc,
	devpath='\rtw\c\rtairos\devices';
	incpath='\rtw\c\rtairos\include';
else
	devpath='/rtw/c/rtairos/devices';
	incpath='/rtw/c/rtairos/include';
end

devices = [matlabroot, devpath];
includes = [matlabroot, incpath];
addpath(devices);
savepath;

cdpath = pwd;
cd devices;

sfuns = dir('*.c');
for i = 1:length(sfuns)
	name = sfuns(i).name;
	disp(sprintf('* mex %s', name));
    eval(['mex -I''', includes, ''' ', name]);
end

cd ..;

disp('done.')

