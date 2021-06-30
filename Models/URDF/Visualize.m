clc;
clear;
close all;
%% Forward kinematics
robot = importrobot('urdf/NUgus.urdf');
robot.DataFormat = 'column';
old = importrobot('urdf/NUgusOld.urdf');
old.DataFormat = 'column';
%% Show robot
figure;
title('New');
show(robot,'Visuals','on');
figure;
title('Old');
show(old,'Visuals','on');