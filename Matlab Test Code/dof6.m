clear;
clc;
%建立机器人模型
% theta d a alpha offset
SL1=Link([0 0 0.150 -pi/2 0 ],'standard'); 
SL2=Link([0 0 0.700 0 0 ],'standard');
SL3=Link([0 0 0.109 -pi/2 0 ],'standard');
SL4=Link([0 0.600 0 pi/2  0 ],'standard');
SL5=Link([0 0 0 -pi/2 0 ],'standard');
SL6=Link([0 0.065 0 0 0 ],'standard');
robot=SerialLink([SL1 SL2 SL3 SL4 SL5 SL6],'name','robot');
figure(1),teach(robot);
