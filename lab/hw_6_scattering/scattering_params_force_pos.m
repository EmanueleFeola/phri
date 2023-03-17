% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% scattering
b = 100;
scattering_filter_freq = 100;

% delay
% time_delay = 0.0001;
discrete_delay = 100;
Ts = 0.001;

% Input function parameter (sin or step with low pass filter)
A = 5;

% Low pass frequency cuff off
Flp = 0.5;
% Sin frequency
Fc = 0.25;

% Human intention controller (PD)
Ph = 10*1; %5; 
Dh = 20*0.8; %1; 

% % Human impedance parameters
Jh = 1;
Bh = 1; %1.5; 

% % Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 0;
Ds = 0;

% % Master controller
% Bm = 20*0.8;
% Km = 0; % do not use in 2 channel force-pos
% 
% % Slave controller
% Bs = 64; %80/10;
% Ks = 10;

% Master controller
Bm = 20*0.8;
Km = 10*1;

% Slave controller
Bs = 4*Bm; 
Ks = 4*Km; 

% Environment impedance parameters
Be = 100; 
Ke = 200; 
xe = 2;
