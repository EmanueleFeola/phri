% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% scattering
b = 1;
scattering_filter_freq = 1000000;

% delay
time_delay = 0.0001;
discrete_delay = 1;
Ts = 0.0001;

% Input function parameter (sin or step with low pass filter)
A = 3;

% Low pass frequency cuff off
Flp = 5;
% Sin frequency
Fc = 1;

% Human intention controller (PD)
Ph = 5; %10*1; 
Dh = 1; %0*0.8; 

% % Human impedance parameters
Jh = 1;
Bh = 1.5; 

% % Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 0;
Ds = 0;

% Master controller
Bm = 20*0.8;
Km = 0; % do not use in 2 channel force-pos

% Slave controller
Bs = 64; %80/10;
Ks = 10;

% Environment impedance parameters
Be = 5; 
Ke = 20; 
xe = 5;
