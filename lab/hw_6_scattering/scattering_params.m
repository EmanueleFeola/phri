% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% scattering
b = 1;
scattering_filter_freq = 10;

% delay
time_delay = 0;

% Input function parameter (sin or step with low pass filter)
A = 2;

% Low pass frequency cuff off
Flp = 5;
% Sin frequency
Fc = 1;

% Human intention controller (PD)
Ph = 10; %10*1; 
Dh = 0.1; %0*0.8; 

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
Km = 10*1;

% Slave controller
Bs = 80;
Ks = 100;

% Environment impedance parameters
Be = 100; 
Ke = 200; 
xe = 5;
