% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% delay
time_delay = 0.1;

% Input function parameter (sin or step with low pass filter)
A = 1;

% Low pass frequency cuff off
Flp = 0.5;
% Sin frequency
Fc = 0.5; 

% Human intention controller (PD)
Ph = 10*1; % questo piu alto ==> x_d = x_m
Dh = 20*0.8;

% Human impedance parameters
Jh = 1;
Bh = 1; 

% Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 0;
Ds = 0;

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
