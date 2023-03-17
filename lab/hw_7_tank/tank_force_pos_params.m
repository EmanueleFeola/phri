clc;
clear;

%Master controller
Km = 50;
Bm = 10;

%Slave controller
Ks = 50;
Bs = 10;

% Tank variables
beta = 0.1;
alpha = 0.1;
H_D = 100;

initial_tank_energy_master = 100 * 10;
initial_tank_energy_slave = 100 * 10;

% parametri come nell'assignment dello scattering
% delay
delay = 10;
Ts = 0.001;

% Input function parameter (sin or step with low pass filter)
% pos-pos no contact
A = 1;
xe = 0.5;

% Low pass frequency cuff off
Flp = 5;
% Sin frequency
Fc = 0.5;

% Human intention controller (PD)
Ph = 10*1; %5; 
Dh = 20*0.8; %1; 

% % Human impedance parameters
Jh = 1;
Bh = 1.5; 

% % Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 0;
Ds = 0;

% Environment impedance parameters
Be = 100; 
Ke = 200; 
