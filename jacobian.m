function func = jacobian(h,VT0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This program is use to find the stability and control derivatives at the
% trim condition

% NUS Mechanical Engineering Final Year Project
% Title: System Identification for an Unmanned Aerial Vehicle
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% -----------------------------------------------------------------------------------
syms a u q t de 

rho = equator(h);

% Data of HALE UAV
% theta = alpha;
% q = 0;
c = 1.74; 
S = 17.1; 
mass = 1248.5;
ly = 4067.5;
g = 9.81;

CL = 6.44*a + 3.8*c/(2*u)*q + 0.355*de;
L = 0.5*rho*u^2*S*CL;

CD = 0.03 + 0.05*CL^2;
D = 0.5*rho*u^2*S*CD; 

CM = 0.05 - 0.683 * a - 9.96*c/(2*u)*q - 0.923*de;
m = 0.5*rho*u^2*S*c*CM;

power = 59164 + 64304*sqrt(rho) - 15312*rho;
T = power/u; % Power required = Thrust * velocity since T=D

func = jacobian([q - (T * sin(a)+L)/(mass*u) + g/u*cos(t-a),
    (((T * cos(a))-D)/mass - g*sin(t-a))/VT0,
    m/ly,
    q], [a u q t de]);
end