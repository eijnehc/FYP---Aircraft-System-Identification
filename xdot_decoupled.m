function dydt = xdot_decoupled(y,de,perturb_de,h,t,testcase)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longitudinal flight simulation using inputs from the test case to obtain
% the states and state derivatives
 
% NUS Mechanical Engineering Final Year Project
% Title: System Identification for an Unmanned Aerial Vehicle
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if testcase == 1
%     if t < 5
%        deflection = de;
%     elseif t < 7
%        deflection = de + perturb_de;
%     elseif t < 9
%        deflection = de - perturb_de;
%     else 
%        deflection = de;
%     end
% elseif testcase == 2
if t < 5
   deflection = de;
elseif t < 7.1
   deflection = de + perturb_de;
elseif t < 8.5
   deflection = de - perturb_de;
elseif t < 9.2
   deflection = de + perturb_de;
elseif t < 9.9
   deflection = de - perturb_de;
else 
   deflection = de;
end


rho = equator(h);

c = 1.74; 
S = 17.1; 
mass = 1248.5;
ly = 4067.5;
g = 9.81;

CL = 6.44*y(1) + 3.8*c/(2*y(2))*y(3) + 0.355*deflection;
L = 0.5*rho*y(2)^2*S*CL;

CD = 0.03 + 0.05*CL.^2;
D = 0.5*rho*y(2)^2*S*CD; 

CM = 0.05 - 0.683 * y(1) - 9.96*c/(2*y(2))*y(3) - 0.923*deflection;
m = 0.5*rho*y(2)^2*S*c*CM;

power = 59164 + 64304*sqrt(rho) - 15312*rho;
T = power/y(2); % Power required = Thrust * velocity since T=D


dydt = zeros(4,1);
dydt(1) = y(3) - (T*sin(y(1)) + L)/(mass * y(2)) + g/y(2)*cos(y(4)-y(1));
dydt(2) = ((T*cos(y(1))-D)/mass - g*sin(y(4)-y(1)));
dydt(3) = m / ly;
dydt(4) = y(3);
end