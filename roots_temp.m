function X = roots_temp(x,alpha)
% Unknown variables are x[height u deltae]

% Conditions
theta = alpha;
q = 0;

% Data of HALE UAV
rhosl = 1.225; % Density at sea level
tsl = 288; % Temperature at sea level
grad = 6.51/1000; % Temperature gradient

rho = rhosl*((tsl-grad*x(1))/tsl)^4.2506; % for 

c = 1.74; 
S = 17.1; 
mass = 1248.5;
ly = 4067.5;
power = 59164 + 64304*sqrt(rho) - 15312*rho;
T = power/x(2); % Power required = drag * velocity
g = 9.81;

CL = 6.44*alpha + 3.8*c/(2*x(2))*q + 0.355*x(3);
L = 0.5*rho*x(2)^2*S*CL;

CD = 0.03 + 0.05*CL^2;
D = 0.5*rho*x(2)^2*S*CD; % Thrust = Drag for levelled flight

CM = 0.05 - 0.683 * alpha - 9.96*c/(2*x(2))*q - 0.923*x(3);
m = 0.5*rho*x(2)^2*S*c*CM;

% Decoupled nonlinear longitudinal equation of motion
X(1) = q - (T*sin(alpha) + L)/(mass * x(2)) + g/x(2)*cos(theta-alpha);
X(2) = (T*cos(alpha)-D)/mass - g*sin(theta-alpha);
X(3) = m / ly;
X(4) = q;
end
