function rho = equator(h)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determine the density base on the trimmed heigh at steady flight
% 
% NUS Mechanical Engineering Final Year Project
% Title: System Identification
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% -----------------------------------------------------------------------------------
rhosl = 1.165; % Density at sea level
tsl = 303.15; % Temperature at sea level
grad = 6.51/1000; % Temperature gradient

rho = rhosl*((tsl-grad*h)/tsl)^4.2506; % density at the specific height
end
