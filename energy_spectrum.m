%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This program is used to compute the energy spectrum of an arbitary
% multistep input signal

% NUS Mechanical Engineering Final Year Project
% Title: System Identification for an Unmanned Aerial Vehicle
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs: 
%           V   Vector with amplitudes of step signals
%           dt  Length of one time step in seconds
%           wa  Starting frequency
%           we  Final frequency
%           nw  Number of frequency points (default=100)
% Outputs:
%           w   Frequencies [1/s]
%           E   Energy spectrum
%


function [w,E] = energy_spectrum(V,dt,wa,we,nw)

% Initialization
n = length(V);                    % Number of time steps
w = linspace(wa, we, nw);         % Frequency in 1/s

% Computation of Energy for specified frequency range
E = V*V';
for j=1:n-1
    E = E + 2 * cos(j*w*dt) .* (V(1:n-j) * V(1+j:n)');
end

if w(1) == 0
    E(1)     = E(1)*dt^2;         % Limit value of (1-cos(w*dt)/w^2)*2 for w->0 = dt^2
    E(2:end) = 2 * E(2:end) .* (1 - cos(w(2:end)*dt)) ./ (w(2:end).^2);
else
    E = 2 * E .* (1 - cos(w*dt)) ./ (w.^2);
end

return;
