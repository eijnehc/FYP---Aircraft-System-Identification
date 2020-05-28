%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This program is used to obtain the flight envelope of the aircraft

% NUS Mechanical Engineering Final Year Project
% Title: System Identification for an Unmanned Aerial Vehicle
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

alpha = [2.4:0.2:25]*pi/180;
solution_eq = [];
solution_temp = [];
n = numel(alpha);

for i = 1:n
x0 = [2000 75 1*pi/180]; % Initial Condition for height u and deltae respectively
X_eq = fsolve(@(x) roots_eq(x,alpha(i)), x0); %input the density at the particular height to guess the 3 unknowns
X_temp = fsolve(@(x) roots_temp(x,alpha(i)), x0);
solution_eq = [solution_eq;X_eq];
% solution_temp = [solution_temp;X_temp];
end

plot(solution_eq(:,2),solution_eq(:,1),'b','LineWidth',2)
hold on
% plot(solution_temp(:,2),solution_temp(:,1))
grid on
% legend('Equatorial','Temperate')
title('Flight Envelope at equator')
xlabel('Velocity ms^-1')
ylabel('Height/m')
set(gca,'FontSize',16)















