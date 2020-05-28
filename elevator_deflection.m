function input = elevator_deflection(time,perturb_de,de)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Time history plots of elevator deflection
% 
% NUS Mechanical Engineering Final Year Project
% Title: System Identification for an Unmanned Aerial Vehicle
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialise empty array
input = zeros(length(time),1);

% 3-2-1-1
for i = 1:length(time)
    if time(i) < 5
        input(i,1) = de;
    elseif time(i) < 7.1
        input(i,1) = de + perturb_de;
    elseif time(i) < 8.5
        input(i,1) = de - perturb_de;
    elseif time(i) < 9.2
        input(i,1) = de + perturb_de;
    elseif time(i) < 9.9
        input(i,1) = de - perturb_de;
    else 
        input(i,1) = de;
    end
end 

% % doublet
% for i = 1:length(time)
%     if time(i) < 5
%         input(i,1) = de;
%     elseif time(i) < 7
%         input(i,1) = de + perturb_de;
%     elseif time(i) < 9
%         input(i,1) = de - perturb_de;
%     else 
%         input(i,1) = de;
%     end
% end 



       