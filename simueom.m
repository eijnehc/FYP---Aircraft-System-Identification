function [sys,x0,str,ts]= simueom(t,x,u,flag,a,v,q,theta,h)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Use the S function block in Simulink is used to linearised the non-linear 
% model

% NUS Mechanical Engineering Final Year Project
% Title: System Identification for an Unmanned Aerial Vehicle
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch flag
    case 0  % initialize
        str=[];
        ts = [0 0];
s= simsizes;
            s.NumContStates = 4;
            s.NumDiscStates = 0;
            s.NumOutputs = 4;
            s.NumInputs = 6;
            s.DirFeedthrough = 0;
            s.NumSampleTimes = 1;
            sys = simsizes(s);
            x0 = [a,v,q,theta];
    case 1
        de  = u(1);
        sys = xdot_decoupled1(t,x,de,h);
    case 3  % output
        sys = x;
case {2 4 9}
        sys =[];
        
    otherwise
        error(['unhandled flag =',num2str(flag)])
end
