% Frequency response magnitudes of the individual terms in the pitching
% moment equation as a function of the input signal frequency.
% Test case specified by Equations (2.6) and (2.7)
%
% Chapter 2: Data Gathering 
% "Flight Vehicle System Identification - A Time Domain Methodology"
% Second Edition
% Author: Ravindra V. Jategaonkar
% published by AIAA, Reston, VA 20191, USA


% State matrix
A = [-1.0027   -0.0023    0.9944         0;...
     0.0378   -0.0003   -0.0004   -0.1071;...
      -5.1600    0.0000   -0.7146         0;...
           0         0    1.0000         0];

% Control Matrix
B = [-0.0548 -0.0038 -6.9732 0]';

% Frequency vector (rad/s)
% W=(0.1:0.1:100);
% W=logspace(-1, 2, 100);

% X-U contribution
C = [0 -0.003 0 0];
D = [0];
SYSXU = ss(A,B,C,D);

% X-A contribution
C = [0.0378 0 0 0];
D = [0];
SYSXA = ss(A,B,C,D);

% Z-U contribution
C = [0 -0.0023 0 0];
D = [0];
SYSZU = ss(A,B,C,D);

% Z-A contribution
C = [-1.0027 0 0 0];
D = [0];
SYSZA = ss(A,B,C,D);


% Z-Q contribution
C = [0 0 0.9944 0];
D = [0];
SYSZQ = ss(A,B,C,D);

% M-Alpha contribution  
C = [0 -5.16 0 0];
D = [0];
SYSMAL = ss(A,B,C,D);

% M-Q contribution  
C = [0 0 -0.7146 0];
D = [0];
SYSMQ = ss(A,B,C,D);

% Bode magnitude plot
bodemag(SYSXU,'b',SYSXA,'g',SYSMAL,'r',SYSMQ,'m',SYSZU,'c',SYSZA,'k',SYSZQ,'y', {0.01 100}); grid
legend
% bodemag(SYSMA,'b',SYSMQ,'g',SYSMDE,'r', {0.01 100}); grid