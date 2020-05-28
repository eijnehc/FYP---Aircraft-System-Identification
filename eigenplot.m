%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This program is used to obtain the eigenvalues of the aircraft for the
% flight envelope

% NUS Mechanical Engineering Final Year Project
% Title: System Identification for an Unmanned Aerial Vehicle
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

alpha = [2.4:0.2:25]*pi/180;
n = numel(alpha);
solu = [];
x = [];
y = [];

for i = 1:n
x0 = [10000 91 1*pi/180]; % Initial Condition for height u and deltae respectively
X = fsolve(@(x) roots_eq(x,alpha(i)), x0); % Input the density at the particular height to guess the 3 unknowns
a = alpha(i);
t = alpha(i);
u = X(2);
de = X(3); 
q = 0;
AB = jacobian(X(1),u); % Getting matrix A with the symbolic variables
AB_solved = double(subs(AB)); % sub in the trimmed values the stability derivatives and use double to get decimal places
A = AB_solved(:,1:4); % Matrix A
B = AB_solved(:,5); % Matrix B
lambda = eig(A); % get the eigenvalues
solu = [solu lambda]; % store the solutions
end

x = [x real(solu)]; % real part
y = [y imag(solu)]; % imaginary part

hold on
grid on
% title('Plot of Short Period and Phugoid Mode for Equatorial Model')
xlabel('Real')
ylabel('Imaginary')
for i = 1:n
scatter(x(:,i),y(:,i),'b') % plot graph 
end
set(gca,'FontSize',16)