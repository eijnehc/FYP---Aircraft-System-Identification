%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a main program to test system identification algorithms for a
% 3 DOF Aircraft Model
% There are two types of inputs you can choose to deflect the elevators
% Next,there is an option to add noise to the system
% Lastly, choose either of the 5 parameter estimation methods

% Aircraft Hale UAV: Longitudinal-directional motion  
% states  - aoa, airspeed, pitch rate, pitch angle
% outputs - aoa, airspeed, pitch rate, pitch angle
% input  - elevator

% NUS Mechanical Engineering Final Year Project
% Title: System Identification 
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

% -----------------------------------------------------------------------------------
%% Obtaining the trim condition
r2d = 180/pi;
d2r = pi/180;
alpha = [2.4:0.2:25]*pi/180;

i = 20; % At index 20 of the alpha vector
x0_est = [10000 91 -1*pi/180]; % Make a guess of the parameters of height(m) u(m/s) and deltae(rad) respectively at the particular AOA
trim = fsolve(@(x) roots_eq(x,alpha(i)), x0_est); % Fsolve to determine the trim conditions

a = alpha(i); % Trim AOA
t = alpha(i); % Trim pitch angle
h = trim(1); % Height
u = trim(2); % Trim total velocity VT0
de = trim(3); % Trim elevator deflection
q = 0; % Pitch rate
x0 = [a; u; q; t]; % Trimmed Conditions

%% Linearising around the trim condition
AB = jacobian(trim(1),u); % State space model at the trim height
AB_solved = double(subs(AB)); % sub in the trimmed values to get stability derivatives and use double to get decimal places
A = AB_solved(:,1:4); % Matrix for state derivatives
B = AB_solved(:,5); % Matrix for control derivatives

%% Stability Analysis
stab = eig(A);
if (stab < 0) % if both the real and imag part are negative, sys is stable
    disp(stab);
    disp('All roots are on the negative plane');
    disp('Longitudinal dynamics of aircraft is stable');  
else 
    disp(stab)
    disp('Not all roots are on the negative plane')
    disp('Longitudinal dynamics of aircraft is not stable')
end

%% Using ODE45 to obtain the x and x_dot 

% Test Case 1 - perturb the elevators for 2 seconds
% fprintf('\nTest case 1: Single doublet input \nTest case 2: 3-2-1-1 input\n');
% testcase = input('Enter number corresponsing to test case: ');
degree = input('Input elevator deflection perturbation (Between -2 to 2 degrees): ');
perturb_de = degree*d2r; % Note: sign convention postive for downward elevator deflection

tspan = 0:0.025:60;

del_e = elevator_deflection(tspan,perturb_de,de);
delta_u = del_e - repmat(de,length(tspan),1);

% 3211-Signal
dt3211 = 0.7;                                      % time step
V      = [0.7 0.7 0.7 -0.7 -0.7 0.7 -0.7];         % amplitudes
[w3,E3] = energy_spectrum(V,dt3211,0.0,20,100);

% doublet-Signal
doublet = 0.7;                         % time step
V      = [0.7 0.7 -0.7 -0.7];         % amplitudes
[w4,E4] = energy_spectrum(V,doublet,0.0,20,100);

% Times histories of the states
% x0(2) = 1;
sol = ode45(@(t,x) xdot_decoupled(x,de,perturb_de,h,t),tspan,x0);

% x and its derivatives wrt time
[xsim,delta_xdot] = deval(sol,tspan);
delta_xsim = xsim' - repmat(x0',length(tspan),1);

% Normalised velocity to freestream velocity
delta_xsim(:,2) = delta_xsim(:,2) / u;
delta_xdot(2,:) = delta_xdot(2,:) / u;
xsim(2,:) = xsim(2,:) / u;

%% Corrupt the state time history with 10% random measurement noise
fprintf('\nInclude noise in the output system? \n1.Yes \n2.No\n');
noise_option = input('Enter digit corresponding to your choice: ');

% initialize the generator using a seed of 1 to repeat arrays of random numbers
rng(1);

if noise_option == 1
% noise in input measurements
in_amplitude = [max(delta_xsim(:,1)); max(delta_xsim(:,2)); max(delta_xsim(:,3)); max(delta_xsim(:,4))];
in_noise = 0.1 * in_amplitude' .* randn(size(delta_xsim)); 
delta_xsim = delta_xsim + in_noise;

% in_amplitude_act = [max(xsim(1,:)); max(xsim(2,:)); max(xsim(3,:)); max(xsim(4,:))];
% in_noise_act = 0.1 * in_amplitude .* randn(size(xsim)); 
% xsim_act = xsim + in_noise_act;

% Restore the original generator settings by calling rng
rng(1);

% noise in output measurements
out_amplitude = [max(delta_xdot(1,:)); max(delta_xdot(2,:)); max(delta_xdot(3,:)); max(delta_xdot(4,:))];
out_noise = 0.05 * out_amplitude .* randn(size(delta_xdot)); 
delta_xdot = delta_xdot + out_noise;
end

%% Parameter estimation methods
X = [delta_xsim'; delta_u']; % Regressor Matrix
X = X';
Y = delta_xdot';

str = 'System Identification Methods';
idx = ~isstrprop(str,'wspace'); % detect spaces
idy = idx | [idx(2:end),true]&[true,idx(1:end-1)]; % ignore single space
und = char(32*~idy); % define output spaces
und(idy) = '-'; % add whatever character to use as the underline
fprintf('%s\n%s\n',und,str,und) % print
fprintf('1.Ordinary Least Square \n2.Total Least Square \n3.Gauss-Newton \n4.Levenberg-Marquardt\n')
method = input('Enter digit corresponding to the method: ');

if method == 1
    % Ordinary least square method
    % sacd = inv(X'*X)*X'*Y;
    sacd = X\Y;
    sacd = sacd';
    
%     % Recursive Least Square Method
%     [Ndata, nparam] = size(X);
%     P = eye(nparam)*1.0e+06; % Initial P matrix
%     lamda = 1;	% forgetting factor
%     theta = zeros (nparam,1);
%     
%     for index = 1:4
%         for k=1:Ndata 
% 
%             x = X(k,:)';
% 
%             % Gain K(k+1)
%             Kgain = P*x / (lamda + x'*P*x );                
% 
%             % Estimate parameters theta(k+1)
%             y(k)  = theta' * x;                            
%             theta = theta + Kgain * ( Y(k,index) - y(k) );
% 
%             % P(k+1)
%             P = ( P - Kgain * x' * P  ) / lamda ;           
% 
%             % Theta(k,:) = theta'; should i show how the para converge?
%         end
%         sacd(index,:) = theta;
%         P = eye(nparam)*1.0e+06; % Initial P matrix
%         theta = zeros (nparam,1);
%     end
    
elseif method == 2
    % Total least sqaure method   
    % a' = Z_a * a + Z_u * u + (1+Z_q) * q + 0 * t
    Z1 = Y(:,1);
    nq = size(X,2);
    nq1 = nq + 1;
    XZcomp = [X Z1];
    [ue,se,ve] = svd(XZcomp,0);
    a_Par = -(ve(1:nq,nq1)/ve(nq1,nq1));
    
    % u' = X_a * a + X_u * u + X_q * q + -g/Vt0 * t
    Z2 = Y(:,2);
    nq = size(X,2);
    nq1 = nq + 1;
    XZcomp = [X Z2];
    [ue,se,ve] = svd(XZcomp,0);
    u_Par = -(ve(1:nq,nq1)/ve(nq1,nq1));
    
    % q' = M_a * a + M_u * u + M_q * q + 0 * t
    Z3 = Y(:,3);
    nq = size(X,2);
    nq1 = nq + 1;
    XZcomp = [X Z3];
    [ue,se,ve] = svd(XZcomp,0);
    q_Par = -(ve(1:nq,nq1)/ve(nq1,nq1));
    
    % t' = 0 * a + 0 * u + 1 * q + 0 * t
    Z4 = Y(:,4);
    nq = size(X,2);
    nq1 = nq + 1;
    XZcomp = [X Z4];
    [ue,se,ve] = svd(XZcomp,0);
    t_Par = -(ve(1:nq,nq1)/ve(nq1,nq1));
    
    sacd = [a_Par'; u_Par'; q_Par'; t_Par'];

else
    % Estimate Structured Continuous-Time SS Models using sysid toolbox
    Ahat = [-1 0 1 0;
            0.03 -0.02 -0.05 -0.1;
            -5 0 -0.5 0;
            0 0 1 0];
    Bhat = [0.05;0.3;6.9;0];
    Chat = eye(4);
    Dhat = 0;

    m = idss(Ahat,Bhat,Chat,Dhat,'Ts',0);

    % Specify the parameter values in the structure matrices that you do not want to estimate
    S = m.Structure;
    S.A.Free(1,4) = false;
    S.A.Free(2,4) = false;
    S.A.Free(3,4) = false;
    S.A.Free(4,:) = false;
    S.B.Free(4,1) = false;
    S.C.Free = false;
    m.Structure = S;

    opt = ssestOptions;
    opt.InitialState = 'estimate';
    opt.Display = 'on';
    opt.EnforceStability = true;
    opt.Focus = 'simulation';
    opt.SearchOptions.MaxIterations = 50;
    
    if method == 3
    opt.SearchMethod = 'gn';
    elseif method == 4
    opt.SearchMethod = 'lm';
    end 

    data = iddata(X(:,1:4),X(:,5),0.025);

    m = ssest(data,m,opt);
end

% matrices A and B estimation
if method <= 2
    Ahat = sacd(:,1:4);
    Bhat = sacd(:,5);
else 
    Ahat = m.A;
    Bhat = m.B;
end

disp('A matrix'); 
disp(A);
disp('Estimated A matrix');
disp(Ahat);

%% Simulation of identified model
% delta_xdot = A * delta_x + B * delta_u
% y = C * delta_x

C = eye(4); % fourth order identity matrix
sys_hat = ss(Ahat,Bhat,C,[]); % continuous estimated state-space model 
delta_xhat = lsim(sys_hat,delta_u,tspan); 

%% Model Validation

% Theil's Inequality Coefficient 
e = delta_xsim - delta_xhat;
esq = e.^2;
ebar = mean(esq);
eroot = sqrt(ebar);
delta_xsim_sq = delta_xsim.^2;
delta_xsim_bar = mean(delta_xsim_sq);
delta_xsim_root = sqrt(delta_xsim_bar);
delta_xhat_sq = delta_xhat.^2;
delta_xhat_bar = mean(delta_xhat_sq);
delta_xhat_root = sqrt(delta_xhat_bar);
tic = eroot./(delta_xsim_root + delta_xhat_root);

str = 'Theil Inequality Coefficient';
idx = ~isstrprop(str,'wspace'); % detect spaces
idy = idx | [idx(2:end),true]&[true,idx(1:end-1)]; % ignore single space
und = char(32*~idy); % define output spaces
und(idy) = '-'; % add whatever character to use as the underline
fprintf('%s\n%s\n',und,str,und) % print

fprintf('Angle of attack: %0.4f\n', tic(1));
fprintf('Forward airspeed: %0.4f\n', tic(2));
fprintf('Pitch angle: %0.4f\n', tic(3));
fprintf('Pitch rate: %0.4f\n', tic(4));

% Goodness of fit
cost_func = 'NMSE';
fit = goodnessOfFit(delta_xhat*r2d,delta_xsim*r2d,cost_func);
% fit = fitlm(delta_xhat*r2d,delta_xsim*r2d)*100;
fit = round(fit,2);

str = 'Goodness of Fit';
idx = ~isstrprop(str,'wspace'); % detect spaces
idy = idx | [idx(2:end),true]&[true,idx(1:end-1)]; % ignore single space
und = char(32*~idy); % define output spaces
und(idy) = '-'; % add whatever character to use as the underline
fprintf('%s\n%s\n',und,str,und) % print

disp(['Angle of attack: ',num2str(fit(1))]);
disp(['Forward airspeed: ',num2str(fit(2))]);
disp(['Pitch Angle: ',num2str(fit(3))]);
disp(['Pitch rate: ',num2str(fit(4))]);

%% Plot Graphs
% Plot the control time history
% plot(tspan,del_e.*r2d,'r','LineWidth',2)
% axis([0 30 -4 1]);
% title('Elevator control input against time','fontsize',14)
% % title ('3-2-1-1')
% xlabel('time/s')
% ylabel('elevator deflection/degrees')
% set(gca,'FontSize',16)

% % plot energy spectrum
% figure,plot(w3,E3/dt3211^2,'r','LineWidth',2);
% hold on
% plot(w4,E4/doublet^2,'b','LineWidth',2);
% axis([0 12, 0 10]);
% xlabel('normalised frequency (rad/s)')
% ylabel('E/\Deltat2')
% grid on
% legend('3-2-1-1','doublet')
% set(gca,'FontSize',16)
% hold off

% % Compare response of simulated data to identified
plot_result(tspan,delta_xsim,delta_xhat,A,Ahat,B,Bhat,method)
% 
% % get the coefficient plots
% CLCD(delta_xsim,delta_xhat,delta_u,u,method);

%% For Simulink
ttoo = elevator_deflection(tspan,perturb_de,de) - repmat(de,length(tspan),1);

testcase1 = timeseries(ttoo,tspan); %3-2-1-1
% del_e = [tspan del_e];
% 
elevator_trim = repmat(de,length(tspan),1);
elevator_trim = timeseries(elevator_trim, tspan);

 
