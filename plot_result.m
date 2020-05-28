function [] = plot_result(tspan,delta_xsim,delta_xhat,A,Ahat,B,Bhat,method)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Time history plots of measured and estimated responses
% Plot of the actual and estimated parameters

% NUS Mechanical Engineering Final Year Project
% Title: System Identification
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r2d = 180/pi;
d2r = pi/180;
cost_func = 'NRMSE';

fntsz = 18;
figure('units','normalized','outerposition',[0 0 1 1])

subplot(2,2,1)
plot(tspan,delta_xsim(:,1)*r2d,'b','LineWidth',1)
hold on
plot(tspan,delta_xhat(:,1)*r2d,'--r','LineWidth',1);
hold off
title('AOA perturbation against time')
xlabel('Time/s')             
ylabel('Degrees')
fit = goodnessOfFit(delta_xhat(:,1)*r2d,delta_xsim(:,1)*r2d,cost_func);
legend('Simulated Data','Predicted Model')
% legend('Simulated Data',strcat('Predicted Model: ',num2str(round(fit*100,2)),'%'))
grid on
set(gca,'FontSize',fntsz)

subplot(2,2,2)
plot(tspan,delta_xsim(:,2),'b','LineWidth',1)
hold on
plot(tspan,delta_xhat(:,2),'--r','LineWidth',1);
hold off
title('Normalised forward airspeed perturbation against time')
xlabel('Time/s')  
ylabel('m/s','fontsize',fntsz)
fit = goodnessOfFit(delta_xhat(:,2),delta_xsim(:,2),cost_func);
% legend('Simulated Data',strcat('Predicted Model: ',num2str(round(fit*100,2)),'%'))
grid on
set(gca,'FontSize',18)

subplot(2,2,3)
plot(tspan,delta_xsim(:,3)*r2d,'b','LineWidth',1)
hold on
plot(tspan,delta_xhat(:,3)*r2d,'--r','LineWidth',1);
hold off
title('Pitch rate perturbation against time')
xlabel('Time/s') 
ylabel('Degrees/s')
fit = goodnessOfFit(delta_xhat(:,3)*r2d,delta_xsim(:,3)*r2d,cost_func);
% legend('Simulated Data',strcat('Predicted Model: ',num2str(round(fit*100,2)),'%'))
grid on
set(gca,'FontSize',fntsz)

subplot(2,2,4)
plot(tspan,delta_xsim(:,4)*r2d,'b','LineWidth',1)
hold on
plot(tspan,delta_xhat(:,4)*r2d,'--r','LineWidth',1);
hold off
title('Pitch angle perturbation against time')
xlabel('Time/s')             
ylabel('Degrees')
fit = goodnessOfFit(delta_xhat(:,4)*r2d,delta_xsim(:,4)*r2d,cost_func);
% legend('Simulated Data',strcat('Predicted Model: ',num2str(round(fit*100,2)),'%'))
grid on
set(gca,'FontSize',fntsz)

if method == 1
    heading = 'Ordinary Least Squares';
elseif method == 2
    heading = 'Total Least Sqaures';
elseif method == 3
    heading = 'Gauss-Newton';
else 
    heading = 'Levenburg-Marquardt';
end

% Plot comparison of the stability estimates
figure
plot(reshape(A(1:3,1:3)',[9 1]),'ok','MarkerSize',8,'MarkerFaceColor','g')
hold on
plot(reshape(Ahat(1:3,1:3)',[9 1]),'dk','MarkerSize',8,'MarkerFaceColor','m')
title(heading)
axis([0 10 -6 1.5])
xticks([1 2 3 4 5 6 7 8 9])
xticklabels({'Z_\alpha','Z_u','1+Z_q','X_\alpha','X_u','X_q','m_\alpha','m_u','m_q'})
grid on
legend('Simulated','Predicted')
set(gca,'FontSize',18)

% Plot comparison of the control estimates
figure
plot(B(1:3),'ok','MarkerSize',8,'MarkerFaceColor','g')
hold on
plot(Bhat(1:3),'dk','MarkerSize',8,'MarkerFaceColor','m')
title(heading)
axis([0 4 -8 1.5])
xticks([1 2 3])
xticklabels({'Z_\delta_e','X_\delta_e','m_\delta_e'})
grid on
legend('Simulated','Predicted')
set(gca,'FontSize',18)
