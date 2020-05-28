function [] = plot_result1(tspan,delta_xsim,delta_xhat1,delta_xhat2,delta_xhat3,delta_xhat4)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Time history plots of measured and estimated responses
% Plot of the actual and estimated parameters

% NUS Mechanical Engineering Final Year Project
% Title: System Identification
% Code by: Chen Jie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r2d = 180/pi;
d2r = pi/180;

fntsz = 18;
figure('units','normalized','outerposition',[0 0 1 1])

subplot(2,2,1)
plot(tspan,delta_xsim(:,1)*r2d,'k','LineWidth',1)
hold on
plot(tspan,delta_xhat1(:,1)*r2d,'--r','LineWidth',1);
plot(tspan,delta_xhat2(:,1)*r2d,'--b','LineWidth',1);
plot(tspan,delta_xhat3(:,1)*r2d,'--m','LineWidth',1);
% plot(tspan,delta_xhat4(:,1)*r2d,'--c','LineWidth',1);
hold off
title('AOA perturbation against time')
xlabel('Time/s')             
ylabel('Degrees')
% fit = goodnessOfFit(delta_xhat(:,1)*r2d,delta_xsim(:,1)*r2d,cost_func);
legend('Simulated Data','Ordinary Least Squares','Total Least Squares','Gauss-Newton')%, 'Levenburg-Marquardt')
% legend('Simulated Data',strcat('Predicted Model: ',num2str(round(fit*100,2)),'%'))
grid on
set(gca,'FontSize',fntsz)

subplot(2,2,2)
plot(tspan,delta_xsim(:,2),'k','LineWidth',1)
hold on
plot(tspan,delta_xhat1(:,2),'--r','LineWidth',1);
plot(tspan,delta_xhat2(:,2),'--b','LineWidth',1);
plot(tspan,delta_xhat3(:,2),'--m','LineWidth',1);
% plot(tspan,delta_xhat4(:,2),'--c','LineWidth',1);
hold off
title('Normalised forward airspeed perturbation eagainst time')
xlabel('Time/s')  
% ylabel('m/s','fontsize',fntsz)
% fit = goodnessOfFit(delta_xhat(:,2),delta_xsim(:,2),cost_func);
% legend('Simulated Data',strcat('Predicted Model: ',num2str(round(fit*100,2)),'%'))
grid on
set(gca,'FontSize',18)

subplot(2,2,3)
plot(tspan,delta_xsim(:,3)*r2d,'k','LineWidth',1)
hold on
plot(tspan,delta_xhat1(:,3)*r2d,'--r','LineWidth',1);
plot(tspan,delta_xhat2(:,3)*r2d,'--b','LineWidth',1);
plot(tspan,delta_xhat3(:,3)*r2d,'--m','LineWidth',1);
% plot(tspan,delta_xhat4(:,3)*r2d,'--c','LineWidth',1);
hold off
title('Pitch rate perturbation against time')
xlabel('Time/s') 
ylabel('Degrees/s')
% fit = goodnessOfFit(delta_xhat(:,3)*r2d,delta_xsim(:,3)*r2d,cost_func);
% legend('Simulated Data',strcat('Predicted Model: ',num2str(round(fit*100,2)),'%'))
grid on
set(gca,'FontSize',fntsz)

subplot(2,2,4)
plot(tspan,delta_xsim(:,4)*r2d,'k','LineWidth',1)
hold on
plot(tspan,delta_xhat1(:,4)*r2d,'--r','LineWidth',1);
plot(tspan,delta_xhat2(:,4)*r2d,'--b','LineWidth',1);
plot(tspan,delta_xhat3(:,4)*r2d,'--m','LineWidth',1);
% plot(tspan,delta_xhat4(:,4)*r2d,'--c','LineWidth',1);
hold off
title('Pitch angle perturbation against time')
xlabel('Time/s')             
ylabel('Degrees')
% fit = goodnessOfFit(delta_xhat(:,4)*r2d,delta_xsim(:,4)*r2d,cost_func);
% legend('Simulated Data',strcat('Predicted Model: ',num2str(round(fit*100,2)),'%'))
grid on
set(gca,'FontSize',fntsz)

