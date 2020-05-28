function CLCD(delta_xsim,delta_xhat,delta_u,u,method)

c = 1.74;
r2d = 180/pi;

CL = 6.44*delta_xsim(:,1) + 3.8*c/(2*delta_xsim(:,2)*u)*delta_xsim(:,3) + 0.355*delta_u;
CL_hat = 6.44*delta_xhat(:,1) + 3.8*c/(2*delta_xhat(:,2)*u)*delta_xhat(:,3) + 0.355*delta_u;

CD = 0.03 + 0.05 * CL.^2;
CD_hat = 0.03 + 0.05 * CL_hat.^2;

x = linspace(min(delta_xhat(:,1))*r2d,max(delta_xhat(:,1))*r2d);
p = polyfit(delta_xhat(:,1)*r2d,CL_hat,1);
y1 = polyval(p,x);
p2 = polyfit(delta_xhat(:,1)*r2d,CD_hat,2);
y2 = polyval(p2,x);
y = 11.5 * [0:0.01:0.05];

if method == 1
    heading = 'Ordinary Least Squares';
elseif method == 2
    heading = 'Total Least Sqaures';
elseif method == 3
    heading = 'Gauss-Newton';
else 
    heading = 'Levenburg-Marquardt';
end

subplot(3,1,1)
plot(CD,CL,'.r')
hold on
plot(y2,y1,'b','linewidth',1.5)
%plot([0:0.01:0.05],y,'--k') % for glide ratio
title({heading,'Drag Polar'})
xlabel('CD') 
ylabel('CL')
%axis([0 0.05 -0.5 0.5]) % For glide ratio
grid on
set(gca,'FontSize',14)
legend('Simulated','Predicted','Location','southeast')
hold off

subplot(3,1,2)
plot(delta_xsim(:,1)*r2d,CL,'r.');
hold on
plot(x,y1,'b','linewidth',1.5)
title('Lift Coefficient against AOA')
xlabel('Angle of Attack/Degrees') 
ylabel('CL')
grid on
set(gca,'FontSize',14)

subplot(3,1,3)
plot(delta_xsim(:,1)*r2d,CD,'r.');
hold on
plot(x,y2,'b','linewidth',1.5)
title('Drag Coefficient against AOA')
xlabel('Angle of Attack/Degrees') 
ylabel('CD')
grid on
set(gca,'FontSize',14)




