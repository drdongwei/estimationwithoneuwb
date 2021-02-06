close all;
figure(1)
plot(time(1:length(xt(1,:))),xt(1,:),'r', 'linewidth', 1)
hold on
plot(time(1:length(xt(2,:))),xt(2,:),'g', 'linewidth', 1)
plot(time(1:length(xt(3,:))),xt(3,:),'b', 'linewidth', 1)

plot(time(1:length(xt(1,:))),gtd(1,1:length(xt(1,:))),'r--', 'linewidth', 1)
plot(time(1:length(xt(2,:))),gtd(2,1:length(xt(2,:))),'g--', 'linewidth', 1)
plot(time(1:length(xt(3,:))),gtd(3,1:length(xt(3,:))),'b--', 'linewidth', 1)

plot(time(1:length(xt(1,:))),xt_imu(1,1:length(xt(1,:))),'r-.')
plot(time(1:length(xt(2,:))),xt_imu(2,1:length(xt(2,:))),'g-.')
plot(time(1:length(xt(3,:))),xt_imu(3,1:length(xt(3,:))),'b-.')

plot(time(1:length(xt(1,:))),uwb(1:length(xt(1,:))),'k--', 'linewidth', 2)
grid on
xlim([0,30])
ylim([-1, 10])
legend('$\hat{x}$','$\hat{y}$','$\hat{z}$','$\hat{x}_g$','$\hat{y}_g$','$\hat{z}_g$','$\hat{x}_a$','$\hat{y}_a$','$\hat{z}_a$','${d}_{uwb}$','Interpreter','LaTex','FontSize',18,'NumColumns',2) 
xlabel('Time (s)')
ylabel('Position (m)')

% return;

figure(2)
plot(time(1:length(xt(1,:))),xt(4,:),'r', 'linewidth', 2)
hold on
plot(time(1:length(xt(2,:))),xt(5,:),'g', 'linewidth', 2)
plot(time(1:length(xt(3,:))),xt(6,:),'b', 'linewidth', 2)

plot(time(1:length(xt(1,:))),gtd(4,1:length(xt(1,:))),'r--', 'linewidth', 2)
plot(time(1:length(xt(2,:))),gtd(5,1:length(xt(2,:))),'g--', 'linewidth', 2)
plot(time(1:length(xt(3,:))),gtd(6,1:length(xt(3,:))),'b--', 'linewidth', 2)

plot(time(1:length(xt(1,:))),xt_imu(4,1:length(xt(1,:))),'r--')
plot(time(1:length(xt(2,:))),xt_imu(5,1:length(xt(2,:))),'g--')
plot(time(1:length(xt(3,:))),xt_imu(6,1:length(xt(3,:))),'b--')

grid on