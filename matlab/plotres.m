close all
[bi,ai] = butter(3,0.1,'low');
figure(1)
plot(time(1:length(xt(1,:))),xt(1,:),'r', 'linewidth', 2)
hold on
plot(time(1:length(xt(2,:))),xt(2,:),'m', 'linewidth', 2)
plot(time(1:length(xt(3,:))),xt(3,:),'b', 'linewidth', 2)

plot(time(1:length(xt(1,:))),gtd(1,1:length(xt(1,:))),'r--', 'linewidth', 2)
plot(time(1:length(xt(2,:))),gtd(2,1:length(xt(2,:))),'m--', 'linewidth', 2)
plot(time(1:length(xt(3,:))),gtd(3,1:length(xt(3,:))),'b--', 'linewidth', 2)

% plot(time(1:length(xt(1,:))),xt_imu(1,1:length(xt(1,:))),'r-.')
% plot(time(1:length(xt(2,:))),xt_imu(2,1:length(xt(2,:))),'m-.')
% plot(time(1:length(xt(3,:))),xt_imu(3,1:length(xt(3,:))),'b-.')

% plot(time(1:length(xt(1,:))),uwb(1:length(xt(1,:))),'k--', 'linewidth', 2)
grid on
xlim([0,45])
ylim([-5, 8])
legend('\alpha','Interpreter','latex')
% legend('\hat{x}','\hat{y}','\hat{z}','$\hat{x}_g$','\hat{y}_g','\hat{z}_g','FontSize',18,'NumColumns',3)
% legend('$\hat{x}$','$\hat{y}$','$\hat{z}$','$\hat{x}_g$','$\hat{y}_g$','$\hat{z}_g$','$\hat{x}_a$','$\hat{y}_a$','$\hat{z}_a$','Interpreter','latex','FontSize',18,'NumColumns',3) 
xlabel('Time (s)')
ylabel('Position (m)')

% return;
%% figure 2
% figure(2)
% plot(time(1:length(xt(1,:))),xt(4,:),'r', 'linewidth', 2)
% hold on
% plot(time(1:length(xt(2,:))),xt(5,:),'m', 'linewidth', 2)
% plot(time(1:length(xt(3,:))),xt(6,:),'b', 'linewidth', 2)
% 
% vx =filtfilt(bi,ai,gtd(4,1:length(xt(1,:))));
% vy =filtfilt(bi,ai,gtd(5,1:length(xt(1,:))));
% vz =filtfilt(bi,ai,gtd(6,1:length(xt(1,:))));
% plot(time(1:length(xt(1,:))),vx,'r--', 'linewidth', 2)
% plot(time(1:length(xt(2,:))),vy,'m--', 'linewidth', 2)
% plot(time(1:length(xt(3,:))),vz,'b--', 'linewidth', 2)
% 
% xlim([0,45])
% ylim([-1.5, 1.5])

% % plot(time(1:length(xt(1,:))),xt_imu(4,1:length(xt(1,:))),'r--')
% % plot(time(1:length(xt(2,:))),xt_imu(5,1:length(xt(2,:))),'c--')
% % plot(time(1:length(xt(3,:))),xt_imu(6,1:length(xt(3,:))),'b--')

% grid on
% 
% figure(3)
% 
% for i = 1:length(vs)
%     vg(i) = sqrt(gtd(4:6,i)'*gtd(4:6,i));
% end
% 
% plot(time(1:length(vg)),vg,'k--', 'linewidth', 1)
% hold on
% plot(time(1:length(vs)),vs,'m', 'linewidth', 1)
% ylim([0,1.5])
% xlim([0,30])
