clear all
data = load('2.mat');

[b1,a1] = butter(3,0.04,'low');
[b2,a2] = butter(3,0.1,'low');
[b3,a3] = butter(3,[0.02,0.1]);
dt = 0.04;
vel =[0, 0];
vel2 =[0, 0];
vg =[0,0];
dg =[0,0];
gtd = data.gtd';
gtd(4:6,:) = data.gtv';
gtd(1,:) = gtd(1,:)-0.4;
% gtd(1,:) = gtd(1,:)+0.4;
gtd(3,:) = gtd(3,:)-0.3;
uwb = data.uwb;
imu = data.imu';
time = data.time;
y = filtfilt(b1,a1,uwb);
t = time(1:800);
for i = 1:800
    vel(i) = abs(y(i+1)-y(i))/(dt);
    vel2(i) = sqrt(max(y(i)^2+y(i+2)^2-2*y(i+1)^2,0)/2)/dt;
    [azo,elo,rhoo] = cart2sph(gtd(4, i), gtd(5, i), gtd(6, i));
    vel3(i) =  rhoo;
    vg(i) = sqrt(gtd(4:6,i)'*gtd(4:6,i));
    dg(i) = sqrt(gtd(1:3,i)'*gtd(1:3,i));
    [az0,el0,rho0] = cart2sph(gtd(1, i), gtd(2, i), gtd(3, i));
    [az,el,rho] = cart2sph(gtd(1, i+1), gtd(2, i+1), gtd(3, i+1));
    vgr(i) = abs(rho - rho0)/dt;
end
% vel = filtfilt(b1,a1,vel);

mean(imu(3,:))
imu(3,:) = (imu(3,:)-9.77);
imu(2,:) = (imu(2,:)-0.059);
% mean(imu(3,:))

lav=200;

mu(1:lav)=imu(2,1:lav);
for i = lav+1:length(imu(2,:))
    mu(i)= imu(2,i)-0.9*mean(imu(2,i-lav:i-1));
%     mu(i)= imu(2,i)-0.9*mean(mu(i-lav:i-1));
end

imu(2,:) = mu;


imu(1,:) = filtfilt(b2,a2,imu(1,:));
imu(2,:) = filtfilt(b2,a2,imu(2,:));
imu(3,:) = filtfilt(b2,a2,imu(3,:));



% 
% lav=200;
% 
% mu(1:lav)=imu(3,1:lav);
% for i = lav+1:length(imu(3,:))
%     mu(i)= imu(3,i)-0.9*mean(imu(3,i-lav:i-1));
% %     mu(i)= imu(2,i)-0.9*mean(mu(i-lav:i-1));
% end
% 
% imu(3,:) = mu;




imu(3,:) = imu(3,:) *0.5;
imu(2,:) = imu(2,:) *0.5;

% imu(2,:) = imu(2,:)-0.059;
x0 = gtd(:,1);

xt = progagation(x0, imu, dt);


figure
plot(t,vel,'r');
hold on
plot(t,vel2,'g');
plot(t,vel3+0.1,'b');
plot(t,vg,'m');
grid on

return
figure
subplot(211)
plot(time, xt(1,:),'r');
hold on
grid on
plot(time, xt(2,:),'g');
plot(time, xt(3,:),'b');
subplot(212)
plot(time, xt(4,:),'r');
hold on
grid on
plot(time, xt(5,:),'g');
plot(time, xt(6,:),'b');
return
figure
plot(time, imu(1,:),'r');
hold on
grid on
plot(time, imu(2,:),'g');
plot(time, imu(3,:),'b');

figure
subplot(211)
plot(t,data.uwb(1:800),'b:')
hold on 
plot(t,y(1:800),'m')
plot(t,dg,'k--','linewidth',2)
grid on
ylabel('$d$ (m)','Interpreter','LaTex')
legend('$d_r$','$d_f$','$d_g$','Interpreter','LaTex','FontSize',18)
xlim([0,30])
subplot(212)
plot(t,vel,'m')
hold on
plot(t,vel2,'r')
hold on
plot(t,vg,'k--','linewidth',2)
plot(t,vgr,'b--','linewidth',2)
legend('$v_d$','$v_{d2}$','$v_g$','$v_{gr}$','Interpreter','LaTex','FontSize',18)
grid on
xlim([0,30])
ylim([0,2])
ylabel('$v_d$ (m/s)','Interpreter','LaTex')
xlabel('Time (s)','Interpreter','LaTex')