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
att = data.att';
time = data.time;
y = filtfilt(b1,a1,uwb);
t = time(1:800);

imu(3,:) = (imu(3,:)-9.77);
imu(2,:) = (imu(2,:)-0.059);

% imu(1,:) = filtfilt(b2,a2,imu(1,:));
% imu(2,:) = filtfilt(b2,a2,imu(2,:));
% imu(3,:) = filtfilt(b2,a2,imu(3,:));
x0 = gtd(:,1);
xt = progagation(x0, imu, dt);

z3 = iddata(gtd(4,:)',imu(1,:)',0.04);
model = arx(z3,[2,2,1])

arv(1:2) = gtd(4,1:2);
for i = 3: length(imu(1,:)) 
    arv(i) = 0.817 * arv(i-1) + 0.1739 * arv(i-2) + 0.001598 * imu(1,i-1) - 0.002422 * imu(1,i-2);
end

figure
plot(time, gtd(4,:),'r');
grid on
hold on
plot(time, xt(4,:),'r:','linewidth',2);
plot(time, arv,'r--','linewidth',2);

return 
figure

plot(time, xt(1,:),'r');
hold on
grid on
plot(time, xt(2,:),'g');
plot(time, xt(3,:),'b');

plot(time, xt_t(1,:),'r--');
plot(time, xt_t(2,:),'g--');
plot(time, xt_t(3,:),'b--');

plot(time, gtd(1,:),'r:','linewidth',2);
plot(time, gtd(2,:),'g:','linewidth',2);
plot(time, gtd(3,:),'b:','linewidth',2);

% figure 
% plot(time, att(3,:))

return
