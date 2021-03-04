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

imu(1,:) = filtfilt(b2,a2,imu(1,:));
imu(2,:) = filtfilt(b2,a2,imu(2,:));
imu(3,:) = filtfilt(b2,a2,imu(3,:));

% imu(3,:) = imu(3,:) *0.1;
% imu(2,:) = imu(2,:) *0.1;



for i=1:length(att(2,:))
ae(1,i)= (cos(att(1,i))*sin(att(2,i))*cos(att(3,i))+sin(att(1,i))*sin(att(3,i))+0.028)*2;
ae(2,i)= (cos(att(1,i))*sin(att(2,i))*sin(att(3,i))-sin(att(1,i))*cos(att(3,i))+0.005)*2;
end
ae(3,:)= imu(3,:);

x0 = gtd(:,1);

xt = progagation(x0, imu, dt);
xt_t = progagation(x0, ae, dt);


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
