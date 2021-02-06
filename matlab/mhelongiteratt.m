%% load data
data = load('2.mat');
lopt = 15;        %set moving horizong
gtd = data.gtd';
gtd(4:6,:) = data.gtv';
uwb = data.uwb';
imu = data.imu';
imu(3,:) = imu(3,:)- mean(imu(3,:));
imu(2,:) = imu(2,:) - 0.099;
%imu(2,:) = imu(2,:);
% 
[b1,a1] = butter(3,0.2,'low');
[b2,a2] = butter(4,0.1,'low');
% 
% imu(1,:) = filtfilt(b,a,imu(1,:));
% imu(2,:) = filtfilt(b,a,imu(2,:));
% imu(3,:) = filtfilt(b,a,imu(3,:));
% imu(1,:) = imu(1,:)- mean(imu(1,:));
% imu(2,:) = imu(2,:)- mean(imu(2,:));
% imu(3,:) = imu(3,:)- mean(imu(3,:));
% imu =imu*0.9;

time = data.time;
dt = 0.04;        

%% initialize
xt = gtd(:, 1:lopt);
xt_imu = gtd(:, 1:lopt);
xi = xt(:, 1);
opt_range = 1800;
li = 0;
du = [0,0,0]';
X = xi;
for i = lopt+1:opt_range %length(uwb)-lopt
    disp(['Step ' , num2str(i)])
    xi = xt(:,i-lopt);
    t_imu = imu(:,i-lopt+1:i);
    t_uwb = uwb(i-lopt+1:i);
    x0 = progagation(xi, t_imu, dt);
    x_t = progagation(xt_imu(:,i-lopt), t_imu, dt);
    xt_imu(:,i) = x_t(:,end);
    options = optimoptions('fmincon','Algorithm','sqp');
    X = fmincon(@(x)objmhe (x, xi, t_imu, t_uwb),x0,[],[],[],[],[],[],[],options);
    xt(:,i) = X(:,end);
%     xt(1,:) = filtfilt(b1,a1,xt(1,:));
%     xt(2,:) = filtfilt(b1,a1,xt(2,:));
%     xt(3,:) = filtfilt(b1,a1,xt(3,:));
%     xt(4,end) = (xt(1,end)-xt(1,end-1))/dt;
%     xt(5,end) = (xt(2,end)-xt(2,end-1))/dt;
%     xt(6,end) = (xt(3,end)-xt(3,end-1))/dt;
%     xt(4,:) = filtfilt(b2,a2,xt(4,:));
%     xt(5,:) = filtfilt(b2,a2,xt(5,:));
%     xt(6,:) = filtfilt(b2,a2,xt(6,:));
%     xt_1(1,:) = filtfilt(b1,a1,xt(1,:));
%     xt_1(2,:) = filtfilt(b1,a1,xt(2,:));
%     xt_1(3,:) = filtfilt(b1,a1,xt(3,:));
%     xt_1(4,:) = filtfilt(b2,a2,xt(4,:));
%     xt_1(5,:) = filtfilt(b2,a2,xt(5,:));
%     xt_1(6,:) = filtfilt(b2,a2,xt(6,:));
end

return

figure(1)
subplot(311)
plot(time(1:length(xt(1,:))),xt(1,:),'r', 'linewidth', 2)
hold on
plot(time(1:length(xt(2,:))),xt(2,:),'g', 'linewidth', 2)
plot(time(1:length(xt(3,:))),xt(3,:),'b', 'linewidth', 2)
grid on

subplot(312)
plot(time(1:length(xt(1,:))),gtd(1,1:length(xt(1,:))),'r', 'linewidth', 2)
hold on
plot(time(1:length(xt(2,:))),gtd(2,1:length(xt(2,:))),'g', 'linewidth', 2)
plot(time(1:length(xt(3,:))),gtd(3,1:length(xt(3,:))),'b', 'linewidth', 2)
plot(time(1:length(xt(1,:))),uwb(1:length(xt(1,:))),'k--', 'linewidth', 2)
subplot(313)
plot(time(1:length(xt(1,:))),xt_imu(1,1:length(xt(1,:))),'r', 'linewidth', 2)
hold on
plot(time(1:length(xt(2,:))),xt_imu(2,1:length(xt(2,:))),'g', 'linewidth', 2)
plot(time(1:length(xt(3,:))),xt_imu(3,1:length(xt(3,:))),'b', 'linewidth', 2)
grid on