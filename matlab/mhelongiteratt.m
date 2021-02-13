%% load data
data = load('2.mat');
lopt = 15;        %set moving horizong
gtd = data.gtd';
gtd(4:6,:) = data.gtv';

gtd(1,:) = gtd(1,:)-0.4;
gtd(3,:) = gtd(3,:)-0.3;
att = data.att';
uwb = data.uwb';
uwb1 = data.uwb1';
uwb2 = data.uwb2';
imu = data.imu';
imu(3,:) = imu(3,:)- mean(imu(3,:))+0.02;
imu(2,:) = imu(2,:) - 0.07;

[b1,a1] = butter(3,0.04,'low');
[b2,a2] = butter(4,0.2,'low');
[bi,ai] = butter(3,0.1,'low');




% lav=200;
% 
% mu(1:lav)=imu(2,1:lav);
% for i = lav+1:length(imu(2,:))
%     mu(i)= imu(2,i)-mean(imu(2,i-lav:i));
% end
% 
% imu(2,:) = mu;

imu(1,:) = filtfilt(bi,ai,imu(1,:));
imu(2,:) = filtfilt(bi,ai,imu(2,:));
imu(3,:) = filtfilt(bi,ai,imu(3,:));
% 
% imu(2,:) = imu(2,:) *0.5;
% imu(3,:) = imu(3,:) *0.5;

time = data.time;
dt = 0.04;  

vy = [0,0];
y = filtfilt(b1,a1,uwb);
uwb = y;
for i = 2:length(uwb)
    vy(i) = abs(y(i)-y(i-1))/(dt);
end
vy = filtfilt(b1,a1,vy);
% plot(vy)

%% initialize
xt = gtd(:, 1:lopt);
xt_imu = gtd(:, 1:lopt);
xi = xt(:, 1);
opt_range = 1300;
li = 0;
du = [0,0,0]';
X = xi;
for i = lopt+3:opt_range %length(uwb)-lopt
    disp(['Step ' , num2str(i)])
    xi = xt(:,i-lopt);

    t_imu = imu(:,i-lopt+1:i);
    t_uwb = uwb(i-lopt+1:i);
    x0 = progagation(xi, t_imu, dt);
    x_t = progagation(xt_imu(:,i-lopt), t_imu, dt);
    xt_imu(:,i) = x_t(:,end);
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
    X = fmincon(@(x)objmhe (x, xi, t_imu, t_uwb, vy(i-lopt+1:i)),x0,[],[],[],[],[],[],[],options);
    xt(:,i) = X(:,end);
    xt(1,:) = filtfilt(b2,a2,xt(1,:));
    xt(2,:) = filtfilt(b2,a2,xt(2,:));
    xt(3,:) = filtfilt(b2,a2,xt(3,:));

    xt(4,:) = filtfilt(b2,a2,xt(4,:));
    xt(5,:) = filtfilt(b2,a2,xt(5,:));
    xt(6,:) = filtfilt(b2,a2,xt(6,:));

end

return
