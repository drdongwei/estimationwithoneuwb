%% load data
data = load('1.mat');
lopt = 15;        %set moving horizong
gtd = data.gtd';
gtd(4:6,:) = data.gtv';
uwb = data.uwb';
imu = data.imu';
att = data.att;
time = data.time;
dt = 0.04;        %servo time horizon
%imu(2,:)=-imu(2,:);

%% initialize
xt = gtd(:, 1:lopt);
xt_imu = gtd(:, 1:lopt);
xi = xt(:, 1);
opt_range = 100;
li = 0;
du = [0,0,0]';
X = xi;
for i = lopt+1:opt_range %length(uwb)-lopt
    disp(['Step ' , num2str(i)])
    if(mod(floor(i/lopt),2)==1)
        gt = gtd(:,i-lopt+1:(floor(i/lopt))*lopt);
    else
        gt = gtd(:,i-lopt+1:i);
    end
    xi = gt(:,1);
    t_imu = imu(:,i-lopt+1:i);
    t_uwb = uwb(i-lopt+1:i);
    lgt = length(gt(1,:));
    x0 = progagation(xi, t_imu, dt);
    x0_t = gt;
    if(lgt < lopt)
        t_a = t_imu(:,lgt+1:end);
        x0_t = progagation(gt(:,end), t_a, dt);
    end
    xt_imu(:,i) = x0_t(:,end);
    if(lgt>5)
        du = ((gt(4:6,end) - gt(4:6,1)) - (x0(4:6,lgt) - x0(4:6, 1)))/(lgt*dt);
    end
    t_imu = t_imu + 0.8*repmat(du,1,lopt);
    x0 = gt;
    if(lgt < lopt)
        x0(:, lgt+1:lopt) = progagation(gt(:,end), t_imu(:,lgt+1:end), 0.04);
    end
    options = optimoptions('fmincon','Algorithm','sqp');
    X = fmincon(@(x)objmhe (x, xi, t_imu, t_uwb, gt),x0,[],[],[],[],[],[],[],options);
    xt(:,i) = X(:,end);
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