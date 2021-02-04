
data = load('1.mat');
gtd = data.gtd;
uwb = data.uwb;
imu = data.imu;
att = data.att;
time = data.time;

figure()
plot(time, gtd(:, 1), 'r')
hold on
plot(time, gtd(:, 2), 'g')
plot(time, gtd(:, 3), 'b')
grid on