clc;clear
data = load('../data/2.mat');
lopt = 15;  % set moving horizon
time = data.time;
dt = 0.04;  % sampling frequency: 25Hz

% ground truth of \boldsymbol{x}
gtd = data.gtd'; 
gtd(4:6,:) = data.gtv'; 
gtd(1,:) = gtd(1,:)-0.6; % remove the bias of VICON
gtd(2,:) = gtd(2,:) + 0.3;
gtd(3,:) = gtd(3,:)-1.65;

% attitude of the quadrotor
att = data.att';  

% ranging measurements of ground robots
uwb = data.uwb'; 
uwb1 = data.uwb1';
uwb2 = data.uwb2';

% acceleration
imu = data.imu'; 
imu(3,:) = imu(3,:)- mean(imu(3,:))+0.02;
imu(2,:) = imu(2,:) - 0.07;

%------------- preprocessing-----------------%
[b1,a1] = butter(3,0.04,'low');  % butterworth filter, cutoff frequency: 0.04*25 = 1Hz
[b2,a2] = butter(4,0.2,'low'); % butterworth filter, cutoff frequency: 0.2*25 = 0.5Hz
[bi,ai] = butter(3,0.1,'low'); % butterworth filter, cutoff frequency: 0.1*25 = 0.25Hz


order = 4;
framelen = 31;
t_uwb = uwb(1:300);
sgf = sgolayfilt(t_uwb,order,framelen);
ybt = filtfilt(b1,a1,t_uwb);

figure
plot(t_uwb);
grid on 
hold on
plot(sgf, 'r--');
plot(ybt, 'g--');


