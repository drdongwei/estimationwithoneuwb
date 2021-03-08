%% load data
clc;clear
data = load('data/2.mat');
lopt = 8;  % set moving horizon
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
[b1,a1] = butter(2,0.04,'low');  % butterworth filter, cutoff frequency: 0.04*25 = 1Hz
[b2,a2] = butter(4,0.2,'low'); % butterworth filter, cutoff frequency: 0.2*25 = 0.5Hz
[bi,ai] = butter(3,0.1,'low'); % butterworth filter, cutoff frequency: 0.1*25 = 0.25Hz

    % filter ranging measurements
    y = filtfilt(b1,a1,uwb(1:lopt+2));
    y1 = filtfilt(b1,a1,uwb1(1:lopt+2));
    y2 = filtfilt(b1,a1,uwb2(1:lopt+2));

    % radical velocity
    vy =  [0,0];
    vy1 = [0,0];
    vy2 = [0,0];
    for i = 2:lopt+2
    	vy(i) = abs(y(i)-y(i-1))/(dt);
        vy1(i) = abs(y1(i)-y1(i-1))/(dt); 
        vy2(i) = abs(y2(i)-y2(i-1))/(dt);
    end
     % filter radical velocity
    vy = filtfilt(b1,a1,vy);
    vy1 = filtfilt(b1,a1,vy1);
    vy2 = filtfilt(b1,a1,vy2);

%p = parpool('local',6);
%------------- preprocessing-----------------%
%% initialize
delta = 10;
xt = gtd(:, 1:lopt+delta); % record estimation by MHE, initial
xt_imu = gtd(:, 1:lopt+delta); %?
xi = xt(:, delta+2); % \hat{x}_2-, use visual estimation as initial, and then use the MHE estimate
opt_range = 1300;
X = xi;
for i = lopt+delta+1:opt_range  % i: current time-step
    
    % calculate current ranging measurements & radical velocity, i-th
    yt = filtfilt(b1,a1,uwb(1:i));
    y(i) = yt(end);
    yt = filtfilt(b1,a1,uwb1(1:i));
    y1(i) = yt(end);
    yt = filtfilt(b1,a1,uwb2(1:i));
    y2(i) = yt(end);
    
    vy(i) = (y(i)-y(i-1))/dt;
    vy1(i) = (y1(i)-y1(i-1))/dt;
    vy2(i) = (y2(i)-y2(i-1))/dt;
    
    vy = filtfilt(b1,a1,vy);
    vy1 = filtfilt(b1,a1,vy1);
    vy2 = filtfilt(b1,a1,vy2);
    
    xi = xt(:,i-lopt+1); % estimated initial value
    disp(['Step ' , num2str(i)])

    % used for MHE, 15 points
    MHE_imu = imu(:,i-lopt+1:i);
    MHE_uwb = y(i-lopt+1:i);
    MHE_uwb1 = y1(i-lopt+1:i);
    MHE_uwb2 = y2(i-lopt+1:i);
    MHE_v = vy(i-lopt+1:i);
    MHE_v1 = vy1(i-lopt+1:i);
    MHE_v2 = vy2(i-lopt+1:i);
    
    x0 = prediction(xi, MHE_imu, dt); % use prediction as initial guess
%     x0 = [xt(i-lopt+1,i) ];
%     x_t = prediction(xt_imu(:,i-lopt), MHE_imu, dt); %??
%     xt_imu(:,i) = x_t(:,end); %？
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
    X = fmincon(@(x)objmhemulti (x, xi, MHE_imu, MHE_uwb, MHE_uwb1, MHE_uwb2, MHE_v, MHE_v1, MHE_v2),x0,[],[],[],[],[],[],[],options);
    xt(:,i) = X(:,end);
    
    % post processing
    xt(1,:) = filtfilt(b2,a2,xt(1,:));
    xt(2,:) = filtfilt(b2,a2,xt(2,:));
    xt(3,:) = filtfilt(b2,a2,xt(3,:));

    xt(4,:) = filtfilt(b2,a2,xt(4,:));
    xt(5,:) = filtfilt(b2,a2,xt(5,:));
    xt(6,:) = filtfilt(b2,a2,xt(6,:));

end

return
