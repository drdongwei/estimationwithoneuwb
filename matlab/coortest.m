clear all
close all
data = load('2.mat');
%tf 0: 4.75, 0, 0
%tf 1: 4.43, 0.59, 0
%tf 2: 4.43, 0.59, 0
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
pa1 = [0 0 0];
pa2 = [0 0 0];
pa3 = [0 0 0];
r1 = 10000;
r2 = 10000;
r3 = 10000;

uwb = data.uwb;
uwb1 = data.uwb1;
uwb2 = data.uwb2;
imu = data.imu';
time = data.time;
y = filtfilt(b1,a1,uwb);
y1 = filtfilt(b1,a1,uwb1);
y2 = filtfilt(b1,a1,uwb2);
t = time(1:800);

gtd1 = gtd;
gtd2 = gtd;

eth = 0.4;

for i = 1:30
    for j= 1:100
        for k = 1:40
            dx = -i/20.0;
            dy = (j/20.0-2.5);
            dz = k/20.0-1;
            t_gtd(1,:) = gtd(1,1:800) + dx;
            t_gtd(2,:) = gtd(2,1:800) + dy;
            t_gtd(3,:) = gtd(3,1:800) + dz;
            t_gtd1(1,:) = gtd1(1,1:800) + dx;
            t_gtd1(2,:) = gtd1(2,1:800) + dy;
            t_gtd1(3,:) = gtd1(3,1:800) + dz;
            t_gtd2(1,:) = gtd2(1,1:800) + dx;
            t_gtd2(2,:) = gtd2(2,1:800) + dy;
            t_gtd2(3,:) = gtd2(3,1:800) + dz;
            lgd = sqrt(sum(t_gtd.^2, 1));
            lgd1 = sqrt(sum(t_gtd1.^2, 1));
            lgd2 = sqrt(sum(t_gtd2.^2, 1));
            
            er = abs(y(1:800)-lgd);
            er = er(er<eth);
            td = sum(er)*800/length(er);
            
            er = abs(y1(1:800)-lgd1);
            er = er(er<eth);
            td1 = sum(er)*800/length(er);
            
            er = abs(y2(1:800)-lgd2);
            er = er(er<eth);
            td2 = sum(er)*800/length(er);
            
            if td < r1
                r1 = td;
                pa1 = [dx,dy,dz];
            end
            if td1 < r2
                r2 = td1;
                pa2 = [dx,dy,dz];
            end
            if td2 < r3
                r3 = td2;
                pa3 = [dx,dy,dz];
            end
        end
    end
end

pa1
pa2
pa3
dp2 = pa2 - pa1
dp3 = pa3 - pa1



