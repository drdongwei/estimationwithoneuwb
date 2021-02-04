data = load('1.mat');
lopt = 25;
if(lopt > length(data.uwb))
    lopt = length(data.uwb);
end

gtd = data.gtd(1:lopt,:)';
uwb = data.uwb(1:lopt)';
imu = data.imu(1:lopt,:)';
att = data.att(1:lopt,:);
time = data.time(1:lopt);
dt = 0.04;
xi = [gtd(1,1), gtd(2,1), gtd(3,1), (gtd(1,2)-gtd(1,1))/dt,(gtd(2,2)-gtd(2,1))/dt,(gtd(3,2)-gtd(3,1))/dt]'
x0 = progagation(xi, imu, 0.04);
%r=objmhe (x0, xi, imu, uwb)
%option =  optimset('Algorithm','active-set'); 
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
X = fmincon(@(x)objmhe (x, xi, imu, uwb),x0,[],[],[],[],[],[],[],options);
figure(1)
plot(time,X(1,:),'r')
hold on
plot(time,X(2,:),'g')
plot(time,X(3,:),'b')
grid on

figure(2)
plot(time,gtd(1,:),'r')
hold on
plot(time,gtd(2,:),'g')
plot(time,gtd(3,:),'b')
grid on