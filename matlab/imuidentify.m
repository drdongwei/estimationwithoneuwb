
clear A
starti = 3;
endi = 100+ starti;
j = 1;
v = gtd(j+1,starti:endi)-gtd(j+1,starti);
va = xt_imu(j+1,starti:endi)-xt_imu(j+1,starti);
t = 0.04 * [starti:endi];
A(1, :) = va;
A(2, :) = t;
A = A';
param = inv(A'*A)* A' * v'
