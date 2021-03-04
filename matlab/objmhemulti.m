function f = objmhemulti(x, x0, u, y, y1, y2, v, v1, v2)
obj = 0;
a = 100;
b = 1.2;
c = 10;
d = 3;
% c = 10;
% d = 10;
e = 50;
f = 50;

% pa = [-0.1,-0.1, -0.1];
pa = [-0,-0, -0];
dt = 0.04;

% A = [1, 0, 0, dt, 0, 0;
%      0, 1, 0, 0, dt, 0;
%      0, 0, 1, 0, 0, dt,;
%      0, 0, 0, 1, 0, 0;
%      0, 0, 0, 0, 1, 0;
%      0, 0, 0, 0, 0, 1;];
 
 A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt;
     0, 0, 0, 1+pa(1)*dt, 0, 0;
     0, 0, 0, 0, 1+pa(2)*dt, 0;
     0, 0, 0, 0, 0, 1+pa(3)*dt;];
 
%   B = [0, 0, 0;
%        0, 0, 0;
%        0, 0, 0;
%       dt, 0,  0;
%       0,  dt, 0;
%       0,  0, dt;];
  
  
  B = [0.5*dt^2, 0, 0;
     0, 0.5*dt^2, 0;
     0, 0, 0.5*dt^2;
     dt, 0,  0;
     0,  dt, 0;
     0,  0, dt;];

xt(:,1) = x0;
t_vp = sqrt(xt(4:6,1)'*xt(4:6,1));
for i = 1:length(u)
    xt(:, i+1) = A  *  x(:, i) + B  *  u(:, i);    
    t_obj = (xt(:, i) - x(:, i))' * (xt(:, i) - x(:, i));
    obj = b * t_obj;
    lx = sqrt(x(1:3,i)' * x(1:3,i));
    d_dis(i) = (lx - y(i))^2;
    t_obj =  c * (lx - y(i))^2;
    obj = obj + t_obj;
%     x1 = x;
%     x1(1,i) = x1(1,i) + 0.15;
%     x1(2,i) = x1(2,i) + 1.8;
%     x1(3,i) = x1(3,i) + 1.95;
% 
%     lx = sqrt(x1(1:3,i)' * x1(1:3,i));
%     d_dis1(i) = (lx - y1(i))^2;
%     t_obj = c * (d_dis1(i));
%     obj = obj + t_obj;
%     x1 = x;
%     x1(1,i) = x1(1,i) - 0.55;
%     x1(2,i) = x1(2,i) - 2.25;
%     x1(3,i) = x1(3,i) + 1.5;
% 
%     lx = sqrt(x1(1:3,i)' * x1(1:3,i));
%     d_dis2(i) = (lx - y2(i))^2;
%     t_obj = c * (d_dis2(i));
%     obj = obj + t_obj;
%     vt = max([v(i),v1(i),v2(i)]);
%    vt = max([v(i),v2(i)]);
%     vt = v(i);
%    v_dis(i) = (sqrt(x(4:6,i)'*x(4:6,i))-vt)^2;
%     t_vs = sqrt(x(4:6,i)'*x(4:6,i));
%     v_dis(i) = ((vt-t_vs)^2)*exp((vt+0.1)/(t_vs+0.1));
%     t_vp = t_vs;
%     t_obj = d*v_dis(i);
%     obj = obj + t_obj;
end

%  d_q = max(d_dis);
%  v_q = max(v_dis);
%  obj = obj + e * d_q + f * v_q;

obj = obj + a* (x(:,1)-x0(:,1))'*(x(:,1)-x0(:,1));

f = obj;





