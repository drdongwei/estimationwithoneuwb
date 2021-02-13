function f = objmhemulti(x, x0, u, y, y1, y2, v, v1, v2)
obj = 0;
a = 100;
b = 1.2;
c = 5;
d = 20;
% c = 10;
% d = 10;
e = 50;
f = 50;

dt = 0.04;

A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt,;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];
 
  B = [0, 0, 0;
       0, 0, 0;
       0, 0, 0;
      dt, 0,  0;
      0,  dt, 0;
      0,  0, dt;];
  
  
  B = [0.5*dt^2, 0, 0;
     0, 0.5*dt^2, 0;
     0, 0, 0.5*dt^2;
     dt, 0,  0;
     0,  dt, 0;
     0,  0, dt;];

xt(:,1) = x0;

for i = 1:length(u)
    xt(:, i+1) = A  *  xt(:, i) + B  *  u(:, i);
    t_obj = (xt(:, i+1) - x(:, i))' * (xt(:, i+1) - x(:, i));
    lx = sqrt(x(1:3,i)' * x(1:3,i));
    d_dis(i) = (lx - y(i))^2;
    t_obj = b * t_obj + c * (lx - y(i))^2;
    obj = obj + t_obj;
    x1 = x;
    x1(1,i) = x1(1,i) + 0.15;
    x1(2,i) = x1(2,i) + 1.8;
    x1(3,i) = x1(3,i) + 1.95;
    lx = sqrt(x1(1:3,i)' * x1(1:3,i));
    d_dis1(i) = (lx - y1(i))^2;
    t_obj = c * (d_dis1(i));
    obj = obj + t_obj;
    x1 = x;
    x1(1,i) = x1(1,i) - 0.55;
    x1(2,i) = x1(2,i) - 2.25;
    x1(3,i) = x1(3,i) + 1.5;
    lx = sqrt(x1(1:3,i)' * x1(1:3,i));
    d_dis2(i) = (lx - y2(i))^2;
    t_obj = c * (d_dis2(i));
    obj = obj + t_obj;
    vt = max([v(i),v1(i),v2(i)]);
    t_obj = d*(sqrt(x(4:6,i)'*x(4:6,i))-vt)^2;
    obj = obj + t_obj;
end



obj = obj + a* (x(:,1)-x0(:,1))'*(x(:,1)-x0(:,1));

f = obj;





