function f = objmhe_s(x, x0, u, y, vy)
obj = 0;
a = 100;
b = 1.2;
c = 10;
d = 5;
e = 100;
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

lu = length(u);

for i = 1:lu
    xt(:, i+1) = A  *  xt(:, i) + B  *  u(:, i);
    t_obj = (xt(:, i+1) - x(:, i))' * (xt(:, i+1) - x(:, i));
    lx = sqrt(x(1:3,i)' * x(1:3,i));
    t_obj = b * t_obj ;
    obj = obj + t_obj;
    d_dis(i) = (lx - y(i))^2;
    k = i;
    if i == lu
        k = i-1;
    end
    [az0,el0,rho0] = cart2sph(x(4, k), x(5, k), x(6, k));

%     svt = xt(4:6, i) * vy(i)/vr;
%     vs = sqrt(svt'*svt);
%     svt = xt(4:6, k+1)'*xt(4:6, k+1)-vr^2;
%     vs = sqrt(max([svt+vy(i)^2,0]));
    v_dis(i) = (rho0 - vy(i))^2;
%      v_dis(i) = (vr - vy(i))^2;
    t_obj = c * d_dis(i) + d * (v_dis(i));
    obj = obj + t_obj;
end

 d_q = max(d_dis);
 v_q = max(v_dis);
 obj = obj + e * d_q + f * v_q;


obj = obj + a* (x(:,1)-x0(:,1))'*(x(:,1)-x0(:,1));

f = obj;





