function f = objmhemulti(x, x0, u, y, y1, y2, v, v1, v2)
obj = 0;
% gamma_1 = 1.2;
% gamma_2 = 10;
% gamma_3 = 100;
% gamma_4 = 3;
gamma_1 = 1.2/10.0;
gamma_2 = 30/10.0;
gamma_3 = 100.0;
gamma_4 = 10/10.0;

dt = 0.04;

A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt,;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];
  
  B = [0.5*dt^2, 0, 0;
     0, 0.5*dt^2, 0;
     0, 0, 0.5*dt^2;
     dt, 0,  0;
     0,  dt, 0;
     0,  0, dt;];

xt(:,1) = x0;

for i = 1:length(u)
    xt(:, i+1) = A  *  x(:, i) + B  *  u(:, i);  % predict
    t_obj = (xt(:, i) - x(:, i))' * (xt(:, i) - x(:, i)); % temp save
    obj = obj + gamma_1 * t_obj; % 1st term
    %% first anchor
    lx = sqrt(x(1:3,i)' * x(1:3,i));
    d_dis(i) = (lx - y(i))^2;
    t_obj =  gamma_2 * (lx - y(i))^2;
    obj = obj + t_obj;    % 1st term + 2nd term
%     %% second anchor
%     x1 = x;
%     x1(1,i) = x1(1,i) + 0.15;
%     x1(2,i) = x1(2,i) + 1.8;
%     x1(3,i) = x1(3,i) + 1.95;
% 
%     lx = sqrt(x1(1:3,i)' * x1(1:3,i));
%     d_dis1(i) = (lx - y1(i))^2;
%     t_obj = c * (d_dis1(i));
%     obj = obj + t_obj;
%     
%    %% third anchor
%     x1 = x;
%     x1(1,i) = x1(1,i) - 0.55;
%     x1(2,i) = x1(2,i) - 2.25;
%     x1(3,i) = x1(3,i) + 1.5;
% 
%     lx = sqrt(x1(1:3,i)' * x1(1:3,i));
%     d_dis2(i) = (lx - y2(i))^2;
%     t_obj = c * (d_dis2(i));
%     obj = obj + t_obj;
   
    %% velocity   
%     vt = max([v(i),v1(i),v2(i)]);
%    vt = max([v(i),v2(i)]);
     vt = v(i);
    t_vs = sqrt(x(4:6,i)'*x(4:6,i));
    v_dis(i) = ((vt-t_vs)^2)*exp((vt+0.1)/(t_vs+0.1));  
    t_obj = gamma_4*v_dis(i);
    obj = obj + t_obj; % 1st term + 2nd term + fourth term

end


obj = obj + gamma_3* (x(:,1)-x0(:,1))'*(x(:,1)-x0(:,1)); % 1st term + 2nd term + fourth term + third term

f = obj;  % function with regard to x





