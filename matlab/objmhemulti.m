function f = objmhemulti(x, x0, u, uwb_d, uwb_d1, uwb_d2, uwb_v0, uwb_v1, uwb_v2)
obj = 0;
% gamma_1 = 1.2;
% gamma_2 = 10;
% gamma_3 = 100;
% gamma_4 = 3;
gamma_1 = 1.2/10.0;
gamma_2 = 30/10.0;
gamma_3 = 100.0;
gamma_4 = 30/10.0;

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
    t_obj = (xt(:, i) - x(:, i))' * (xt(:, i) - x(:, i)); 
    obj = obj + gamma_1 * t_obj; % 1st term
    %% first anchor
    estimate_d = sqrt(x(1:3,i)' * x(1:3,i));
    d_dis(i) = (estimate_d - uwb_d(i))^2;
    t_obj =  gamma_2 * (estimate_d - uwb_d(i))^2;
    obj = obj + t_obj;    % 1st term + 2nd term
%     %% second anchor
%     x1 = x;
%     x1(1,i) = x1(1,i) + 0.15; % bias
%     x1(2,i) = x1(2,i) + 1.8;
%     x1(3,i) = x1(3,i) + 1.95;
% 
%     estimate_d = sqrt(x1(1:3,i)' * x1(1:3,i));
%     d_dis1(i) = (estimate_d - uwb_d1(i))^2;
%     t_obj = c * (d_dis1(i));
%     obj = obj + t_obj;
%     
%    %% third anchor
%     x1 = x;
%     x1(1,i) = x1(1,i) - 0.55; % bias
%     x1(2,i) = x1(2,i) - 2.25;
%     x1(3,i) = x1(3,i) + 1.5;
% 
%     estimate_d = sqrt(x1(1:3,i)' * x1(1:3,i));
%     d_dis2(i) = (estimate_d - uwb_d2(i))^2;
%     t_obj = c * (d_dis2(i));
%     obj = obj + t_obj;
   
    %% velocity   
%     uwb_max_v = max([uwb_v0(i),uwb_v1(i),uwb_v2(i)]);
%    uwb_max_v = max([uwb_v0(i),uwb_v1(i)]);
     uwb_max_v = uwb_v0(i);
    estimate_v = sqrt(x(4:6,i)'*x(4:6,i));
    v_dis(i) = ((uwb_max_v-estimate_v)^2)*exp((uwb_max_v+0.1)/(estimate_v+0.1));  
    t_obj = gamma_4*v_dis(i);
    obj = obj + t_obj; % 1st term + 2nd term + fourth term

end

obj = obj + gamma_3* (x(:,1)-x0(:,1))'*(x(:,1)-x0(:,1)); % 1st term + 2nd term + fourth term + third term

f = obj;  % function with regard to x





