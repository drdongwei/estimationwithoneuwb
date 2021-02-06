function f = objmhe(x, x0, u, y)
obj = 0;
a = 100;
b = 1.2;
c = 8;
d = 5;

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

for i = 1:length(y)
    xt(:, i+1) = A * xt(:, i) + B * u(:, i);
    t_obj = (xt(:, i+1) - x(:, i))'*(xt(:, i+1) - x(:, i));
    lx = sqrt(x(1:3,i)'*x(1:3,i));
    t_obj = b*t_obj + c*(lx - y(i))^2;
    obj = obj + t_obj;
%     if(i<length(gt(1,:)))
%        t_obj = (x(:, i) - gt(:, i))'*(x(:, i) - gt(:, i));
%        obj = obj + a * t_obj;
%     end
%     if(i>1)
%        t_obj = (x(4:6, i) - x(4:6, i-1))'*(x(4:6, i) - x(4:6, i-1));
%        obj = obj +  d* t_obj;
%     end
end

for i = 1:6
    av = mean(x(i,:));
    t_obj = (x(i, :) -av)*(x(i, :) - av)';
    obj = obj + d* t_obj;
end
obj = obj + a* (x(:,1)-x0(:,1))'*(x(:,1)-x0(:,1));

f = obj;





