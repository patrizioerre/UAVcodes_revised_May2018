function [dW5]=syst_diff5(t,W5,lambda5,zita1,f5,lambda1);

w5=W5(1);
v5=W5(2);

dw5=v5;
dv5=-zita1*sqrt(lambda1)*2*v5 - lambda5*w5 + f5;

dW5=[dw5;dv5];
end