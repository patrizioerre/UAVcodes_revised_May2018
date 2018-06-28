function [dW5]=syst_diff5(t,W5,lambda5,zita5,f5);

w5=W5(1);
v5=W5(2);

dw5=v5;
dv5=-zita5*sqrt(lambda5)*2*v5 - lambda5*w5 + f5;

dW5=[dw5;dv5];
end