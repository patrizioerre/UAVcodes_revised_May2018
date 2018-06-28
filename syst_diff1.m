function [dW1]=syst_diff1(t,W1,lambda1,zita1,f1);

w1=W1(1);
v1=W1(2);

dw1=v1;
dv1=-2*zita1*sqrt(lambda1)*v1 - lambda1*w1 + f1;

dW1=[dw1;dv1];

end