function [dW2]=syst_diff2(t,W2,lambda2,zita1,f2,lambda1);

w2=W2(1);
v2=W2(2);

dw2=v2;
dv2=-2*zita1*sqrt(lambda1)*v2 - lambda2*w2 + f2;

dW2=[dw2;dv2];

end