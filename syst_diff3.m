function [dW3]=syst_diff3(t,W3,lambda3,zita1,f3,lambda1);

w3=W3(1);
v3=W3(2);

dw3=v3;
dv3=-zita1*sqrt(lambda1)*2*v3 - lambda3*w3 + f3;

dW3=[dw3;dv3];

end