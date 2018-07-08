function [dW4]=syst_diff4(t,W4,lambda4,zita1,f4,lambda1);

w4=W4(1);
v4=W4(2);

dw4=v4;
dv4=-zita1*sqrt(lambda1)*2*v4 - lambda4*w4 + f4;

dW4=[dw4;dv4];

end