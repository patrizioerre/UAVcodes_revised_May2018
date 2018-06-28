function [dW4]=syst_diff4(t,W4,lambda4,zita4,f4);

w4=W4(1);
v4=W4(2);

dw4=v4;
dv4=-zita4*sqrt(lambda4)*2*v4 - lambda4*w4 + f4;

dW4=[dw4;dv4];

end