function [dW3]=syst_diff3(t,W3,lambda3,zita3,f3);

w3=W3(1);
v3=W3(2);

dw3=v3;
dv3=-zita3*sqrt(lambda3)*2*v3 - lambda3*w3 + f3;

dW3=[dw3;dv3];

end