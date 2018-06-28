function [w,wdot,u01,u02,u03,u04,u05,u01d,u02d,u03d,u04d,u05d]=find_w(L_panel,fi1,fi2,fi3,fi4,fi5,u01,u02,u03,u04,u05,u01d,u02d,u03d,u04d,u05d,length_coor,time_step)
format long

thickness=0.0169;
density=76.3;
rho=density*thickness;

zita1=0.01;
zita2=0.01;
zita3=0.01;
zita4=0.01;
zita5=0.01;

lambda1=(30*2*pi)^2;
lambda2=(190*2*pi)^2;
lambda3=(210*2*pi)^2;
lambda4=(550*2*pi)^2;
lambda5=(617*2*pi)^2;

[fp1]=load_interp(L_panel,length_coor);

[f1,f2,f3,f4,f5]=load_projection_prove(fp1,fi1,fi2,fi3,fi4,fi5,length_coor);


tspan = [0 100*time_step];

w01 = u01;
v01 = u01d;
w02 = u02;
v02 = u02d;
w03 = u03;
v03 = u03d;
w04 = u04;
v04 = u04d;
w05 = u05;
v05 = u05d;

W01 = [w01 ; v01];
W02 = [w02 ; v02];
W03 = [w03 ; v03];
W04 = [w04 ; v04];
W05 = [w05 ; v05];

%[t,W] = ode45(@(v,w) -3*v-4*w, tspan, w0);
% [t,W] = ode45(@syst_diff1, tspan, W0);

[t1,W1] = ode45(@(t1,W1) syst_diff1(t1,W1,lambda1,zita1,f1), tspan, W01);
[t2,W2] = ode45(@(t2,W2) syst_diff2(t2,W2,lambda2,zita2,f2), tspan, W02);
[t3,W3] = ode45(@(t3,W3) syst_diff3(t3,W3,lambda3,zita3,f3), tspan, W03);
[t4,W4] = ode45(@(t4,W4) syst_diff4(t4,W4,lambda4,zita4,f4), tspan, W04);
[t5,W5] = ode45(@(t5,W5) syst_diff5(t5,W5,lambda5,zita5,f5), tspan, W05);

w1=interp1(t1,W1(:,1),time_step);
w2=interp1(t2,W2(:,1),time_step);
w3=interp1(t3,W3(:,1),time_step);
w4=interp1(t4,W4(:,1),time_step);
w5=interp1(t5,W5(:,1),time_step);

u01=w1;
u02=w2;
u03=w3;
u04=w4;
u05=w5;


w1d=interp1(t1,W1(:,2),time_step);
w2d=interp1(t2,W2(:,2),time_step);
w3d=interp1(t3,W3(:,2),time_step);
w4d=interp1(t4,W4(:,2),time_step);
w5d=interp1(t5,W5(:,2),time_step);

u01d=w1d;
u02d=w2d;
u03d=w3d;
u04d=w4d;
u05d=w5d;

w = ( w1*fi1 +w2*fi2 +w3*fi3 +w4*fi4 +w5*fi5 )*rho;

wdot = ( w1d*fi1 +w2d*fi2 +w3d*fi3 +w4d*fi4 +w5d*fi5 )*rho;



% plot(t1,W1(:,1))
%[w,v] = ode45(@(w,v) v, tspan, w0d);

end

