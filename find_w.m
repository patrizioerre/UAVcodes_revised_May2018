function [w,u01,u02,u03,u04,u05,u01d,u02d,u03d,u04d,u05d]=find_w(L_panel,fi1,fi2,fi3,fi4,fi5,u01,u02,u03,u04,u05,u01d,u02d,u03d,u04d,u05d,length_coor)
tic
%define variables
syms w1(t) w2(t) w3(t) w4(t) w5(t) w1diff(t) w2diff(t) w3diff(t) w4diff(t) w5diff(t) 
% syms f1(t) f2(t) f3(t) f4(t) f5(t) 
w1diff(t)=diff(w1(t));
w2diff(t)=diff(w2(t));
w3diff(t)=diff(w3(t));
w4diff(t)=diff(w4(t));
w5diff(t)=diff(w5(t));


%define thickness and damping ratio
thickness=0.0169;
density=76.3;
rho=density*thickness;
zita1=0.01;
zita2=0.01;
zita3=0.01;
zita4=0.01;
zita5=0.01;
% zita1=0;
% zita2=0;
% zita3=0;
% zita4=0;
% zita5=0;

% Load

% f1(t)=25*sin(lambda1*t);
% f1(t)=(10*max(max(abs(fi1)))/(rho))*2;
% % f1(t)=0;
% % % that's the f1 component of a load with two 10N point forces applied on the edges of the wing 
% f1(t)=7.56
% % f1= -43.229437461235619
% % f1=-1.038355415584229e+02
% % f1(t)=f1
% 
% % f1=52
% % 
% f1=0
% % f2(t)=(-500*max(max(abs(fi2)))/(rho))*2;
% % % f2(t)=0;
% 
% % f2=-1.011237585670643e+02
% % f2=67
% % f2=0
% % f2=16.3
% % f2(t)=f2
% % 
% f2=    67
% % f3(t)=(10*max(max(abs(fi3)))/(rho))*2;
% % f3(t)=0;
% % f3=48.8
% f3=-6.4
% % f3=     24.244848225899865
% % f3=  97.622725504167818
% % f3(t)=f3
% 
% % f4(t)=(10*max(max(abs(fi4)))/(rho))*2;
% f3=0
% % f4=     33.089913920878949
% % f4 = 87.044782716765454
% % f4=2.65
% % f4=43.5
% f4(t)=0;
% % f4(t)=f4
%    
% % 
% % f5(t)=(-500*max(max(abs(fi5)))/(rho))*2;
% % % f5(t)=0;
% f5=     -16.038219657594016
% f5=  -1.010539870716207e+02
% % f5(t)=f5
% % f5=-1.66
% % f5=67
% %f5=0
% f5=-67

lambda1=(30*2*pi)^2;
lambda2=(190*2*pi)^2;
lambda3=(210*2*pi)^2;
lambda4=(550*2*pi)^2;
lambda5=(617*2*pi)^2;

[fp1]=load_interp(L_panel,length_coor);

[f1,f2,f3,f4,f5]=load_projection_prove(fp1,fi1,fi2,fi3,fi4,fi5,length_coor);

% ODE system
ode1 = diff(w1,t,2) + 2*zita1*sqrt(lambda1)*diff(w1,t) + lambda1*w1 == f1 ;
ode2 = diff(w2,t,2) + 2*zita2*sqrt(lambda2)*diff(w2,t) + lambda2*w2 == f2 ;
ode3 = diff(w3,t,2) + 2*zita3*sqrt(lambda3)*diff(w3,t) + lambda3*w3 == f3 ;
ode4 = diff(w4,t,2) + 2*zita4*sqrt(lambda4)*diff(w4,t) + lambda4*w4 == f4 ;
ode5 = diff(w5,t,2) + 2*zita5*sqrt(lambda5)*diff(w5,t) + lambda5*w5 == f5 ;

% odes = [ode1];
% odes = [ode1; ode2];
odes = [ode1; ode2; ode3; ode4; ode5];

% Initial conditions 
cond1 = w1(0) == u01;
cond2 = w1diff(0) == u01d;
cond3 = w2(0) == u02;
cond4 = w2diff(0) == u02d;
cond5 = w3(0) == u03;
cond6 = w3diff(0) == u03d;
cond7 = w4(0) == u04;
cond8 = w4diff(0) == u04d;
cond9 = w5(0) == u05;
cond10 = w5diff(0) == u05d;

% conds = [cond1; cond2]
% % conds = [cond1; cond2; cond3; cond4];
conds = [cond1; cond2; cond3; cond4; cond5; cond6; cond7; cond8; cond9; cond10];

% Solve function 
% [w1Sol(t)] = dsolve(odes,conds);
% [w1Sol(t), w2Sol(t)] = dsolve(odes,conds);
[w1Sol(t), w2Sol(t),w3Sol(t), w4Sol(t),w5Sol(t)] = dsolve(odes,conds);

% Solution is given in functional form, with feval and double you turn it
% into an array
% tVal = linspace(0,20);
% yVal = feval(w1Sol,tVal);
% plot(tVal,yVal)
% double(yVal)

% Plotting solutions with fplot
% figure(1)
% fplot(w1Sol)
% xlim([0,10])
% ylim([-0.1,0.1])
time_step=0.00025;

w1=eval(subs(w1Sol,time_step));
w2=eval(subs(w2Sol,time_step));
w3=eval(subs(w3Sol,time_step));
w4=eval(subs(w4Sol,time_step));
w5=eval(subs(w5Sol,time_step));
% 
% w1=eval(subs(w1Sol,0.0044));
% w2=eval(subs(w2Sol,0.0044));
% w3=eval(subs(w3Sol,0.0044));
% w4=eval(subs(w4Sol,0.0044));
% w5=eval(subs(w5Sol,0.0044));

u01=w1;
u02=w2;
u03=w3;
u04=w4;
u05=w5;




w1d=eval(subs(diff(w1Sol,t),time_step));
w2d=eval(subs(diff(w2Sol,t),time_step));
w3d=eval(subs(diff(w3Sol,t),time_step));
w4d=eval(subs(diff(w4Sol,t),time_step));
w5d=eval(subs(diff(w5Sol,t),time_step));

% w1d=eval(subs(diff(w1Sol,t),0.0044));
% w2d=eval(subs(diff(w2Sol,t),0.0044));
% w3d=eval(subs(diff(w3Sol,t),0.0044));
% w4d=eval(subs(diff(w4Sol,t),0.0044));
% w5d=eval(subs(diff(w5Sol,t),0.0044));

u01d=w1d;
u02d=w2d;
u03d=w3d;
u04d=w4d;
u05d=w5d;


w = ( w1*fi1 +w2*fi2 +w3*fi3 +w4*fi4 +w5*fi5 )*rho;



% u01=trapz(y,trapz(x,w.*fi1,2))*rho;
% u02=trapz(y,trapz(x,w.*fi2,2))*rho;
% u03=trapz(y,trapz(x,w.*fi3,2))*rho;
% u04=trapz(y,trapz(x,w.*fi4,2))*rho;
% u05=trapz(y,trapz(x,w.*fi5,2))*rho;
% 
% u01d=trapz(y,trapz(x,w.*fi1,2))*rho;
% u02d=trapz(y,trapz(x,w.*fi2,2))*rho;
% u03d=trapz(y,trapz(x,w.*fi3,2))*rho;
% u04d=trapz(y,trapz(x,w.*fi4,2))*rho;
% u05d=trapz(y,trapz(x,w.*fi5,2))*rho;


% 
% figure(2)
% fplot(w2Sol)
% xlim([0,1])
% ylim([-0.1,0.1])
% 
% figure(3)
% fplot(w3Sol)
% xlim([0,10])
% ylim([-0.1,0.1])
% 
% figure(4)
% fplot(w4Sol)
% xlim([0,1])
% ylim([-0.1,0.1])
% 
% figure(5)
% fplot(w5Sol)
% xlim([0,10])
% ylim([-0.1,0.1])









% Find solution using known solution form with convolution
% t=linspace(0,1,1000);
% % f1=25*ones(1,length(t));
% 
% f1=zeros(1,length(t));
% f1(1)=25;
% 
% % f1=sin(lambda1*t);
% w1=1/sqrt(lam1)*conv(f1,sin(sqrt(lam1)*t),'same');
% figure(6)
% plot(w1)
toc
end