function [w,wdot,u01,u02,u03,u04,u05,u01d,u02d,u03d,u04d,u05d]...
 = find_w(L_panel,fi1,fi2,fi3,fi4,fi5,u01,u02,u03,u04,u05,u01d,u02d,u03d,u04d,u05d,length_coor,time_step)

% SYNTAX
%        [x, xdot, x2dot] = newmarkint(M, C, K, R, x0, xdot0, t, varargin)
% INPUT
%        [M] :       System Mass              [n,n]
%        [C] :       System Damping           [n,n]
%        [K] :       System Stiffness         [n,n]
%        [R] :       Externally Applied Load  [n,nt]
%        [x0] :      Initial Position         [n,1]
%        [xdot0] :   Initial Velocity         [n,1]
%        [t] :       Time Vector              [1,nt]
%        [varargin]: Options
%
% OUTPUT
%       [x]:        Displacemente Response   [n,nt]
%       [xdot]:     Velocity                 [n,nt]
%       [x2dot]:    Acceleration             [n,nt]
%
%
%  nt = number of time steps
%  n = number of nodes
% The options include changing the value of the "alfa" and "beta"
% coefficient which appear in the formulation of the method. By default
% these values are set to alfa = 1/2 and beta = 1/4.
%
% EXAMPLE
% To change nemark's coefficients, say to alfa = 1/3 and beta = 1/5, 
% the syntax is:
%       [u, udot, u2dot] = newmark_int(t,p,u0,udot0,m,k,xi, 1/3, 1/5)  
%
%-------------------------------------------------------------------------

    alfa = 1 / 2;
    beta = 1 / 4;

format long

thickness=0.005;
density=38.15;
rho=density*thickness;

zita1=0.02;
zita2=0.01;
zita3=0.01;
zita4=0.01;
zita5=0.01;

lambda1=(11.641*2*pi)^2;
lambda2=(81.496*2*pi)^2;
lambda3=(72.850*2*pi)^2;
lambda4=(203.70*2*pi)^2;
lambda5=(240.70*2*pi)^2;


[fp1]=load_interp(L_panel,length_coor);

[f1,f2,f3,f4,f5]=load_projection_prove(fp1,fi1,fi2,fi3,fi4,fi5,length_coor,density,thickness);

R1=[f1 f2 f3 f4 f5]';

M=[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 1 0;0 0 0 0 1];

K=[lambda1 0 0 0 0;0 lambda2 0 0 0;0 0 lambda3 0 0;0 0 0 lambda4 0;0 0 0 0 lambda5];

C=[2*sqrt(lambda1)*zita1 0 0 0 0;0 2*sqrt(lambda2)*zita1 0 0 0;0 0 2*sqrt(lambda1)*zita1 0 0;0 0 0 2*sqrt(lambda1)*zita1 0;0 0 0 0 2*sqrt(lambda1)*zita1];

x0=[u01 u02 u03 u04 u05]';

xdot0=[u01d u02d u03d u04d u05d]';



dt = time_step/10;
t=linspace(0,time_step,10);
nt = fix((t(end)- t(1)) / dt);
n = length(M);

% Constants used in Newmark's integration
a1 = alfa / (beta * dt);
a2 = 1 / (beta * dt ^ 2);
a3 = 1 / (beta * dt);
a4 = alfa / beta;
a5 = 1/(2 * beta);
a6 = (alfa / (2 * beta) - 1) * dt;


x = zeros(n,nt);
xdot = zeros(n,nt);
x2dot = zeros(n,nt);

% Initial Conditions
x(:, 1) = x0;
xdot(:, 1) = xdot0;
R = [R1 zeros(n,nt)];
x2dot(:,1) = M \ (R(:, 1) - C * xdot(:, 1) - K * x(:, 1)) ;

Kcap = K + a1 * C + a2 * M;

a = a3 * M + a4 * C;
b = a5 * M + a6 * C;

% Time step starts
for i = 1 : nt
    delR = R(:, i) + a * xdot(:, i) + b * x2dot(:, i);
    delx = Kcap \ delR ;
    delxdot = a1 * delx - a4 * xdot(:, i) - a6 * x2dot(:, i);
    delx2dot = a2 * delx - a3 * xdot(:, i) - a5 * x2dot(:, i);
    x(:, i + 1) = x(:, i) + delx;
    xdot(:, i + 1) = xdot(:, i) + delxdot;
    x2dot(:, i + 1) = x2dot(:, i) + delx2dot;
end

u01=x(1,end);
u02=x(2,end);
u03=x(3,end);
u04=x(4,end);
u05=x(5,end);
u01d=xdot(1,end);
u02d=xdot(2,end);
u03d=xdot(3,end);
u04d=xdot(4,end);
u05d=xdot(5,end);


% for i=1:size(x(1,:),2)
%     vert_displ1(:,:,i) = ( x(1,i)*fi1  )*rho;
%     %vert_displ(:,:,i) = ( W1(i,1)*fi1 +W2(i,1)*fi2 +W3(i,1)*fi3 +W4(i,1)*fi4 +W5(i,1)*fi5 )*rho;
%     %wdot(:,:,i) = ( W1(i,2)*fi1 +W2(i,2)*fi2 +W3(i,2)*fi3 +W4(i,2)*fi4 +W5(i,2)*fi5 )*rho;
%     
%     disp(i)=vert_displ1(length_coor,length_coor/2,i);
% end

w = ( x(1,end)*fi1 +x(2,end)*fi2 +x(3,end)*fi3 +x(4,end)*fi4 +x(5,end)*fi5 )*rho;

wdot = ( xdot(1,end)*fi1 +xdot(2,end)*fi2 +xdot(3,end)*fi3 +xdot(4,end)*fi4 +xdot(5,end)*fi5 )*rho;

end