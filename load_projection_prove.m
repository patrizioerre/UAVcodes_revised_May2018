
function [f1,f2,f3,f4,f5]=load_projection_prove(fp1,fi1,fi2,fi3,fi4,fi5,length_coor)
%load definition
% 
% length_coor=200;
y=linspace(0,0.912,length_coor)';
x=linspace(0,0.30,length_coor)';
length_coor=length(x);
fp=zeros(length_coor);
% fp(length_coor,length_coor)=-10; %trailing edge tip
% fp(length_coor,1)=10;    %leading edge tip
% fp(length_coor,length_coor/2)=12.7
% fp(length_coor,length_coor)=0; %trailing edge tip
% fp(length_coor,1)=20;    %leading edge tip
% fp(length_coor,length_coor/2)=0
% fp(length_coor,length_coor/4)=20

% fp(50,100)=1000;
% fp=(0.135/(length(x)*length(y)))*ones(length(x),length(y)); 
% it corresponds to the total force on the 0.15x0.9 wingbox with 1Pa applied
% 
% sinus=sin(6.28/3.6*y);
% fp=[sinus,1]
% 
% fp=10*mode1s;
% fp=L
% fp=load_interp(L_panel,x,y)

fp=fp1;
f=fp/(76.3*0.0169);


%load projection on eigenfunctions
% f1=sum(sum(times(f,fi1)))/area;
% f2=sum(sum(times(f,fi2)))/area;
% f3=sum(sum(times(f,fi3)))/area;
% f4=sum(sum(times(f,fi4)))/area;
% f5=sum(sum(times(f,fi5)))/area;

% f1=trapz(y,trapz(x,f.*mode1s,2))./trapz(y,trapz(x,abs(mode1s.*mode1s),2));
% f2=trapz(y,trapz(x,f.*mode2s,2))./trapz(y,trapz(x,abs(mode2s.*mode2s),2));
% f3=trapz(y,trapz(x,f.*mode3s,2))./trapz(y,trapz(x,abs(mode3s.*mode3s),2));
% f4=trapz(y,trapz(x,f.*mode4s,2))./trapz(y,trapz(x,abs(mode4s.*mode4s),2));
% f5=trapz(y,trapz(x,f.*mode5s,2))./trapz(y,trapz(x,abs(mode5s.*mode5s),2));
area=(0.3/length_coor)*(0.912/length_coor);
area=3.6;
area=1;
f1=sum(sum(times(f,fi1)))/area;
f2=sum(sum(times(f,fi2)))/area;
f3=sum(sum(times(f,fi3)))/area;
f4=sum(sum(times(f,fi4)))/area;
f5=sum(sum(times(f,fi5)))/area;
% f11=trapz(y,trapz(x,f.*fi1,2))/area;
% f21=trapz(y,trapz(x,f.*fi2,2))/area;
% f31=trapz(y,trapz(x,f.*fi3,2))/area;
% f41=trapz(y,trapz(x,f.*fi4,2))/area;
% f51=trapz(y,trapz(x,f.*fi5,2))/area;
% 

% f_surf=f1.*mode1s+f2.*mode2s+f3.*mode3s+f4.*mode4s+f5.*mode5s;
f_surf=f1*fi1+f2*fi2+f3*fi3+f4*fi4+f5*fi5;
% f_surf1=f11*fi1+f21*fi2+f31*fi3+f41*fi4+f51*fi5;

figure(10)
surf(x,y,f_surf)
% hold on
% surf(x,y,f_surf1)
end