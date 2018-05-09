function [Fb,Mb]=UAVFM_backup(AOA,beta,de,da,dr,Vb)


% Steady UAV panel code method by B. Davoudi including wing, tail and rudder 
% Aerospace Engineering Department, University of Michigan Ebe 2018

%% aircraft characteristics

Nxw=4;Nxt=4;               % chordwise panel per hald wing
Nyw=9;Nyt=5;               % spanwise panel per hald wing
Nxr=2;Nyr=6;
ARw=176.5^2/7150;             % Aspect ratio
ARt=3;
ARr=3;
bw=2;bt=bw*64.4/176.5;         % wing span
br=bt;
%bt=1;ARt=ARw;
trw=1.8;trt=2;trr=5;        % taper ratio
%Lam=degtorad(10);             % sweep angle, backward swept, positive
%dih=degtorad(5);             % dihedral angle defined at the c/4

Lam=10*pi/180;
dih=5*pi/180;

%% comands

%aoa=degtorad(AOA);             % angle of attack
aoa=AOA*pi/180;
u=Vb;                          % incidence velocity vector

% aoaf_Lw=degtorad(da);aoaf_Lt=degtorad(de);
% aoaf_Rw=degtorad(-da);aoaf_Rt=degtorad(de);
% aoar=degtorad(dr);

aoaf_Lw=-da*pi/180;aoaf_Lt=-de*pi/180;
aoaf_Rw=da*pi/180;aoaf_Rt=-de*pi/180;
aoar=dr*pi/180;


%% geometry of the UAV

% wing
[xw,yw,zw,xcolw,ycolw,zcolw,nw,dl_xw,dlyw,Sw,alphaw,~]=geometry(ARw,bw,trw,Nxw,Nyw,Lam,dih,aoa,aoaf_Lw,aoaf_Rw);
% tail
% I set aoa equal to zero for tail
[xt,yt,zt,xcolt,ycolt,zcolt,nt,dl_xt,dlyt,St,alphat,~]=geometry(ARt,bt,trt,Nxt,Nyt,Lam,dih,-2*pi/180,aoaf_Lt,aoaf_Rt);

% moving the tail to the aft
xt=bw/2+xt;xcolt=bw/2+xcolt;
zt=bw/16+zt;zcolt=bw/16+zcolt;

% rudder
[xr1,yr1,zr1,xcolr1,ycolr1,zcolr1,nr1,dl_xr,dlyr,Sr,alphar1,~]=geometry(ARr,br,trr,Nxr,Nyr,Lam,0,0,aoar,aoar);

% choosing the right wing (pilot right)
% choosing the positive y part of the wing - should not add the zoffset
% here!!
zr=zr1(:,Nyr+1:end);xr=bw/2+xr1(:,Nyr+1:end);yr=yr1(:,Nyr+1:end);
zcolr=zcolr1(:,Nyr+1:end);xcolr=bw/2+xcolr1(:,Nyr+1:end);ycolr=ycolr1(:,Nyr+1:end);
alphar=alphar1(:,Nyr+1:end);

% modifying normal vector for rudder
nr2=[nr1(:,Nyr+1:2*Nyr),nr1(:,3*Nyr+1:end)];
nr=zeros(3,Nyr*Nxr);
theta=90;

for i=1:Nxr*Nyr
    nr(:,i) =  [1,0,0;0,cosd(theta),-sind(theta);0,sind(theta),cosd(theta)]*nr2(:,i);
end


% constructing x,y,x that incorporates wing, tail and rudder locations

Ny=Nyw;

x=[xw;[xt,zeros(Nxt,2*Ny-2*Nyt)];[xr,zeros(Nxr,2*Ny-Nyr)]];y=[yw;[yt,zeros(Nxt,2*Ny-2*Nyt)];[yr,zeros(Nxr,2*Ny-Nyr)]];z=[zw;[zt,zeros(Nxt,2*Ny-2*Nyt)];[zr,zeros(Nxr,2*Ny-Nyr)]];
xcol=[xcolw;[xcolt,zeros(Nxt,2*Ny-2*Nyt)];[xcolr,zeros(Nxr,2*Ny-Nyr)]];ycol=[ycolw;[ycolt,zeros(Nxt,2*Ny-2*Nyt)];[ycolr,zeros(Nxr,2*Ny-Nyr)]];
zcol=[zcolw;[zcolt,zeros(Nxt,2*Ny-2*Nyt)];[zcolr,zeros(Nxr,2*Ny-Nyr)]];n=[nw,nt,nr];dl_x=[dl_xw,dl_xt,dl_xr];dly=[dlyw,dlyt,dlyr];
alpha=[alphaw;[alphat,zeros(Nxt,2*Ny-2*Nyt)];[alphar,zeros(Nxr,2*Ny-Nyr)]];

%% solver

% solving for vorticty
[G,~,~,a,a_d,w_ind_drag]=fast_steady_tail_rudder(x,y,z,xcol,ycol,zcol,n,dl_x,dly,Nxw,Nxt,Nxr,Nyw,Nyt,Nyr,u,alpha,Lam,dih,bw);

plot(G,'.')

% calculate forces

% Gww(1:2*Nyw)= G(1+2*(Nxw-1)*Nyw:2*Nxw*Nyw);
% Gwt(1:2*Nyt)= G(1+2*Nxw*Nyw+2*(Nxt-1)*Nyt:2*Nxw*Nyw+2*Nxt*Nyt);
% Gwr(1:Nyr)= G(1+2*Nxw*Nyw+2*Nxt*Nyt+(Nxr-1)*Nyr:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr);

t=1;dt=1;

%[cLnewv,Lift,Drag,dp,d,roll,yaw]
[~,Liftw,Dragw,~,~,rollw,yaww]=force_calc(Nxw,Nyw,a(:,1:2*Nxw*Nyw),a_d(:,1:2*Nxw*Nyw),w_ind_drag(1:2*Nxw*Nyw),bw,G(1:2*Nxw*Nyw,:),G(1:2*Nxw*Nyw,1),t,dt,Sw,dl_x(1:2*Nyw),dly(1),norm(Vb,2),aoa,Lam,dih,aoaf_Lw,aoaf_Rw,0);
%                                     force_calc(Nx,  Ny,a,               a_d,               w_ind_drag, b,G,               G5,               t,dt,S,dl_x,               dly,   uinf,aoa,Lam,dih,aoaf_L ,aoaf_R)
[~,Liftt,Dragt,~,~,rollt,yawt]=force_calc(Nxt,Nyt,a(:,1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),a_d(:,1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),w_ind_drag(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),bt,G(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,:),G(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,1),t,dt,St,dl_x(1+2*Nyw:2*(Nyw+Nyt)),dly(2),norm(u,2),aoa,Lam,dih,aoaf_Lt,aoaf_Rt,0);

% we can add the effct of side slip angle as aoa for rudder -- just saying
[~,Liftr,Dragr,~,~,~,~]=force_calc(Nxr,Nyr/2,a(:,1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),a_d(:,1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),w_ind_drag(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),br,G(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,:),G(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,1),t,dt,Sr,dl_x(1+2*Nyw+2*Nyt:2*(Nyw+Nyt)+Nyr),dly(3),norm(u,2),0,Lam,0,0,aoar,1);

Gtest=G(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr);
% plot(a_d(:,1:2*Nxw*Nyw+2*Nxt*Nyt))

% positive x,y,x --> point to the airplane nose, right wing, ground
% a positive aoa leads to negative lift
% a postive beta (means winf from hand side of the pilot)
% leads to negative side force
% drag is always negative

Lift=Liftw+Liftt;

Liftr

% Dragr seems to be so wrong!
Drag=(Dragw+Dragt)*cos(beta*pi/180);
Side=Drag*sin(beta*pi/180);

roll=rollw+rollt;

xcgoffset=0.04*bw; % behind the wing ac
yaw=yaww+yawt+Liftr*(bw/2-xcgoffset);

% pitch=Liftw*(xcgoffset)-Liftt*(bw/2-xcgoffset);

% assuming xcg is ahead of the Liftw vector and pitch up positive

pitch=-Liftw*xcgoffset-Liftt*(bw/2+xcgoffset);


Fb=[-Drag;-Side;-Lift];
Mb=[roll;pitch;yaw];

%%  plotting real wing -- converting the middle points data to edges.

% zr=bw/16+yr1(:,Nyr+1:end);xr=bw/2+xr1(:,Nyr+1:end);yr=zr1(:,Nyr+1:end);
% 
% figure(4)
% 
% sp(dps,x,y,z,b,b,Nx,Ny,tr,S,Lam,dih,aoa,aoaf_L,aoaf_R,1,max(max(dps)),0,0);
% 
% toc
% 
% figure(5)
% 
% sp(d,x,y,z,b,b,Nx,Ny,tr,S,Lam,dih,aoa,aoaf_L,aoaf_R,1,max(max(d)),0,0);

end