% Simplified unsteady 3D UAV Vortex Panel Method by B. Davoudi 5/6/2018

tic
clc; clear all;
Nxw=4;Nxt=4;               % chordwise panel per hald wing
Nyw=9;Nyt=5;               % spanwise panel per hald wing
Nxr=2;Nyr=10;
ARw=176.5^2/7150;             % Aspect ratio
ARt=3;
ARr=3;
bw=1;bt=bw*64.4/176.5;         % wing span
br=bt;
%bt=1;ARt=ARw;
trw=1.8;trt=2;trr=2;        % taper ratio
Lam=degtorad(0);             % sweep angle, backward swept, positive
dih=degtorad(0);             % dihedral angle defined at the c/4
aoa=degtorad(7);             % angle of attack
uinf=1;                      % incidence velocity
u=uinf*[1 0 0];              % incidence velocity vector

aoaf_Lw=degtorad(6);aoaf_Lt=degtorad(-5);  % left (-y) and right (+y) is looking from aft
aoaf_Rw=degtorad(-6);aoaf_Rt=degtorad(5);
aoar=degtorad(10);

%% geometry of the UAV

% wing
[xw,yw,zw,xcolw,ycolw,zcolw,nw,dl_xw,dlyw,Sw,alphaw,crw]=geometry(ARw,bw,trw,Nxw,Nyw,Lam,dih,aoa,aoaf_Lw,aoaf_Rw);
% tail
% I set aoa equal to zero for tail
[xt,yt,zt,xcolt,ycolt,zcolt,nt,dl_xt,dlyt,St,alphat,~]=geometry(ARt,bt,trt,Nxt,Nyt,Lam,dih,-2 * pi/180,aoaf_Lt,aoaf_Rt);

% moving the tail to the aft
xt=bw/2+xt;xcolt=bw/2+xcolt;
zt=bw/16+zt;zcolt=bw/16+zcolt;

% rudder
[xr1,yr1,zr1,xcolr1,ycolr1,zcolr1,nr1,dl_xr1,dlyr,Sr,alphar1,~]=geometry(ARr,br,trr,Nxr,Nyr,Lam,0,0,aoar,aoar);

% choosing the right wing (pilot right)
% choosing the positive y part of the wing - should not add the zoffset
% here!!
zr=zr1(:,Nyr+1:end);xr=bw/2+xr1(:,Nyr+1:end);yr=yr1(:,Nyr+1:end);
zcolr=zcolr1(:,Nyr+1:end);xcolr=bw/2+xcolr1(:,Nyr+1:end);ycolr=ycolr1(:,Nyr+1:end);
alphar=alphar1(:,Nyr+1:end);dl_xr=dl_xr1(Nyr+1:end);

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

%% Steady Solution

% steady state initial solution
[Gs,Am_wing,A,a,a_d,w_ind_drag]=fast_steady_tail_rudder(x,y,z,xcol,ycol,zcol,n,dl_x,dly,Nxw,Nxt,Nxr,Nyw,Nyt,Nyr,u,alpha,Lam,dih,bw);

figure
plot(Gs); hold on;
 
figure
t=1;dt=1;
[cL,Liftw,Dragw,dps,ds,rollw,yaww]=force_calc(Nxw,Nyw,a(:,1:2*Nxw*Nyw),a_d(:,1:2*Nxw*Nyw),w_ind_drag(1:2*Nxw*Nyw),bw,Gs(1:2*Nxw*Nyw,:),Gs(1:2*Nxw*Nyw,1),t,dt,Sw,dl_x(1:2*Nyw),dly(1),norm(u,2),aoa,Lam,dih,aoaf_Lw,aoaf_Rw,0);                              
plot(yw(1,:),cL);hold on;xlabel('span');ylabel('C_l');grid minor;

%% wake extention from TE of physical surfaces

%time step
dt=0.5*min(dl_xw)*Nxw/uinf;   %%%%%%%%%%%%%%%%%%%%%% 1/4

% maximum simulation time
tmax=floor(bw/(uinf*dt));

for i=1:tmax
    xww(i,:)=xw(Nxw,:)+dl_x(1:2*Nyw).*cos(alphaw(end,:))+uinf*(i-1)*dt;
    yww(i,:)=yw(Nxw,:);
    zww(i,:)=zw(Nxw,:)-dl_x(1:2*Nyw).*sin(alphaw(end,:))*cos(dih);
    if i<=tmax/2
        % size of the tail wake
        xwt(i,:)=xt(Nxt,:)+dl_x(2*Nyw+1:2*(Nyt+Nyw)).*cos(alphat(end,:))+uinf*(i-1)*dt;
        ywt(i,:)=yt(Nxt,:);
        zwt(i,:)=zt(Nxt,:)-dl_x(2*Nyw+1:2*(Nyt+Nyw)).*sin(alphat(end,:))*cos(dih);
        % size of rudder wake
        xwr(i,:)=xr(Nxr,:)+dl_x(2*Nyw+2*Nyt+1:2*(Nyt+Nyw)+Nyr).*cos(alphar(end,:))+uinf*(i-1)*dt;
        ywr(i,:)=yr(Nxr,:);
        zwr(i,:)=zr(Nxr,:)-dl_x(2*Nyw+2*Nyt+1:2*(Nyt+Nyw)+Nyr).*sin(alphar(end,:))*cos(0);
    end
end

figure
surf(xt,yt,zt); hold on;surf(xw,yw,zw);surf(xr,yr,zr);surf(xwr,ywr,zwr);surf(xww,yww,zww);surf(xwt,ywt,zwt);axis('equal');

%% unsteady solution, for instance: response to a gust

% the initial wake vortex strength obtained by steady solution
Gww=repmat(Gs(1+2*(Nxw-1)*Nyw:2*Nxw*Nyw),tmax,1);
Gwt=repmat(Gs(1+2*Nxw*Nyw+2*(Nxt-1)*Nyt:2*Nxt*Nyt+2*Nxw*Nyw),floor(tmax/2),1);
Gwr=repmat(Gs(1+2*Nxw*Nyw+2*Nxt*Nyt+(Nxr-1)*Nyr:2*Nxt*Nyt+2*Nxw*Nyw+Nxr*Nyr),floor(tmax/2),1);
% gust info
vtmax=tmax;
xvor=-bw/2;
yvor=bw/2-bw/32;
zvor=-bw/16;
rc=0.162*bw/8;
% Gv=1.122*uinf*b/8;
Gv=0;

for t=1:vtmax
    t
    xvor=xvor+uinf*dt;
    
    
    for i=1:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr  % total of the panels in the wing and wake
        
        % once the panels on the wing are considered, tail panels are the next
        % starting from 2*Nxw*Nyw+1 to 2*Nxw*Nyw+2*Nxt*Nyt
        %% physical surfaces (wing, tail and rudder)
        % for wing
        if i<=2*Nxw*Nyw
            index1col=ceil(i/(2*Nyw));
            index2col=i-(2*Nyw)*(index1col-1);
            %        once we start consdiering the tail panels
        elseif i>2*Nxw*Nyw && i<=2*Nxw*Nyw+2*Nxt*Nyt
            index1col=Nxw+ceil((i-2*Nxw*Nyw)/(2*Nyt));                     % goes from Nxw+1 to Nxt
            index2col=i-2*Nxw*Nyw-(2*Nyt)*(index1col-Nxw-1);               % goes from 1 to Nyt
            %        once we start consdiering the rudder panels
        elseif i>2*Nxw*Nyw+2*Nxt*Nyt
            index1col=Nxw+Nxt+ceil((i-2*Nxw*Nyw-2*Nxt*Nyt)/Nyr);       % goes from Nxr+1 to Nxr
            index2col=i-2*Nxw*Nyw-2*Nxt*Nyt-Nyr*(index1col-Nxw-Nxt-1);      % goes from 1 to Nyr
        end
        
        % collocation points for wing, tail and rudder
        xcol1=xcol(index1col,index2col);
        ycol1=ycol(index1col,index2col);
        zcol1=zcol(index1col,index2col);
        
        % correcting the position of the collocation points on rudder
        
        if i>2*Nxw*Nyw+2*Nxt*Nyt
            M=[1,0,0;0,cosd(90),-sind(90);0,sind(90),cosd(90)]*[xcol1;ycol1;zcol1];
            xcol1=M(1);ycol1=M(2);zcol1=M(3);
        end
        
        %% wake of physical surfaces
        % wing wake
        Nww=tmax;           % number of wing wake panels
        wake2vw=0;a2w=0;
        
        for j=1:2*Nww*Nyw
            index1ww=ceil(j/(2*Nyw));
            index2ww=j-(2*Nyw)*(index1ww-1);
            xw1=xww(index1ww,index2ww);      yw1=yww(index1ww,index2ww);      zw1=zww(index1ww,index2ww);
            [a1w,wake1vw]=vortexring(n(:,i),uinf*dt,dly(1),0,0,0,xcol1,ycol1,zcol1,xw1,yw1,zw1,Gww(j),0);
            wake2vw=wake2vw+wake1vw;
            a2w=a2w+a1w;
        end
        
        %% tail wake
        Nwt=floor(tmax/2);  % number of tail wake panels
        wake2vt=0;a2t=0;
        
        for j=1:2*Nwt*Nyt
            index1wt=ceil(j/(2*Nyt));
            index2wt=j-(2*Nyt)*(index1wt-1);
            xw1=xwt(index1wt,index2wt);      yw1=ywt(index1wt,index2wt);      zw1=zwt(index1wt,index2wt);
            [a1t,wake1vt]=vortexring(n(:,i),uinf*dt,dly(2),0,0,0,xcol1,ycol1,zcol1,xw1,yw1,zw1,Gwt(j),0);
            wake2vt=wake2vt+wake1vt;
            a2t=a2t+a1t;
        end
        
        %% rudder wake
        
        Nwr=floor(tmax/2); % number of rudder wake panels
        wake2vr=0;a2r=0;
        
        for j=1:Nwr*Nyr
            index1wr=ceil(j/(Nyr));
            index2wr=j-(Nyr)*(index1wr-1);
            xw1=xwr(index1wr,index2wr);      yw1=ywr(index1wr,index2wr);      zw1=zwr(index1wr,index2wr);
            [a1r,wake1vr]=vortexring(n(:,i),uinf*dt,dly(3),0,0,0,xcol1,ycol1,zcol1,xw1,yw1,zw1,Gwr(j),90);
            wake2vr=wake2vr+wake1vr;
            a2r=a2r+a1r;
        end
        
        %%
        
        waken(1,i)=wake2vw+wake2vt+wake2vr;        % total effect of the wakes on the panel i
        %% vortex induced effect
        [Vv,Vvn(i)]=vortexline(n(:,i),xcol1,ycol1,zcol1,xvor,yvor,zvor,xvor,yvor+bw/16,zvor,Gv);
        a(:,i)=a2w+a2t+Vv;
    end
    
    % RHS
    RHSn=dot(-repmat(u',1,2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),n)'-waken'-Vvn';
    % RHSn=dot(-repmat(u',1,2*Nx*Ny),n)'-waken'-reshape(Vv,2*Nx*Ny,1);
    G(:,t)=Am_wing*RHSn;    % vorticity on the wing
    
    % passing the each row of the wake to the next level
    for i=1:tmax-1
        Gww(1+2*i*Nyw:2*(i+1)*Nyw)=Gww(1+2*(i-1)*Nyw:2*i*Nyw) ;
        if i <=tmax/2-1   % size of the tail wake
            Gwt(1+2*i*Nyt:2*(i+1)*Nyt)=Gwt(1+2*(i-1)*Nyt:2*i*Nyt) ;
            Gwr(1+i*Nyr:(i+1)*Nyr)=Gwr(1+(i-1)*Nyr:i*Nyr) ;
        end
    end
    
    Gww(1:2*Nyw)= G(1+2*(Nxw-1)*Nyw:2*Nxw*Nyw,t);
    Gwt(1:2*Nyt)= G(1+2*Nxw*Nyw+2*(Nxt-1)*Nyt:2*Nxw*Nyw+2*Nxt*Nyt,t);
    Gwr(1:Nyr)= G(1+2*Nxw*Nyw+2*Nxt*Nyt+(Nxr-1)*Nyr:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,t);
    
    %[cLnewv(t),cl_section(:,t),dp(:,:,t)]=force_calc(Nxw,Nyw,a(:,1:2*Nxw*Nyw),G(1:2*Nxw*Nyw,:),Gs(1:2*Nxw*Nyw,1),t,dt,Sw,dl_x(1:2*Nyw),dly(1),uinf,aoa,Lam,dih,aoaf_Lw,aoaf_Rw);
    %[cLnewvt(t),cl_sectiont(:,t),dpt(:,:,t)]=force_calc(Nxt,Nyt,a(:,1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),G(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,:),Gs(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,1),t,dt,St,dl_x(1+2*Nyw:2*(Nyw+Nyt)),dly(2),uinf,aoa,Lam,dih,aoaf_Lt,aoaf_Rt);
    %[cLnewvr(t),cl_sectionr(:,t),dpr(:,:,t)]=force_calc(Nxr,Nyr/2,a(:,1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),G(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,:),Gs(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,1),t,dt,Sr,dl_x(1+2*Nyw+2*Nyt:2*(Nyw+Nyt)+Nyr),dly(3),uinf,0,Lam,0,0,aoar);
%force_calc(Nxw,Nyw,a(:,1:2*Nxw*Nyw),a_d(:,1:2*Nxw*Nyw),w_ind_drag(1:2*Nxw*Nyw),bw,G(1:2*Nxw*Nyw,:),G(1:2*Nxw*Nyw,1),t,dt,Sw,dl_x(1:2*Nyw),dly(1),norm(u,2),aoa,Lam,dih,aoaf_Lw,aoaf_Rw)
[cLnewv(:,t),Liftw,Dragw,dp(:,:,t),~,rollw,yaww]=force_calc(Nxw,Nyw,a(:,1:2*Nxw*Nyw),a_d(:,1:2*Nxw*Nyw),w_ind_drag(1:2*Nxw*Nyw),bw,G(1:2*Nxw*Nyw,:),Gs(1:2*Nxw*Nyw,1),t,dt,Sw,dl_x(1:2*Nyw),dly(1),norm(u,2),aoa,Lam,dih,aoaf_Lw,aoaf_Rw,0);
%       

[cLnewvt(:,t),Liftt,Dragt,dpt(:,:,t),~,rollt,yawt]=force_calc(Nxt,Nyt,a(:,1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),a_d(:,1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),w_ind_drag(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),bt,G(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,:),Gs(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,1),t,dt,St,dl_x(1+2*Nyw:2*(Nyw+Nyt)),dly(2),norm(u,2),aoa,Lam,dih,aoaf_Lt,aoaf_Rt,0);

% we can add the effct of side slip angle as aoa for rudder -- just saying
[cLnewvr(:,t),Liftr,Dragr,dpr(:,:,t),~,rollr,yawr]=force_calc(Nxr,Nyr/2,a(:,1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),a_d(:,1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),w_ind_drag(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),br,G(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,:),Gs(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,1),t,dt,Sr,dl_x(1+2*Nyw+2*Nyt:2*(Nyw+Nyt)+Nyr),dly(3),norm(u,2),0,Lam,0,0,aoar,1);

end
toc
%% figure
%plot(linspace(0,vtmax*dt*uinf/cr,vtmax),cLnewv);xlabel('time'); ylabel('C_L');grid minor;hold on;
%figure
%plot(y,cl_section(:,10));
%axis([0 9 0 0.55]);
% surf(x,y,z,[G(1:2*Ny,34)';G(2*Ny+1:4*Ny,34)';G(4*Ny+1:6*Ny,34)';G(6*Ny+1:8*Ny,34)'])
figure
surf(repmat(linspace(0,vtmax*dt*uinf/crw,vtmax),2*Ny,1),repmat(y(1,1:2*Ny),vtmax,1)',cLnewv)
xlabel('time');ylabel('span');zlabel('c_l');
view(-150,15);

%% plotting real wing -- converting the middle points data to edges.
% zr=bw/16+yr1(:,Nyr+1:end);xr=bw/2+xr1(:,Nyr+1:end);yr=zr1(:,Nyr+1:end);

% just for plot
t=1
% sp(dps,xw,yw,zw,bw,bt,Nxw,Nyw,trw,Sw,Lam,dih,aoa,aoaf_Lw,aoaf_Rw,t,.0001,0,0);

sp(dp,xw,yw,zw,bw,bt,Nxw,Nyw,trw,Sw,Lam,dih,aoa,aoaf_Lw,aoaf_Rw,t,.8,0,0);

sp(dpt,xt,yt,zt,bt,bw,Nxt,Nyt,trt,St,Lam,dih,aoa,aoaf_Lt,aoaf_Rt,t,.8,1,0);
sp(dpr,xr,yr,zr,br,bw,Nxr,Nyr,trr,Sr,Lam,0,0,0,aoar,tmax,.8,0,1);