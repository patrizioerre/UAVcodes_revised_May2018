% Simplified unsteady 3D UAV Vortex Panel Method by B. Davoudi 5/6/2018
% this code is for full UAV, I will cooment unneccessary lines, generally
% "w" refers to wing, "t" refers to tail, "r" refres to rudder

tic
clc; clear all;
Nxw=4;Nxt=0;               % chordwise panel per hald wing
Nyw=9;Nyt=0;               % spanwise panel per hald wing
Nxr=0;Nyr=0;
ARw=176.5^2/7150;             % Aspect ratio
ARt=0;
ARr=0;
bw=1;bt=0;         % wing span
br=bt;
trw=1.8;trt=0;trr=0;        % taper ratio
Lam=degtorad(0);             % sweep angle, backward swept, positive
dih=degtorad(0);             % dihedral angle defined at the c/4
aoa=degtorad(0);             % angle of attack
uinf=1;                      % incidence velocity
u=uinf*[1 0 0];              % incidence velocity vector

aoaf_Lw=degtorad(0);aoaf_Lt=degtorad(0);  % left (-y) and right (+y) is looking from aft
aoaf_Rw=degtorad(0);aoaf_Rt=degtorad(0);
aoar=degtorad(0);

%% Initial geometry of the UAV

% wing
[xw,yw,zw,xcolw,ycolw,zcolw,nw,dl_xw,dlyw,Sw,alphaw,crw]=geometry(ARw,bw,trw,Nxw,Nyw,Lam,dih,aoa,aoaf_Lw,aoaf_Rw);
% tail
% I set aoa equal to zero for tail
% [xt,yt,zt,xcolt,ycolt,zcolt,nt,dl_xt,dlyt,St,alphat,~]=geometry(ARt,bt,trt,Nxt,Nyt,Lam,dih,-2 * pi/180,aoaf_Lt,aoaf_Rt);

% moving the tail to the aft
%xt=bw/2+xt;xcolt=bw/2+xcolt;
%zt=bw/16+zt;zcolt=bw/16+zcolt;

% rudder
%[xr1,yr1,zr1,xcolr1,ycolr1,zcolr1,nr1,dl_xr1,dlyr,Sr,alphar1,~]=geometry(ARr,br,trr,Nxr,Nyr,Lam,0,0,aoar,aoar);

% choosing the right wing (pilot right)
% choosing the positive y part of the wing - should not add the zoffset
% here!!
%zr=zr1(:,Nyr+1:end);xr=bw/2+xr1(:,Nyr+1:end);yr=yr1(:,Nyr+1:end);
%zcolr=zcolr1(:,Nyr+1:end);xcolr=bw/2+xcolr1(:,Nyr+1:end);ycolr=ycolr1(:,Nyr+1:end);
%alphar=alphar1(:,Nyr+1:end);dl_xr=dl_xr1(Nyr+1:end);

% modifying normal vector for rudder
%nr2=[nr1(:,Nyr+1:2*Nyr),nr1(:,3*Nyr+1:end)];
%nr=zeros(3,Nyr*Nxr);
%theta=90;

% for i=1:Nxr*Nyr
%     nr(:,i) =  [1,0,0;0,cosd(theta),-sind(theta);0,sind(theta),cosd(theta)]*nr2(:,i);
% end

% constructing x,y,x that incorporates wing, tail and rudder locations

Ny=Nyw;

% x=[xw;[xt,zeros(Nxt,2*Ny-2*Nyt)];[xr,zeros(Nxr,2*Ny-Nyr)]];y=[yw;[yt,zeros(Nxt,2*Ny-2*Nyt)];[yr,zeros(Nxr,2*Ny-Nyr)]];z=[zw;[zt,zeros(Nxt,2*Ny-2*Nyt)];[zr,zeros(Nxr,2*Ny-Nyr)]];
% xcol=[xcolw;[xcolt,zeros(Nxt,2*Ny-2*Nyt)];[xcolr,zeros(Nxr,2*Ny-Nyr)]];ycol=[ycolw;[ycolt,zeros(Nxt,2*Ny-2*Nyt)];[ycolr,zeros(Nxr,2*Ny-Nyr)]];
% zcol=[zcolw;[zcolt,zeros(Nxt,2*Ny-2*Nyt)];[zcolr,zeros(Nxr,2*Ny-Nyr)]];n=[nw,nt,nr];dl_x=[dl_xw,dl_xt,dl_xr];dly=[dlyw,dlyt,dlyr];
% alpha=[alphaw;[alphat,zeros(Nxt,2*Ny-2*Nyt)];[alphar,zeros(Nxr,2*Ny-Nyr)]];

x=xw;y=yw;z=zw;xcol=xcolw;ycol=ycolw;zcol=zcolw;alpha=alphaw;n=nw;dl_x=dl_xw;dly=dlyw;

%% Steady Solution for the initial solution

% steady state initial solution
[Gs,Am_wing,A,a,a_d,w_ind_drag]=fast_steady_tail_rudder(x,y,z,xcol,ycol,zcol,n,dl_x,dly,Nxw,Nxt,Nxr,Nyw,Nyt,Nyr,u,alpha,Lam,dih,bw);

figure
plot(Gs); hold on;

figure
t=1;dt=1;
[cL,Liftw,Dragw,dps,ds,rollw,yaww]=force_calc(Nxw,Nyw,a(:,1:2*Nxw*Nyw),a_d(:,1:2*Nxw*Nyw),w_ind_drag(1:2*Nxw*Nyw),bw,Gs(1:2*Nxw*Nyw,:),Gs(1:2*Nxw*Nyw,1),t,dt,Sw,dl_x(1:2*Nyw),dly(1),norm(u,2),aoa,Lam,dih,aoaf_Lw,aoaf_Rw,0);
plot(yw(1,:),cL);hold on;xlabel('span');ylabel('C_l');grid minor;

%% Wake extention from TE of physical surfaces

%time step
dt=0.5*min(dl_xw)*Nxw/uinf * 0.5;   %%%%%%%%%%%%%%%%%%%%%% 1/4

% size of the wake panels
dlw=dt*uinf;

% maximum simulation time
tmax=floor(2*bw/(uinf*dt)) ; % arbitary (twice of the wing span)

wake_length=tmax; % this is long enough, wake after this length will be ignored

for i=1:wake_length
    xww(i,:)=xw(Nxw,:)+dl_x(1:2*Nyw).*cos(alphaw(end,:))+uinf*(i-1)*dt;
    yww(i,:)=yw(Nxw,:);
    zww(i,:)=zw(Nxw,:)-dl_x(1:2*Nyw).*sin(alphaw(end,:))*cos(dih);
    % if i<=tmax/2
    % size of the tail wake
    %     xwt(i,:)=xt(Nxt,:)+dl_x(2*Nyw+1:2*(Nyt+Nyw)).*cos(alphat(end,:))+uinf*(i-1)*dt;
    %     ywt(i,:)=yt(Nxt,:);
    %     zwt(i,:)=zt(Nxt,:)-dl_x(2*Nyw+1:2*(Nyt+Nyw)).*sin(alphat(end,:))*cos(dih);
    % size of rudder wake
    %     xwr(i,:)=xr(Nxr,:)+dl_x(2*Nyw+2*Nyt+1:2*(Nyt+Nyw)+Nyr).*cos(alphar(end,:))+uinf*(i-1)*dt;
    %     ywr(i,:)=yr(Nxr,:);
    %     zwr(i,:)=zr(Nxr,:)-dl_x(2*Nyw+2*Nyt+1:2*(Nyt+Nyw)+Nyr).*sin(alphar(end,:))*cos(0);
    % end
end

% the physical surfaces and associated wakes
figure;hold on;
%surf(xt,yt,zt);
surf(xw,yw,zw);
%surf(xr,yr,zr);
%surf(xwr,ywr,zwr);
surf(xww,yww,zww);
%surf(xwt,ywt,zwt);
axis('equal');

%% unsteady solution, for instance: response to a gust

% the initial wake vortex strength obtained by steady solution -- starting
% the second wake row
Gww=repmat(Gs(1+2*(Nxw-1)*Nyw:2*Nxw*Nyw),wake_length,1);
%Gwt=repmat(Gs(1+2*Nxw*Nyw+2*(Nxt-1)*Nyt:2*Nxt*Nyt+2*Nxw*Nyw),floor(tmax/2),1);
%Gwr=repmat(Gs(1+2*Nxw*Nyw+2*Nxt*Nyt+(Nxr-1)*Nyr:2*Nxt*Nyt+2*Nxw*Nyw+Nxr*Nyr),floor(tmax/2),1);

% gust info
% vtmax=tmax;
% xvor=-bw/2;
% yvor=bw/2-bw/32;
% zvor=-bw/16;
% rc=0.162*bw/8;
% Gv=1.122*uinf*b/8;
% Gv=0;

% initial values:
dih=degtorad(zeros(Nxw,2*Nyw));
Lam=degtorad(ones(Nxw,2*Nyw));
alpha=degtorad(7*ones(Nxw,2*Nyw));

% for sudden angle of attack increase:
% t=1 steady, t=2 suddenly an angle off attack is set, and wing moves u*dt
% (one row of the wake), which is taken care of using A,_wing below, t=3,
% there are two panels in the wake with non-zero wake values, the closer
% one is handles with Am_wing and the second panel is handles using the
% "wake of the physical surfaces"....and do on and so forth
for t=1:tmax
    t
    %    xvor=xvor+uinf*dt;
    
    %% defining the new geometry and new wake
    
    if t>1
        
        % here you can use your new inputs for angles and positions to have
        % your new geometry: Lam,dih,aoa? you have them from you structure
        % code -- this is the old code, I can only send in scalar Lam,dih,aoa
        
        aoa1=degtorad(7);
        dih1=degtorad(0);
        Lam1=degtorad(0);
        
        [x,y,z,xcol,ycol,zcol,n,dl_x,dly,S,alpha,cr]=geometry(ARw,bw,trw,Nxw,Nyw,Lam1,dih1,aoa1,aoaf_Lw,aoaf_Rw);
        
        % here we need to find Am_wing matrix with new geometry, you should
        % send in the new geometry and angles: Lam,dih,aoa which basically
        % the same as aoa1, dih1 and Lam1 but matrix form
        
        % for example:
        dih=degtorad(zeros(Nxw,2*Nyw));
        Lam=degtorad(zeros(Nxw,2*Nyw));
        alpha=degtorad(7*ones(Nxw,2*Nyw));
        
        % Am_wing has the effect of the first wake row already taken into
        % consideration
        [G22,Am_wing,~,~,~,~]=fast_steady_tail_rudder_patrizio(x,y,z,xcol,ycol,zcol,n,dl_x,dly,dlw,Nxw,Nxt,Nxr,Nyw,Nyt,Nyr,u,alpha,Lam,dih,bw);
        
        % shifting the wake one step downstream
        
        for q1=wake_length-1:-1:1
                
                xww(q1+1,:)=xww(q1,:)+uinf*dt;
                yww(q1+1,:)=yww(q1,:);
                zww(q1+1,:)=zww(q1,:);
        end
        % this is the fist row of panels of the wake which is newly obtained shed from Trailing Edge -- you
        % should update alpha and dih -- this has to come after the
        % shifting ^^
        
             xww(1,:)=x(Nxw,:)+dl_x.*cos(alpha(Nxw,:));
             yww(1,:)=y(Nxw,:);
             zww(1,:)=z(Nxw,:)-dl_x.*sin(alpha(Nxw,:)).*cos(dih(Nxw,:));
       

    end
    
    
    %% solver
    
    for i=1:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr  % total of the panels in the wing and wake
        
        % once the panels on the wing are considered, tail panels are the next
        % starting from 2*Nxw*Nyw+1 to 2*Nxw*Nyw+2*Nxt*Nyt
        %% physical surfaces (wing, tail and rudder)
        % collocation points indices
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
        Nww=wake_length-1;           % number of wing wake panels excluding the first panel
        wake2vw=0;a2w=0;
        %%%% add swipe and dihedra to wake!!!! ---done
        % note that the first wake row has already been considered in the
        % Am_wing, j=1 belongs to the second row wake
        for j=1:2*Nww*Nyw
            % j=1:2*Nww*Nyw
            % ...+1 exist because we start the considering the second
            % row -- first row already was considered. similarly added a -1
            % to index2ww keep things right
            index1ww=ceil(j/(2*Nyw))          +1;
            index2ww=j-(2*Nyw)*(index1ww-1    -1);
            xw1=xww(index1ww,index2ww);      yw1=yww(index1ww,index2ww);      zw1=zww(index1ww,index2ww);
            [a1w,wake1vw]=vortexring(n(:,i),uinf*dt,dly(1),0,Lam(Nxw,index2ww),dih(Nxw,index2ww),xcol1,ycol1,zcol1,xw1,yw1,zw1,Gww(j),0);
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
            [a1t,wake1vt]=vortexring(n(:,i),uinf*dt,dly(2),0,Lam(Nxt+Nxw,index2wt),dih(Nxt+Nxw,index2wt),xcol1,ycol1,zcol1,xw1,yw1,zw1,Gwt(j),0);
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
            [a1r,wake1vr]=vortexring(n(:,i),uinf*dt,dly(3),0,Lam(Nxt+Nxw+Nxr,index2wr),0,xcol1,ycol1,zcol1,xw1,yw1,zw1,Gwr(j),90);
            wake2vr=wake2vr+wake1vr;
            a2r=a2r+a1r;
        end
        
        %%        
        waken(1,i)=wake2vw+wake2vt+wake2vr;        % total effect of the wakes on the panel i
        %% vortex induced effect
        %         [Vv,Vvn(i)]=vortexline(n(:,i),xcol1,ycol1,zcol1,xvor,yvor,zvor,xvor,yvor+bw/16,zvor,Gv);
        %         a(:,i)=a2w+a2t+Vv;
    end
    
    % RHS
    % RHSn=dot(-repmat(u',1,2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),n)'-waken'-Vvn';
    
    % waken has the effect of the wake of wing on the wing
    
    RHSn=dot(-repmat(u',1,2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),n)'-waken';
    
    % RHSn=dot(-repmat(u',1,2*Nx*Ny),n)'-waken'-reshape(Vv,2*Nx*Ny,1);
    G(:,t)=Am_wing*RHSn;    % vorticity on the wing
    
    % passing the each row of the wake to the next level
    for i=tmax-1:-1:1
        Gww(1+2*i*Nyw:2*(i+1)*Nyw)=Gww(1+2*(i-1)*Nyw:2*i*Nyw) ;
        if i <=tmax/2-1   % size of the tail wake
            %    Gwt(1+2*i*Nyt:2*(i+1)*Nyt)=Gwt(1+2*(i-1)*Nyt:2*i*Nyt) ;
            %    Gwr(1+i*Nyr:(i+1)*Nyr)=Gwr(1+(i-1)*Nyr:i*Nyr) ;
        end
    end
    

    % passing the TE vortex (which is the same as first wake row) to the second row of the wake
    Gww(1:2*Nyw)= G(1+2*(Nxw-1)*Nyw:2*Nxw*Nyw,t);

    %    Gwt(1:2*Nyt)= G(1+2*Nxw*Nyw+2*(Nxt-1)*Nyt:2*Nxw*Nyw+2*Nxt*Nyt,t);
    %    Gwr(1:Nyr)= G(1+2*Nxw*Nyw+2*Nxt*Nyt+(Nxr-1)*Nyr:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,t);
    
    %[cLnewv(t),cl_section(:,t),dp(:,:,t)]=force_calc(Nxw,Nyw,a(:,1:2*Nxw*Nyw),G(1:2*Nxw*Nyw,:),Gs(1:2*Nxw*Nyw,1),t,dt,Sw,dl_x(1:2*Nyw),dly(1),uinf,aoa,Lam,dih,aoaf_Lw,aoaf_Rw);
    %[cLnewvt(t),cl_sectiont(:,t),dpt(:,:,t)]=force_calc(Nxt,Nyt,a(:,1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),G(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,:),Gs(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,1),t,dt,St,dl_x(1+2*Nyw:2*(Nyw+Nyt)),dly(2),uinf,aoa,Lam,dih,aoaf_Lt,aoaf_Rt);
    %[cLnewvr(t),cl_sectionr(:,t),dpr(:,:,t)]=force_calc(Nxr,Nyr/2,a(:,1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),G(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,:),Gs(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,1),t,dt,Sr,dl_x(1+2*Nyw+2*Nyt:2*(Nyw+Nyt)+Nyr),dly(3),uinf,0,Lam,0,0,aoar);
    %force_calc(Nxw,Nyw,a(:,1:2*Nxw*Nyw),a_d(:,1:2*Nxw*Nyw),w_ind_drag(1:2*Nxw*Nyw),bw,G(1:2*Nxw*Nyw,:),G(1:2*Nxw*Nyw,1),t,dt,Sw,dl_x(1:2*Nyw),dly(1),norm(u,2),aoa,Lam,dih,aoaf_Lw,aoaf_Rw)
    
    [cLnewv(:,t),Liftw(t),Dragw,dp(:,:,t),~,rollw,yaww]=force_calc_patrizio(Nxw,Nyw,a(:,1:2*Nxw*Nyw),a_d(:,1:2*Nxw*Nyw),w_ind_drag(1:2*Nxw*Nyw),bw,G(1:2*Nxw*Nyw,:),Gs(1:2*Nxw*Nyw),t,dt,Sw,dl_x(1:2*Nyw),dly(1),norm(u,2),alpha,Lam,dih,aoaf_Lw,aoaf_Rw,0);
    %
    
    %[cLnewvt(:,t),Liftt,Dragt,dpt(:,:,t),~,rollt,yawt]=force_calc(Nxt,Nyt,a(:,1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),a_d(:,1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),w_ind_drag(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt),bt,G(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,:),Gs(1+2*Nxw*Nyw:2*Nxw*Nyw+2*Nxt*Nyt,1),t,dt,St,dl_x(1+2*Nyw:2*(Nyw+Nyt)),dly(2),norm(u,2),aoa,Lam,dih,aoaf_Lt,aoaf_Rt,0);
    
    % we can add the effct of side slip angle as aoa for rudder -- just saying
    % [cLnewvr(:,t),Liftr,Dragr,dpr(:,:,t),~,rollr,yawr]=force_calc(Nxr,Nyr/2,a(:,1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),a_d(:,1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),w_ind_drag(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr),br,G(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,:),Gs(1+2*Nxw*Nyw+2*Nxt*Nyt:2*Nxw*Nyw+2*Nxt*Nyt+Nxr*Nyr,1),t,dt,Sr,dl_x(1+2*Nyw+2*Nyt:2*(Nyw+Nyt)+Nyr),dly(3),norm(u,2),0,Lam,0,0,aoar,1);
    
end
toc
%% figure
%plot(linspace(0,vtmax*dt*uinf/cr,vtmax),cLnewv);xlabel('time'); ylabel('C_L');grid minor;hold on;
%figure
%plot(y,cl_section(:,10));
%axis([0 9 0 0.55]);
% surf(x,y,z,[G(1:2*Ny,34)';G(2*Ny+1:4*Ny,34)';G(4*Ny+1:6*Ny,34)';G(6*Ny+1:8*Ny,34)'])
figure

plot(linspace(0,tmax*dt*uinf/crw,tmax),Liftw)
xlabel('time');ylabel('lift'),ylim([0,0.07]),xlim([0,tmax*dt*uinf/crw])

figure
surf(repmat(linspace(0,tmax*dt*uinf/crw,tmax),2*Ny,1),repmat(y(1,1:2*Ny),tmax,1)',cLnewv)
xlabel('time');ylabel('span');zlabel('c_l');
view(-150,15);

%% plotting real wing -- converting the middle points data to edges.
% zr=bw/16+yr1(:,Nyr+1:end);xr=bw/2+xr1(:,Nyr+1:end);yr=zr1(:,Nyr+1:end);

% just for plot, t is the time you want to plot the contours
t=5;

sp(dp,xw,yw,zw,bw,bt,Nxw,Nyw,trw,Sw,Lam,dih,aoa,aoaf_Lw,aoaf_Rw,t,.8,0,0);

% sp(dps,xw,yw,zw,bw,bt,Nxw,Nyw,trw,Sw,Lam,dih,aoa,aoaf_Lw,aoaf_Rw,t,.0001,0,0);
%sp(dpt,xt,yt,zt,bt,bw,Nxt,Nyt,trt,St,Lam,dih,aoa,aoaf_Lt,aoaf_Rt,t,.8,1,0);
%sp(dpr,xr,yr,zr,br,bw,Nxr,Nyr,trr,Sr,Lam,0,0,0,aoar,tmax,.8,0,1);