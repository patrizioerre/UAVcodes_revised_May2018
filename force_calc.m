function [cL,Lift,Drag,dp,d,roll,yaw,L_panel]=force_calc(Nx,Ny,a,a_d,w_ind_drag,b,G,Gs,t,dt,S,dl_x,dly,uinf,aoa,Lam,dih,aoaf_L,aoaf_R,rudder) %codegen

% velocity induced by the wake on a collocation point
% for j direction (span wise) for the right half wing

% Similarly, τ i and τ j are the panel
% tangential vectors in the i and j directions (of course, these vectors are different for each
% panel and the ij subscript from τ i i j is dropped for the sake of simplicity)

dzj=-dly*(sin(dih)-sin(aoa)*sin(Lam));
dyj=-dly*cos(Lam)*cos(dih);
dxj=-dly*sin(Lam)*cos(aoa);

vj=[dxj,dyj,dzj]/norm([dxj,dyj,dzj]);

% for i direction (chord wise)
dxi=dl_x(1)*cos(aoa);
dyi=0;
dzi=-dl_x(1)*sin(aoa)*cos(dih);

vi=[dxi,dyi,dzi]/norm([dxi,dyi,dzi]);

% tangent to the pannels from -y (right) to +y (left)
% tau_i_j= d Tau_j / d Tau_i = dyi mean how the dy of the panel
% changes in the i (x) direction. note pos x is stream-wise, pos z sky and
% pos y is right hand sode of the pilot (which is called "left" here)

% my new thought
Tau_i_i=repmat(vi(1),Nx,2*Ny);
Tau_i_j=repmat(vi(2),Nx,2*Ny);
Tau_i_k=repmat(vi(3),Nx,2*Ny);
Tau_j_i=[repmat(vj(1),Nx,Ny) -repmat(vj(1),Nx,Ny)];
Tau_j_j=repmat(vj(2),Nx,2*Ny); % since tangent in y direction does not change sign from left to right
Tau_j_k=[repmat(vj(3),Nx,Ny) -repmat(vj(3),Nx,Ny)];

if rudder
    
    Tau_j_i=-repmat(vj(1),Nx,2*Ny);
    Tau_j_k=-repmat(vj(3),Nx,2*Ny);
    
end


% my old--idk if tru or not
% Tau_i_i=[repmat(vi(1),Nx,2*Ny)];
% Tau_i_j=[repmat(vi(2),Nx,Ny) -repmat(vi(2),Nx,Ny)];
% Tau_i_k=[repmat(vi(3),Nx,2*Ny)];
% Tau_j_i=[repmat(vj(1),Nx,Ny) -repmat(vj(1),Nx,Ny)];
% Tau_j_j=[repmat(vj(2),Nx,Ny) -repmat(vj(2),Nx,Ny)];  %%%% this is WRONG
% Tau_j_k=[repmat(vj(3),Nx,Ny) -repmat(vj(3),Nx,Ny)];

%% flap consideration

% how the falp looks like? already defined in the geometry!
% that defintion should be the same as this section:

% for i direction (chord wise)

dxi_L= dl_x(1)*cos(aoaf_L+aoa);
dzi_L= -dl_x(1)*sin(aoaf_L+aoa)*cos(dih);
vi_L=[dxi_L,dyi,dzi_L]/norm([dxi_L,dyi,dzi_L]);

dxi_R= dl_x(1)*cos(aoaf_R+aoa);
dzi_R= -dl_x(1)*sin(aoaf_R+aoa)*cos(dih);
vi_R=[dxi_R,dyi,dzi_R]/norm([dxi_R,dyi,dzi_R]);

% for j direction (span wise) left and right are different

dxj_L=dly*sin(Lam)*cos(aoaf_L+aoa);

dzj_L=dly*(sin(dih)-sin(aoaf_L+aoa)*sin(Lam));

vj_L=[dxj_L,dyj,dzj_L]/norm([dxj_L,dyj,dzj_L]);

dxj_R= -dly*sin(Lam)*cos(aoaf_R+aoa);

dzj_R= -dly*(sin(dih)-sin(aoaf_R+aoa)*sin(Lam));

vj_R=[dxj_R,dyj,dzj_R]/norm([dxj_R,dyj,dzj_R]);

% tangent to the flaps (Left and Right)

% for Fontana aircraft in each wing, the flap is extended 80% started from tip.
Nf=round(0.85*Ny);
alpha=repmat(aoa,Nx,2*Ny);

if rudder   
    Nf=2*Ny;  % flap is extended all over the rudder span
end

for i=1:Nf
    
    
    % flap angles of attack
    
    alpha(Nx,2*Ny-i+1)=aoaf_R+aoa;
    alpha(Nx,i)=aoaf_L+aoa;
    
    Tau_i_i(Nx,2*Ny-i+1)=vi_L(1);
    Tau_i_j(Nx,2*Ny-i+1)=vi_L(2);
    Tau_i_k(Nx,2*Ny-i+1)=vi_L(3);
    
    Tau_i_i(Nx,i)=vi_R(1);
    Tau_i_j(Nx,i)=vi_R(2);
    Tau_i_k(Nx,i)=vi_R(3);
    
    
    % this is actuallu right from pilot point of view
    
    Tau_j_i(Nx,2*Ny-i+1)=vj_L(1);
    Tau_j_j(Nx,2*Ny-i+1)=vj_L(2);
    Tau_j_k(Nx,2*Ny-i+1)=vj_L(3);
    
    % this is actually left from pilot point of view
    
    Tau_j_i(Nx,i)=vj_R(1);
    Tau_j_j(Nx,i)=vj_R(2);
    Tau_j_k(Nx,i)=vj_R(3);
    

if rudder
    % this is the right hand side of the pilot -- trun 90 around treamwise
    % become rudder thats how its defined
    
    Tau_i_i(Nx,i)=vi_L(1);
    Tau_i_j(Nx,i)=vi_L(2);
    Tau_i_k(Nx,i)=vi_L(3);
    
    Tau_j_i(Nx,i)=vj_L(1);
    Tau_j_j(Nx,i)=vj_L(2);
    Tau_j_k(Nx,i)=vj_L(3);
end
end

%% used only for verification against 2D

% Tau_i_i(Nx,:)=[vi_L(1)];
% Tau_i_j(Nx,:)=[vi_L(2)];
% Tau_i_k(Nx,:)=[vi_L(3)];

%%

G1=zeros(Nx,2*Ny);G2=zeros(Nx,2*Ny);
u_ind=zeros(Nx,2*Ny);v_ind=zeros(Nx,2*Ny);
w_ind=zeros(Nx,2*Ny);w_ind_d=zeros(Nx,2*Ny);
wake_w_ind=zeros(Nx,2*Ny);

for j=1:2*Nx*Ny
    
    index1=ceil(j/(2*Ny));
    index2=j-(2*Ny)*(index1-1);
    G1(index1,index2)=G(j,t)';
    
    if t>1
        G2(index1,index2)=G(j,t-1)';
    else
        % using the steady solution as the initial condition.
        G2(index1,index2)=Gs(j);
    end
    u_ind(index1,index2)=a(1,j);
    v_ind(index1,index2)=a(2,j);
    w_ind(index1,index2)=a(3,j);
    w_ind_d(index1,index2)=a_d(3,j);
    wake_w_ind(index1,index2)=w_ind_drag(j);
    
end

dGdx=[G1(1,:);diff(G1,1,1)]./repmat(dl_x,Nx,1);

% dGdy=[zeros(Nx,1),diff(G1(:,1:Ny),1,2)/dly,diff(G1(:,Ny+1:2*Ny),1,2)/dly,zeros(Nx,1)];
dGdy=[diff(G1(:,1:Ny),1,2)/dly,zeros(Nx,2),diff(G1(:,Ny+1:2*Ny),1,2)/dly];
dgdt=(G1-G2)/dt;

if rudder

   dGdy=[zeros(Nx,1),diff(G1(:,1:2*Ny),1,2)/dly];
    
end



if t==1
    dgdt=0;
end

% rec wing n
% dp= (u_ind +repmat(uinf,Nx,2*Ny)).*[repmat(cos(aoa),Nx,Ny) repmat(cos(aoa),Nx,Ny)] .* dGdx + ...
%     v_ind.*repmat(cos(Lam)*cos(dih),Nx,2*Ny).* dGdy +...
%     dgdt;
rho=1.15;

% pressure differential accross the wing


dp=rho* ((u_ind +repmat(uinf,Nx,2*Ny)).*Tau_i_i.* dGdx + ...
    v_ind.* Tau_i_j.* dGdx + ...
    w_ind.* Tau_i_k.* dGdx + ...
    (u_ind +repmat(uinf,Nx,2*Ny)).*Tau_j_i.* dGdy + ...
    v_ind.* Tau_j_j.* dGdy + ...
    w_ind.* Tau_j_k.* dGdy + ...
    dgdt);

% note that I had to make dgdt a matrix so that "simulink" would understand it!!!

F=sum(sum((dp.*repmat(dl_x,Nx,1)*dly)));


% area of a strip
Sc=dly*Nx*dl_x;
L_panel=(dp.*repmat(dl_x,Nx,1)*dly).*cos(aoa).*cos(dih);
L=sum(dp.*repmat(dl_x,Nx,1)*dly).*cos(aoa).*cos(dih);

%% Lift

Lift=sum(L);                      % total lift

%% Lift distribution?

cL=L./(0.5*rho*uinf^2*Sc);


%% roll and yaw calculation for rudder neglected for now

% moment calculation
% when right is larger (+y) than left (-y), the roll is positive


roll=trapz(linspace(-0.5,0,Ny)*b,-L(1:Ny))+trapz(linspace(0,0.5,Ny)*b,L(Ny+1:2*Ny));


%%
% page 428 of Katz
% note that I added a negative sign bc the induced downwash is basically
% negative (in the -z direction)

d=-rho*(wake_w_ind+w_ind_d).*dGdx.*repmat(dl_x,Nx,1)*dly + dgdt .* repmat(Sc,Nx,1) .* sin(alpha)  ;

%d=rho*(wake_w_ind+w_ind).*dGdx.*repmat(dl_x,Nx,1)*dly + dgdt .* repmat(Sc,Nx,1) .* sin(alpha)  ;

%% choose drag or drag distribution?

Drag=sum(sum(d));                     % total drag
%Drag=sum(d)./(0.5*rho*uinf^2*Sc);    % distribution


D=sum(d);

yaw=-(trapz(linspace(-0.5,0,Ny)*b,-D(1:Ny))+trapz(linspace(0,0.5,Ny)*b,D(Ny+1:2*Ny)));


end
%%%%%%%%%%%%% look up to understand how dxi... were obtained %%%%%%%%%%%
%        % right vortex
%         y1=y-0.5*dly*cos(Lam)*cos(dih);
%
%         % left vortex
%         y2=y+0.5*dly*cos(Lam)*cos(dih);
%         % 1 and 2 are the left and right neighbors (j-direction) and 3 and 4 are the ones
%         % downstream (i-direction)
%
%         if y<0
%             x1=x+0.5*dly*sin(Lam)*cos(aoa);
%             z1=z+0.5*dly*(sin(dih)-sin(aoa)*sin(Lam));
%             x2=x-0.5*dly*sin(Lam)*cos(aoa);
%             z2=z-0.5*dly*(sin(dih)-sin(aoa)*sin(Lam));
%             x3=x1+dlx*cos(aoa);
%             x4=x2+dlx*cos(aoa);
%             z3=z1-dlx*sin(aoa)*cos(dih);
%             z4=z2-dlx*sin(aoa)*cos(dih);
%
%
%         elseif y>0
%             x1=x-0.5*dly*sin(Lam)*cos(aoa);
%             z1=z-0.5*dly*(sin(dih)-sin(aoa)*sin(Lam));
%             x2=x+0.5*dly*sin(Lam)*cos(aoa);
%             z2=z+0.5*dly*(sin(dih)-sin(aoa)*sin(Lam));
%             x3=x1+dlx*cos(aoa);
%             x4=x2+dlx*cos(aoa);
%             z3=z1-dlx*sin(aoa)*cos(dih);
%             z4=z2-dlx*sin(aoa)*cos(dih);
%         end