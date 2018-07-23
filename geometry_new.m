function [x,y,z,xcol,ycol,zcol,n,dl_x,dly,S,alpha,cr,dih,zdot,f_points]=geometry(AR,b,tr,Nx,Ny,Lam_pre,dih_pre,aoa_pre,aoaf_L,aoaf_R,length_coor,k,phi,w,wdot)

S=b^2/AR;                    % wing surface
wl=b/cos(Lam_pre)/cos(dih_pre);      % span length normal to the unswept wing(length of two wings) %%%%%%%%%%%%%%%
cr=2*S/wl*(1/(1+1/tr));      % chord length at the root
uinf=1;                      % incidence velocity
u=uinf*[1 0 0];              % incidence velocity vector
hh=1/(tr-1)*wl*0.5;          % hh+b = triangle height formed by a half wing extented from the tip
dly=0.5*wl/Ny;               % pannel length span wise


% location of nodes at the quarter chord -y to y
% note leng starts from root to tip!!
if rem(2*Ny,2)>0
    leng=linspace(dly,0.5*wl-0.5*dly,Ny-0.5);
else
    leng=linspace(0.5*dly,0.5*wl-0.5*dly,Ny);
end

% chord lengths arranged from -y to y
if tr~=1
    %  c=[fliplr(cr*((hh+b/2)-linspace(0.5*dly,0.5*b-0.5*dly,Ny))/(hh+b/2)*cos(aoa)) cr*((hh+b/2)-linspace(0.5*dly,0.5*b-0.5*dly,Ny))/(hh+b/2)*cos(aoa)];
    if rem(2*Ny,2)>0
        c=[fliplr(cr*((hh+b/2)-leng)/(hh+b/2)*cos(aoa_pre)) cr*cos(aoa_pre) cr*((hh+b/2)-leng)/(hh+b/2)*cos(aoa_pre)];
    else
        c=[fliplr(cr*((hh+b/2)-leng)/(hh+b/2)*cos(aoa_pre)) cr*((hh+b/2)-leng)/(hh+b/2)*cos(aoa_pre)];
    end
else
    % airfoil at the root
    c=repmat(cr,1,2*Ny);
end


if rem(2*Ny,2)>0
    leng2=[fliplr(-leng) 0 leng];
    leng3=[fliplr(leng) 0 leng];
else
    leng2=[fliplr(-leng) leng];
    leng3=[fliplr(leng) leng];
end

dl_x=c/Nx;                                        % pannel length stream wise
dlx=min(dl_x);                                    % used for delt t

% 
% alpha=repmat(aoa,Nx,2*Ny);

x=[];
y=[];
% locations of each pannel quarter chord on the wing
for i=1:Nx
    y(i,:)=leng2*cos(Lam_pre)*cos(dih_pre);
    % z(i,:)=leng3*sin(dih)*cos(aoa)-dl_x*(i-round(Nx/4+1))*sin(aoa)*cos(dih)
    % x(i,:)=leng3*sin(Lam)+dl_x*(i-round(Nx/4+1))*cos(aoa);
    x(i,:)=leng3*sin(Lam_pre)-(c-dl_x)/2 *cos(aoa_pre) + dl_x*(i-1)*cos(aoa_pre);
    z_pre(i,:)=leng3*sin(dih_pre)*cos(aoa_pre) + (c-dl_x)/2* sin(aoa_pre)*cos(dih_pre) - dl_x*(i-1)*sin(aoa_pre)*cos(dih_pre);
end





%Rosatelli's:  you must add here collocation point for the wake 
% for i=1:Nxt
% y(i,:)=leng2t*cos(Lam)*cos(dih);
    % z(i,:)=leng3*sin(dih)*cos(aoa)-dl_x*(i-round(Nx/4+1))*sin(aoa)*cos(dih)
    % x(i,:)=leng3*sin(Lam)+dl_x*(i-round(Nx/4+1))*cos(aoa);
%     x(i,:)=leng3t*sin(Lam)-(c-dl_x)/2 *cos(aoa) + dl_x*(i-1)*cos(aoa);
%     z(i,:)=leng3t*sin(dih)*cos(aoa) + (c-dl_x)/2 * sin(aoa)*cos(dih) - dl_x*(i-1)*sin(aoa)*cos(dih);



%% collocation points only differs in x and z
xcol=x+0.5*repmat(dl_x,Nx,1)*cos(aoa_pre);
zcol_pre=z_pre-0.5*repmat(dl_x,Nx,1)*sin(aoa_pre)*cos(dih_pre);
ycol=y;

if exist('w','var')==0
           % length_coor=100;
            w=zeros(length_coor);
         else w=w;
end
        
if exist('wdot','var')==0
           % length_coor=100;
            wdot=zeros(length_coor);
         else wdot=wdot;
         end
 
y_struc=linspace(0,0.912,length_coor)';
x_struc=linspace(0,0.30,length_coor)';

[aoa_p,dih_p,z_p,zcol_p,zdot]=find_deformed(w,x_struc,y_struc,Nx,Ny,wdot);


alpha=[fliplr(aoa_p) aoa_p]+aoa_pre;
dih=[fliplr(dih_p) dih_p]+dih_pre;
z=[fliplr(z_p) z_p]+z_pre;
zcol=[fliplr(zcol_p) zcol_p]+zcol_pre;
zdot=[fliplr(zdot) zdot];





%% Fontana
% for Fontana aircraft in each wing, the flap is extended 80% started from tip.

Nf=round(1*Ny);
% Nf=Ny;

% Nov 2017
% note that we put the bound vortex on the last wing panel, th eextension
% would be out of the wing if there is a flap, so we can modify the last
% wing panel (breal it at %75 which is hard to implement )or start the flap 
% panel at the end of the extended wing panel. this mean nothing to do for
% the flap paanel.. note that the deflection willbe taken care of in the
% fast steady code!!
   



    % Rosatelli's modification: being alpha the matrix of panels AOA I just
    % modified it changing the rear panel (streamwise) AOA; I suppose first 
    % columns of alpha are related to the tip of the left wing
%     k=0; %spatial frequency
%     phi=0; %spatial displ
    f=@(y) 0.02*sin(k*(2*pi)*y/0.912+deg2rad(phi));
    f_points=eval(subs(f,linspace(0,b/2,Ny)));
    for k=1:length(f_points)
        if abs(f_points(k))<0.000001
            mod_aoa_r_flap(k)=0;
        else    
            mod_aoa_r_flap(k)=-atan(f_points(k)/(0.3/Nx));
    end
    end
    
    
%   mod_aoa_r=linspace(degtorad(11)+aoa,degtorad(0)+aoa,Ny);
%   mod_aoa_r=linspace(degtorad(0)+aoa,degtorad(0)+aoa,Ny);
    
    mod_aoa_r=mod_aoa_r_flap+aoa_pre;
    mod_aoa_l=fliplr(mod_aoa_r);
    mod_aoa=[mod_aoa_l mod_aoa_r];
    alpha(Nx,1:end)=mod_aoa;
   
    

for i=1:Nf
    %% flap angles of attack
%     alpha(Nx,2*Ny-i+1)=aoaf_R+aoa;
%     alpha(Nx,i)=aoaf_L+aoa;
     
    %% flap position
% Rosatelli's modification: collocation points modified by a continuosuly
% changing aoa, not the fixed aoaf_l pr aoaf_r
%  new way -- only collocation points will be affected

     % left
     xcol(Nx,i)=x(Nx,i)+0.5*dl_x(i)*cos(mod_aoa_l(i));
     zcol(Nx,i)=z(Nx,i)-0.5*dl_x(i)*sin(mod_aoa_l(i))*cos(dih_pre);
     % right
     xcol(Nx,2*Ny-i+1)=x(Nx,2*Ny-i+1)+0.5*dl_x(2*Ny-i+1)*cos(mod_aoa_l(i));
     zcol(Nx,2*Ny-i+1)=z(Nx,2*Ny-i+1)-0.5*dl_x(2*Ny-i+1)*sin(mod_aoa_l(i))*cos(dih_pre);
 
%  old way
    %right ... not neccssary
%     x(Nx,2*Ny-i+1)=x(Nx,2*Ny-i+1)+0.25*dl_x(2*Ny-i+1)*(cos(aoaf_R+aoa)-1)*cos(aoa);
%     y(Nx,2*Ny-i+1)=y(Nx,2*Ny-i+1);
%     z(Nx,2*Ny-i+1)=z(Nx,2*Ny-i+1)-0.25*dl_x(2*Ny-i+1)*(sin(aoaf_R+aoa)-sin(aoa))*cos(dih);    
%     xcol(Nx,2*Ny-i+1)=xcol(Nx,2*Ny-i+1)+0.25*dl_x(2*Ny-i+1)*(cos(aoaf_R+aoa)-1)*cos(aoa);
%     zcol(Nx,2*Ny-i+1)=zcol(Nx,2*Ny-i+1)-0.25*dl_x(2*Ny-i+1)*(sin(aoaf_R+aoa)-sin(aoa))*cos(dih);     
    %left ... not neccssary    
%     x(Nx,i)=x(Nx,i)+0.25*dl_x(i)*(cos(aoaf_L+aoa)-1)*cos(aoa);
%     y(Nx,i)=y(Nx,i);
%     z(Nx,i)=z(Nx,i)-0.25*dl_x(i)*(sin(aoaf_L+aoa)-sin(aoa))*cos(dih);
%     
%     xcol(Nx,i)=xcol(Nx,i)+0.25*dl_x(i)*(cos(aoaf_L+aoa)-1)*cos(aoa);
%     zcol(Nx,i)=zcol(Nx,i)-0.25*dl_x(i)*(sin(aoaf_L+aoa)-sin(aoa))*cos(dih);    
end



%% used only for verification against 2D
% alpha(Nx,:)=aoaf_L+aoa;

%% normal vectors 
   % Rosatelli's modification
   
for j=1:Nx-1
    
    for i=1:Ny-1           
    % edgers is the forward side of the panel pointing to root
    edgers(:,i+(j-1)*Ny) = [-x(j,i)+x(j,i+1) -y(j,i)+y(j,i+1) -z(j,i)+z(j,i+1)];
    edgers(:,Ny*j)= [0 b/Ny -z(j,i+1)+dl_x(Ny)*(Nx-1-j)*sin(alpha(j,Ny))];
    % edgers(:,Ny)= [x(j,i+1)-dl_x(Ny)*(Nx-1-j)*cos(alpha(j,Ny)) y(j,i+1)-0 z(j,i+1)-dl_x(Ny)*(Nx-1-j)*sin(alpha(j,Ny))];
    
    
    % edgerc is the left side of the panel pointing to TE
    edgerc(:,i+(j-1)*Ny) = [-x(j,i)+x(j+1,i) -y(j,i)+y(j+1,i) -z(j,i)+z(j+1,i)]; %scrivi espresisone di segmenti chorwise
    edgerc(:,Ny*j)= [-x(j,i+1)+dl_x(Ny)*(Nx-j)*cos(alpha(j,Ny)) 0 (-z(j,i+1)+dl_x(Ny)*(Nx-2-j)*sin(alpha(j,Ny)))*2];
    
%     n_r(:,i+Nx*(j-1))=cross(edgers(:,i+Nx*(j-1)),edgerc(:,i+Nx*(j-1)))/norm(cross(edgers(:,i+Nx*(j-1)),edgerc(:,i+Nx*(j-1))));
    n_r(:,i)=cross(edgerc(:,i+(j-1)*Ny),edgers(:,i+(j-1)*Ny))/norm(cross(edgerc(:,i+(j-1)*Ny),edgers(:,i+(j-1)*Ny)));
    n_r(:,Ny)=cross(edgerc(:,Ny*j),edgers(:,Ny*j))/norm(cross(edgerc(:,Ny*j),edgers(:,Ny*j)));
    end
    n_l=fliplr([n_r(1,:);-n_r(2,:);n_r(3,:)]);
    n(:,1+2*Ny*(j-1):j*Ny*2)=[n_r n_l];
end
    

% figure(1)
% par=100
% quiver3(z(1:3,:),par*[n(2,1:20);n(2,21:40);n(2,41:60)],par*[n(1,1:20);n(1,21:40);n(1,41:60)],[n(3,1:20);n(3,21:40);n(3,41:60)])
% hold on 
% surf(z(1:3,:))
% 
% figure(2)
% [U,V,W]=surfnorm(z(1:3,:))
% quiver3(z(1:3,:),1000*U,1000*V,W)
% hold on
% surf(z(1:3,:))


% edger=[x(1,round(Ny+2))-x(1,round(Ny+1)) y(1,round(Ny+2))-y(1,round(Ny+1)) z(1,round(Ny+2))-z(1,round(Ny+1))];    % the c/4 vector line for the left half (+y)
% edgel=[x(1,1)-x(1,2) y(1,1)-y(1,2) z(1,1)-z(1,2)];                      % the c/4 vector line for the right half (-y)
% edgeb=[cr/Nx*cos(aoa_pre) 0 -cr/Nx*sin(aoa_pre)];                                 % root base section
% 
% % right and left normal vectors
% nl=cross(edgel,edgeb)/norm(cross(edgel,edgeb));
% nr=cross(edgeb,edger)/norm(cross(edgeb,edger));
% 
% %  normal to the pannels normal from -y to +y
% 
% if rem(2*Ny,2)>0
%     n=repmat([repmat(nl',1,Ny-0.5) [sin(aoa_pre); 0 ;cos(aoa_pre)] repmat(nr',1,Ny-0.5)],1,Nx);
% else
%     n=repmat([repmat(nl',1,Ny) repmat(nr',1,Ny)],1,Nx);
% end

% don't need n for visualization
% if rem(2*Ny,2)<=0
%   for i=1:2*Ny  
%     % effect of flap, note that from the TE row, any two points have the same
%     % edge as the flaps. only the base edge changes....
%     
%     % Left
%     
%     mod_dl_xr=dl_x;
%     mod_dl_xl=fliplr(mod_dl_xr);
%     mod_dl_x=[mod_dl_xl mod_dl_xr];
%     
%  edgebf(i)=[mod_dl_x(i)*cos(alpha(Nx,i)) 0 -mod_dl_x(i)*sin(alpha(i))];
%  
%  edgelf(i)=[x(Nx,i)-x(Nx,i+1) y(Nx,i)-y(Nx,i+1) z(Nx,i)-z(Nx,i+1)];
%   
% 
%  n(:,2*(Nx-1)*Ny+i)=cross(edgebf,edgelf)/norm(cross(edgebf,edgelf));
%     
% %  % Right
% % 
% %  edgebfr=[dl_x(2)*cos(alpha(Nx,2)) 0 -dl_x(2)*sin(alpha(Nx,2))];
% %       
% %  edgerf=[x(Nx,3)-x(Nx,4) y(Nx,3)-y(Nx,4) z(Nx,3)-z(Nx,4)];
% %     
% %     
% %     %% Fontana
% %     
% %     
% %         n(:,2*Nx*Ny-i+1)=cross(edgebf,edgelf)/norm(cross(edgebf,edgelf));
% %         n(:,2*(Nx-1)*Ny+i)=cross(edgerf,edgebfr)/norm(cross(edgebfr,edgerf));
%     end
%     
%     %% used only for verification against 2D
%     %     for i=2*(Nx-1)*Ny+1:2*Nx*Ny
%     %      n(:,i)=cross(edgerf,edgebfr)/norm(cross(edgebfr,edgerf));
%     %     end
% end
%if rem(2*Ny,2)<=0
  for i=1:Ny-1  
    % effect of flap, note that from the TE row, any two points have the same
    % edge as the flaps. only the base edge changes....
    
    % Right if looking plane from front
 % edgebfr is the chordwise side of a panel in the last row pointing to TE
 edgebfr(:,i)=[dl_x(i)*cos(alpha(Nx,i)) 0 -dl_x(i)*sin(alpha(Nx,i))];
 edgebfr(:,Ny)=[dl_x(i)*cos(alpha(Nx,Ny)) 0 -dl_x(i)*sin(alpha(Nx,Ny))];
  % edgerf is the rear side of a panel in the last row pointing to root
 edgerf(:,i)=[-x(Nx,i)+x(Nx,i+1) -y(Nx,i)+y(Nx,i+1) -z(Nx,i)+z(Nx,i+1)];
 edgerf(:,Ny)=[0 b/Ny/2 -z(Nx,i+1)-dl_x(i+1)*sin(aoa_pre)]; 
 n_rf(:,i)=cross(edgebfr(:,i),edgerf(:,i))/norm(cross(edgebfr(:,i),edgerf(:,i)));
 n_rf(:,Ny)=cross(edgebfr(:,Ny),edgerf(:,Ny))/norm(cross(edgebfr(:,Ny),edgerf(:,Ny)));
  end
 n_lf=fliplr([n_rf(1,:);-n_rf(2,:);n_rf(3,:)]);
 n(:,2*Ny*(Nx-1)+1:Nx*Ny*2)=[n_rf n_lf];
 %  Left if looking plane from front
  for i=Ny+1:2*Ny  
    % effect of flap, note that from the TE row, any two points have the same
    % edge as the flaps. only the base edge changes....
    % Right if looking plane from front
 

 edgebfl(:,i)=[dl_x(i)*cos(alpha(Nx,i)) 0 -dl_x(i)*sin(alpha(Nx,i))];

 edgelf(:,i)=[x(Nx,i)-x(Nx,i-1) y(Nx,i)-y(Nx,i-1) z(Nx,i)-z(Nx,i-1)];
  
 n(:,2*(Nx-1)*Ny+i)=cross(edgebfl(:,i),edgelf(:,i))/norm(cross(edgebfl(:,i),edgelf(:,i)));

  end

    

%     %% Fontana
%     
%     
%         n(:,2*Nx*Ny-i+1)=cross(edgebf,edgelf)/norm(cross(edgebf,edgelf));
%         n(:,2*(Nx-1)*Ny+i)=cross(edgerf,edgebfr)/norm(cross(edgebfr,edgerf));
 
    
    %% used only for verification against 2D
    %     for i=2*(Nx-1)*Ny+1:2*Nx*Ny
    %      n(:,i)=cross(edgerf,edgebfr)/norm(cross(edgebfr,edgerf));
    %     end
%end
end