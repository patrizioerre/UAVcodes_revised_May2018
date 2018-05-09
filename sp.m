function sp(dp,x,y,z,b,bw,Nx,Ny,tr,S,Lam,dih,aoa,aoaf_L,aoaf_R,t,dpmax,tail,rudder)

vtmax=size(dp,3);


bn=b+0.5*b/Ny/2;
ARn=bn^2/(S*(Nx+1)/Nx*(Ny+1/4)/Ny);
trn=tr;

% you may want to change this wtih the new geometry code

[xi,yi,zi,~,~,~,~,dl_xi,~,~,~,~]=geometry(ARn,bn,trn,Nx+1,2*Ny+1/2,Lam,dih,aoa,0,0);


if tail
    xi=bw/2+xi;
    yi=yi;
    zi=bw/16+zi ;
end

% if rudder
%
%     x = x;
%     z = y + bw/16;
%     y = z;
%
%
%     xi = bw/2 + xi;
%     zi = yi + bw/16;
%     yi = zi ;
%
% end


% xi(Nx+1,4*Ny-2)=NaN;xi(Nx+1,4)=NaN;
% % right flap geometry
% xifr(1,1)=xi(Nx,3);xifr(1,2)=xi(Nx,5);
% xifr(2,1)=xi(Nx,3)+dl_xi(3)*cos(aoa+aoaf_R);
% xifr(2,2)=xi(Nx,5)+dl_xi(5)*cos(aoa+aoaf_R);
% yifr(1,1)=yi(Nx,3);yifr(1,2)=yi(Nx,5);yifr(2,1)=yi(Nx,3);yifr(2,2)=yi(Nx,5);
% zifr(1,1)=zi(Nx,3);zifr(1,2)=zi(Nx,5);
% zifr(2,1)=zi(Nx,3)-dl_xi(3)*sin(aoaf_R+aoa)*cos(dih);
% zifr(2,2)=zi(Nx,5)-dl_xi(5)*sin(aoaf_R+aoa)*cos(dih);
%
% % left flap geometry
% xifl(1,1)=xi(Nx,4*Ny-3);xifl(1,2)=xi(Nx,4*Ny-1);
% xifl(2,1)=xi(Nx,4*Ny-3)+dl_xi(4*Ny-3)*cos(aoa+aoaf_L);
% xifl(2,2)=xi(Nx,4*Ny-1)+dl_xi(4*Ny-1)*cos(aoa+aoaf_L);
% yifl(1,1)=yi(Nx,4*Ny-3);yifl(1,2)=yi(Nx,4*Ny-1);yifl(2,1)=yi(Nx,4*Ny-3);yifl(2,2)=yi(Nx,4*Ny-1);
% zifl(1,1)=zi(Nx,4*Ny-3);zifl(1,2)=zi(Nx,4*Ny-1);
% zifl(2,1)=zi(Nx,4*Ny-3)-dl_xi(4*Ny-3)*sin(aoaf_L+aoa)*cos(dih);
% zifl(2,2)=zi(Nx,4*Ny-1)-dl_xi(4*Ny-1)*sin(aoaf_L+aoa)*cos(dih);

%% for Fontana flap
Nf=round(0.85*Ny);  % for Fontana in each wing, the flap is atended as 80% started from tip.

Nf=Ny;
for i=1:2*Nf
    xi(Nx+1,i)=NaN;xi(Nx+1,4*Ny-i+2)=NaN;
end

for i=1:2*Nf+1
    
    % right flap geometry looking from aft to front (+y)
    xifr(1,i)=xi(Nx,4*Ny-i+2);
    xifr(2,i)=xi(Nx,4*Ny-i+2)+dl_xi(4*Ny-i+2)*cos(aoa+aoaf_R);
    yifr(1,i)=yi(Nx,4*Ny-i+2);
    yifr(2,i)=yi(Nx,4*Ny-i+2);
    zifr(1,i)=zi(Nx,4*Ny-i+2);
    zifr(2,i)=zi(Nx,4*Ny-i+2)-dl_xi(4*Ny-i+2)*sin(aoaf_R+aoa)*cos(dih);
    
    % left flap geometry looking from aft to front (-y)
    xifl(1,i)=xi(Nx,i);
    xifl(2,i)=xi(Nx,i)+dl_xi(i)*cos(aoa+aoaf_L);
    yifl(1,i)=yi(Nx,i);
    yifl(2,i)=yi(Nx,i);
    zifl(1,i)=zi(Nx,i);
    zifl(2,i)=zi(Nx,i)-dl_xi(i)*sin(aoaf_L+aoa)*cos(dih);
       
end

%%
for i=1:Nx
    if 1-rudder
        xv(1+(i-1)*2*Ny:2*Ny*i)=x(i,1:2*Ny);
        yv(1+(i-1)*2*Ny:2*Ny*i)=y(i,1:2*Ny);
        zv(1+(i-1)*2*Ny:2*Ny*i)=z(i,1:2*Ny);
        for j=1:vtmax
            dpv(1+(i-1)*2*Ny:2*Ny*i,j)=dp(i,1:2*Ny,j);
        end
    else % the x,y, are imported for rudder without further changes for oreintation
        xv(1+(i-1)*Ny:Ny*i)=x(i,1:Ny);
        yv(1+(i-1)*Ny:Ny*i)=y(i,1:Ny);
        zv(1+(i-1)*Ny:Ny*i)=z(i,1:Ny);
        for j=1:vtmax
            %%%%check here for rudder
            dpv(1+(i-1)*Ny:Ny*i,j)=dp(i,1:Ny,j);
        end
    end
end

if rudder 
    zi=yi + bw/16;
    yi=zi;
    xi=xi + bw/2
    
    % xr already had the bw/2 addition
    zv=yv + bw/16;
    yv=zv;
    xv=xv;  
end

% the first is the interpolatoin (linear) and the second is extraploation (nearest)
aa=scatteredInterpolant(xv',yv',zv',dpv(:,t),'linear','linear');

dpl=aa(xi(:,1:2*Ny+1),yi(:,1:2*Ny+1),zi(:,1:2*Ny+1));
dpr=aa(fliplr(xi(:,2*Ny+1:4*Ny+1)), fliplr(yi(:,2*Ny+1:4*Ny+1)), fliplr(zi(:,2*Ny+1:4*Ny+1)));

if rudder-1
    % left piece
    surf(xi(:,1:2*Ny+1),yi(:,1:2*Ny+1),zi(:,1:2*Ny+1),dpl);
    hold on;axis('equal');
    surf(xifl,yifl,zifl,aa(xifl,yifl,zifl))
    % right piece
    surf( fliplr(xi(:,2*Ny+1:4*Ny+1)), fliplr(yi(:,2*Ny+1:4*Ny+1)),...
        fliplr(zi(:,2*Ny+1:4*Ny+1)),dpr);
    
    surf(xifr,yifr,zifr,aa(xifr,yifr,zifr));
      
elseif rudder
    
    surf( fliplr(xi(:,2*Ny+1:4*Ny+1)), fliplr(yi(:,2*Ny+1:4*Ny+1)),...
        fliplr(zi(:,2*Ny+1:4*Ny+1)),dpr);
    hold on; surf(xifr,yifr,zifr,aa(xifr,yifr,zifr));
end

%surf(xifr,yifr,zifr,repmat(dp(Nx,2,10),2,2));surf(xifl,yifl,zifl,repmat(dp(Nx,2*Ny-1,10),2,2));

set(gca, 'CLim', [min(min(min((dp)))), dpmax]);

colorbar;

%% movie

% figure
% for i=1:vtmax
%   aa=scatteredInterpolant(xv',yv',zv',dpv(:,i));
% surf(xi,yi,zi,aa(xi,yi,zi));hold on;axis('equal');
% surf(xifr,yifr,zifr,aa(xifr,yifr,zifr));surf(xifl,yifl,zifl,aa(xifr,yifr,zifr));
%
% %     surf(x,y,z,dp(:,:,i));drawnow;
%    set(gca, 'CLim', [0, .5]);colorbar;
%      axis('equal');
%     Mv(i)=getframe;
% end
% v = VideoWriter('movie.avi');
% open(v)
% writeVideo(v,Mv)
% close(v)
