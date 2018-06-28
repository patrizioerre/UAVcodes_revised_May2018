% finds a matrix of aoa, dihedral angles and out of plane panel center locations 
% of every panel given a matrix of out of plane displacements w, the
% vectors x and y used for structural model and the number of aerodynamic
% panels in the two directions Nx and Ny.

% x is chordwise direction, y spanwise

% Nx=4;
% Ny=20;
% length_coor=200;
% y=linspace(0,0.912,length_coor)';
% x=linspace(0,0.3,length_coor)';

% function [aoa,dih,panel_center_z]=find_deformed(w,x,y,Nx,Ny)
function [aoa,dih,z_p,zcol_p,extrd]=find_deformed(w,x,y,Nx,Ny,wdot)
%define grid points of aerodynamic panels to extrapolate displacement field 
x_ext_v1=linspace(0,0.3,Nx+1);
x_ext_v=linspace(0,0.3,Nx);
x_ext_v2=linspace(0,0.3,4*Nx+1);
y_ext_v1=linspace(0,0.912,Ny+1);
y_ext_v=linspace(0,0.912,Ny);
y_ext_v2=linspace(0,0.912,2*Ny+1);
[y_ext1,x_ext]=meshgrid(y_ext_v1,x_ext_v);
[y_ext,x_ext1]=meshgrid(y_ext_v,x_ext_v1);
[y_ext2,x_ext2]=meshgrid(y_ext_v2,x_ext_v2);
[y_extd,x_extd]=meshgrid(y_ext_v,x_ext_v);
x=x(1:size(w,1));
y=y(1:size(w,1));

%extrapolate displacement field over the grid
extry1=griddata(x,y,w,x_ext,y_ext1);
extrx1=griddata(x,y,w,x_ext1,y_ext);
extr2=griddata(x,y,w,x_ext2,y_ext2);
extrd=griddata(x,y,wdot,x_extd,y_extd);

%define distance of grid points
xs_ext=x_ext_v1(2)-x_ext_v1(1);
ys_ext=y_ext_v1(2)-y_ext_v1(1);

format long
% find dihedral angle of each panel: first, calculates difference in height
% of every grid point along each column (wing span direction), divides it
% for distance of the grid point and then find the angle
for i=1:length(x_ext_v)
extrdiffyc(i,:)=diff(extry1(i,:));
extrdiffy(i,:)=extrdiffyc(i,:)/ys_ext;

for j=1:length(extrdiffy(i,:))

 if abs(extrdiffy(i,j))<0.00001
   dih(i,j)=0;
 else dih(i,j)=asin(extrdiffy(i,j));
 end
 
end
% for j=2:length(dih_c(:,i))
% dih(j,i)=dih_c(j,i)+dih(j-1,i);
% end
end

dih_deg=dih*(180/(pi));



% find angle of attack of each panel: first, calculates difference in height
% of every grid point along each row (chordwise direction), divides it
% for distance of the grid point and then find the angle
for i=1:length(y_ext_v)
extrdiffxc(:,i)=diff(extrx1(:,i));
extrdiffx(:,i)=extrdiffxc(:,i)/xs_ext;

for j=1:length(extrdiffx(:,i))

  if abs(extrdiffx(j,i))<0.00001
    aoa(j,i)=0;
  else aoa(j,i)=-asin(extrdiffx(j,i));
    end
    
end

end

aoa_deg=aoa*(180/(pi));


for i=2:4:length(x_ext_v2)
for j=2:2:length(y_ext_v2)
    z_p(1/2+1/4*i,1/2*j)=extr2(i,j);
end
end


for i=4:4:length(x_ext_v2)
for j=2:2:length(y_ext_v2)
    zcol_p(1/4*i,1/2*j)=extr2(i,j);
end
end


% 
% 
% 
% % define grid of the panel center locations in x and y direction
% 
% panel_center_y=y_ext_v(2:end)-(ys_ext/2);
% panel_center_x=x_ext_v(2:end)-(xs_ext/2);
% 
% % define grid of panel center locations in z direction: firs calculates how
% % much the panel height because of dihedral angle modification, then sums
% % it to the vertical displacement due to angle of attack modification
% panel_center_z=zeros(length(y_ext_v)-1,length(x_ext_v)-1);
% 
% 
% for i=1:length(x_ext_v)-1
% for j=2:length(y_ext_v)-1
% panel_center_z_dih(1,i)=(ys_ext/2)*tan(dih(1,i)) ;
% panel_center_z_dih(j,i)=panel_center_z_dih(j-1,i)+(ys_ext)*tan(dih(j,i)) ;
% end
% end
% 
% 
% for j=1:(length(y_ext_v)-1)
% panel_center_z(j,length(x_ext_v)-1)=panel_center_z_dih(j,length(x_ext_v)-1)+(xs_ext/2)*tan(aoa(j,length(x_ext_v)-1)); 
%    
% for i=2:(length(x_ext_v)-1)
% panel_center_z(j,length(x_ext_v)-i)=panel_center_z_dih(j,length(x_ext_v)-i)+panel_center_z(j,length(x_ext_v)-i)+(xs_ext)*tan(aoa(j,length(x_ext_v)-i)) ;
% % panel_center_z(j,length(x_ext_v)-i)=panel_center_z(j,length(x_ext_v)-i+1)+(xs_ext)*tan(dih(j,length(x_ext_v)-i)) 
% end
% 
% end
% 


% panel_center_z(1,j)=tan(dih(1,j))*(ys_ext/2)
% panel_center_z(i,j)=panel_center_z(i,j)+tan(dih(1,j))*(ys_ext/2)


end