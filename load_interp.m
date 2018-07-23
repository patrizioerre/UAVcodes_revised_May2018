function [fp1]=load_interp(L_panel,length_coor)
%y_int and x_int are matrices of coordinates for L_matrix matrix 
% length_coor=length(x);
y=linspace(0,0.912,length_coor)';
x=linspace(0,0.30,length_coor)';
L_panel_h=L_panel(:,size(L_panel,2)/2+1:end);

for i=1:size(L_panel_h,1) %find number of L_matrix rows
for j=1:size(L_panel_h,2)  %find number of L_matrix columns
y_int(i,:)=linspace(0,0.9,size(L_panel_h,2));
x_int(:,j)=linspace(0,0.3,size(L_panel_h,1));
  end
end
red=(length(x)*length(y))/(size(L_panel_h,1)*size(L_panel_h,2));
fp1 = griddata(x_int,y_int,L_panel_h/red,x',y,'v4');

% sx=length(x)/size(L_matrix_h,1);
% sy=length(y)/size(L_matrix_h,2);
% 
% fp2=zeros(length(x));
% for i=sx/2:sx:size(L_matrix_h,1)*sx
% for j=sy/2:sy:size(L_matrix_h,2)*sy
%     
%     f2(i,j)=L_matrix_h(i/(sx)+1/2,j/(sy)+1/2);
%     
% end 
% end
% 
% L_line=sum(L_matrix_h);
% fp3=zeros(length(x));
% 
% for j=sy/2:sy:size(L_line,2)*sy
%     
%     f3(length(x)/4,j)=L_line(j/(sy)+1/2);
%     
% end 
% 
% 

% almost working attempt at linear double-interpolating over y-axis first 
% and x-axis later

% y=linspace(0,0.9,100)';
% x=linspace(0,0.30,100)';
% y=yq_int;
% x=xq_int;
% 
% v_int=zeros(4,40);
% vq1=zeros(length(x),length(y));
% for i=1:size(L_matrix,1)
%   for j=1:size(L_matrix,2)
% y_int(i,:)=linspace(0,0.9,size(L_matrix,2));
% x_int(:,j)=linspace(0,0.3,size(L_matrix,1));
% v_int=L_matrix;
%   end
% end
% 
% for i=1:size(L_matrix,1)
% vq1(i,:)= interp1(y_int(i,:),v_int(i,:),yq_int)';
% end
% 
% for j=1:size(vq1,2)
% vq1(:,j)= interp1(x_int(:,j),v_int(:,j),yq_int)';
% end





% not working attempt at linear double-interpolating over y-axis first 
% and x-axis later

% for i=1:size(L_matrix,1) %find number of L_matrix rows
%     v_int=zeros(4,40)
%     y_int(i,:)=linspace(0,0.9,size(L_matrix,2));
%     v_int(i,:)=L_matrix(i,:);
%     
%     for j=1:size(L_matrix,2)
%     x_int(:,j)=linspace(0,0.3,size(L_matrix,1));
%     v_int(:,j)=L_matrix(:,j);
%     yq_int=y;
%     xq_int=x;
%     vq1 = interp1(y_int(i,j),v_int(i,j),yq_int(i,j));
%     
%     end
% end
% surf(x_int,y_int,v_int,xq_int,yq_int,vq1,':.');
% title('Linear Interpolation');
end