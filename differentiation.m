function [wdiffx,wdiffxx,wdiffxxx,wdiffxxxx,wdiffy,wdiffyy,wdiffyyy,wdiffyyyy]=differentiation(x,y,w)

xs=x(2)-x(1);
ys=y(2)-y(1);

for i=1:length(x)
wdiffyc(:,i)=diff(w(:,i));
wdiffy(:,i)=wdiffyc(:,i)/ys;
end

for i=1:length(y)
wdiffxc(i,:)=diff(w(i,:));
wdiffx(i,:)=wdiffxc(i,:)/xs;
end

for i=1:(length(x)-1)
wdiffyyc(:,i)=diff(wdiffyc(:,i));
wdiffyy=wdiffyyc/ys^2;
end

for i=1:(length(y)-1)
wdiffxxc(i,:)=diff(wdiffxc(i,:));
wdiffxx=wdiffxxc/xs^2;
end

for i=1:(length(x)-2)
wdiffyyyc(:,i)=diff(wdiffyyc(:,i));
wdiffyyy=wdiffyyyc/ys^3;
end

for i=1:(length(y)-2)
wdiffxxxc(i,:)=diff(wdiffxxc(i,:));
wdiffxxx=wdiffxxxc/xs^3;
end

for i=1:(length(y)-2)
wdiffyyxc(i,:)=diff(wdiffyyc(i,:));
wdiffyyx=wdiffyyxc/(ys^2*xs);
end

for i=1:(length(x)-2)
wdiffxxyc(:,i)=diff(wdiffxxc(:,i));
wdiffxxy=wdiffxxyc/(xs^2*ys);
end

for i=1:(length(x)-3)
wdiffyyyyc(:,i)=diff(wdiffyyyc(:,i));
wdiffyyyy=wdiffyyyyc/ys^4;
end

for i=1:(length(y)-3)
wdiffxxxxc(i,:)=diff(wdiffxxxc(i,:));
wdiffxxxx=wdiffxxxxc/xs^4;
end


% figure(1)
% hold on
% surf(wdiffx)
% figure(2)
% hold on
% surf(wdiffxx)
% figure(3)
% hold on
% surf(wdiffxxxx)


end
% lam1=2950000;
% difference=D*(wdiffxxxx(1:96,1:96)+2*wdiffxx(1:96,1:96).*wdiffyy(1:96,1:96)+wdiffyyyy(1:96,1:96))-lam1*fi1(1:96,1:96);
% surf(x(1:96),y(1:96),difference)
% 
% for i=1:10000
%     D=linspace(2920000,294000,1);
% difference(i)=D(i)*(wdiffxxxx(1:96,1:96)+2*wdiffxx(1:96,1:96).*wdiffyy(1:96,1:96)+wdiffyyyy(1:96,1:96))-1.608495438637974e+05*mode2n(1:96,1:96);
% N(i)=norm(difference(i))
% end
% [Y,p]=min(difference)
% surf(x(1:96),y(1:96),difference)