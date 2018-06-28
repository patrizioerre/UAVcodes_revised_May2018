close all

length_coor=50;
X=[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;...
   0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15];
X(2,:)=2*X(2,:);
Y=[linspace(0,0.912,length(X)); linspace(0,0.912,length(X))];

y=linspace(0,0.912,length_coor)';
x=linspace(0,0.30,length_coor)';

% plate=[0 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0 0];
% mode1r=[0 4.796388E-02 1.835426E-01 3.948400E-01 6.712726E-01 1.002145E+00 1.377145E+00 1.786438E+00 2.221035E+00  2.672858E+00 3.135179E+00 3.602668E+00 4.072039E+00; 0 4.792893E-02 1.833062E-01 3.943829E-01 6.706661E-01 1.001413E+00 1.376303E+00 1.785497E+00 2.220014E+00 2.671771E+00 3.134050E+00 3.601507E+00 4.070874E+00];
mode1r(1,:)=[0 1.194927E-02 4.724742E-02 1.034937E-01  1.795167E-01 2.736497E-01 3.849438E-01 5.118515E-01 6.532025E-01 8.074811E-01 9.736933E-01 1.150429E+00 1.336557E+00 1.530762E+00 1.732114E+00  1.939440E+00 2.151758E+00 2.368043E+00 2.587553E+00 2.809460E+00 3.033065E+00 3.257729E+00 3.483059E+00 3.708706E+00 3.934461E+00 ];
mode1r(2,:)=mode1r(1,:);
mode1rn=mode1r./max(max(mode1r));

% mode2r=[0 -1.671517E-01 -5.416659E-01 -9.667804E-01 -1.381109E+00 -1.767986E+00 -2.115188E+00 -2.416296E+00 -2.663850E+00 -2.855460E+00 -2.987396E+00 -3.066112E+00 -3.106387E+00;0 1.761629E-01 5.673431E-01 1.007953E+00 1.438760E+00 1.847798E+00 2.225544E+00 2.567287E+00 2.864933E+00 3.114467E+00 3.308713E+00 3.451023E+00 3.554024E+00];
mode2r(1,:)=[0 -5.578748E-02 -1.934408E-01 -3.833468E-01 -5.983438E-01 -8.152701E-01 -1.028237E+00 -1.235503E+00 -1.436775E+00 -1.632780E+00 -1.820483E+00 -1.997840E+00 -2.165560E+00 -2.325751E+00 -2.474550E+00 -2.609493E+00 -2.731867E+00 -2.844736E+00 -2.943709E+00 -3.026177E+00 -3.093982E+00 -3.151293E+00 -3.195369E+00 -3.225816E+00 -3.247150E+00 ];
mode2r(2,:)=[0 5.578748E-02 1.934408E-01 3.833468E-01 5.983438E-01 8.152701E-01 1.028237E+00 1.235503E+00 1.436775E+00 1.632780E+00 1.820483E+00 1.997840E+00 2.165560E+00 2.325751E+00 2.474550E+00 2.609493E+00 2.731867E+00 2.844736E+00 2.943709E+00 3.026177E+00 3.093982E+00 3.151293E+00 3.195369E+00 3.225816E+00 3.247150E+00];
mode2rn=mode2r./(-mode2r(1,25));

% mode3r=[0.0 2.677361E-01 9.063684E-01 1.675234E+00 2.371237E+00 2.823309E+00 2.914814E+00 2.582982E+00 1.832365E+00 7.171803E-01 -6.654797E-01 -2.205163E+00 -3.809944E+00; 0.0 2.751693E-01 9.281050E-01 1.712480E+00 2.424054E+00 2.891046E+00 2.996910E+00 2.678116E+00 1.939215E+00 8.336853E-01 -5.416136E-01 -2.076264E+00 -3.678337E+00];
mode3r(1,:)=[0 -7.753934E-02 -2.811767E-01 -5.715109E-01 -9.184335E-01 -1.293596E+00 -1.672789E+00 -2.031545E+00 -2.346864E+00 -2.600039E+00 -2.775749E+00 -2.862467E+00 -2.849756E+00 -2.733128E+00 -2.511168E+00 -2.188107E+00 -1.768708E+00  -1.262193E+00 -6.788715E-01 -3.319665E-02 6.610465E-01 1.389419E+00 2.140587E+00 2.903551E+00 3.670894E+00 ];
mode3r(2,:)=mode3r(1,:);
mode3rn=mode3r./max(max(mode3r));

% mode4r=[0.0 -6.659509E-01 -1.957754E+00 -2.917045E+00 -2.988660E+00 -2.027915E+00 -3.827361E-01 1.326637E+00 2.428856E+00 2.441301E+00 1.298578E+00 -7.342263E-01 -3.202817E+00; 0.0 -6.391058E-01 -1.866428E+00 -2.762179E+00 -2.797172E+00 -1.829380E+00 -2.119280E-01 1.443529E+00 2.475327E+00 2.410855E+00 1.200241E+00 -8.870320E-01 -3.397897E+00];
mode4r(1,:)=[0 -2.055985E-01 -6.922932E-01 -1.305111E+00 -1.920268E+00 -2.433848E+00 -2.775364E+00 -2.893619E+00 -2.758346E+00 -2.367676E+00 -1.758957E+00 -9.948077E-01 -1.506832E-01 6.927494E-01 1.452219E+00 2.047884E+00 2.410009E+00 2.493395E+00 2.285286E+00 1.799364E+00 1.064712E+00 1.287662E-01 -9.508771E-01 -2.110573E+00 -3.302102E+00 ];
mode4r(2,:)=mode4r(1,:);
mode4rn=mode4r./mode4r(1,length(mode4r(1,:)));

% mode5r=[0.0 -4.690301E-01 -1.448330E+00 -2.358053E+00 -2.898754E+00 -2.985879E+00 -2.569407E+00 -1.730217E+00 -5.808191E-01 6.872018E-01 1.866902E+00 2.818524E+00 3.524245E+00;0.0 5.638687E-01 1.726087E+00 2.781741E+00 3.372033E+00 3.399857E+00 2.835409E+00 1.809392E+00 4.866156E-01 -8.954875E-01 -2.110548E+00 -3.025035E+00 -3.646919E+00];
mode5r(1,:)=[0  1.715441E-01  5.809319E-01  1.120372E+00  1.696676E+00  2.231434E+00  2.664839E+00  2.966272E+00  3.142228E+00  3.218981E+00  3.156259E+00  2.936602E+00  2.586883E+00  2.156131E+00  1.638051E+00  1.038872E+00  3.878387E-01 -2.751048E-01 -9.182878E-01 -1.515591E+00 -2.053697E+00 -2.529655E+00 -2.910472E+00 -3.186157E+00 -3.388186E+00 ];
mode5r(2,:)=[0 -1.715441E-01 -5.809319E-01 -1.120372E+00 -1.696676E+00 -2.231434E+00 -2.664839E+00 -2.966272E+00 -3.142228E+00 -3.218981E+00 -3.156259E+00 -2.936602E+00 -2.586883E+00 -2.156131E+00 -1.638051E+00 -1.038872E+00 -3.878387E-01  2.751048E-01  9.182878E-01  1.515591E+00  2.053697E+00  2.529655E+00  2.910472E+00  3.186157E+00  3.388186E+00 ];
mode5rn=mode5r./max(max(mode5r));


% mode1s = griddata(X,Y,mode1rn,x,y','natural');
% mode2s = griddata(X,Y,mode2rn,x,y','natural');
% mode3s = griddata(X,Y,mode3rn,x,y','cubic');
% mode4s = griddata(X,Y,mode4rn,x,y','cubic');
% mode5s = griddata(X,Y,mode5rn,x,y','cubic');

syms x1 x2 x3 x4 x5 der1f der2f der3f der4f
 
% viene usando questa a*cosh((alfa/0.912)*x)+b*cos((alfa/0.912)*x)+d*sinh((alfa/0.912)*x)+e*sin((alfa/0.912)*x)

% interpy1=mode1rn(1,:);
% interpx1=Y(1,:);

a =      0.5065  ;
alfa =       1.853  ;
b =     -0.5066  ;
d =     -0.3718;
e =      0.3751  ;

f1= a*cosh((alfa/0.912)*x1)+b*cos((alfa/0.912)*x1)+d*sinh((alfa/0.912)*x1)+e*sin((alfa/0.912)*x1);

func=eval(subs(f1,y));
mode1n=func.*ones(length(x))';
mode1s=mode1n./max(max(abs(mode1n)));

% [ics,ips]=meshgrid(x,y);
% mode1s=feval(fittedmodel,ics,ips);
% fi1=sqrt(1/trapz(y,trapz(x,mode1s.*mode1s,2)))*mode1s;
% [wdiffx1,wdiffxx1,wdiffxxx1,wdiffxxxx1,wdiffy1,wdiffyy1,wdiffyyy1,wdiffyyyy1]=differentiation(x,y,fi1);
% figure(1)
% surf(fi1)
% figure(2)
% surf(wdiffy1)
% figure(3)
% surf(wdiffyy1)
% figure(4)
% surf(wdiffyyy1)
% figure(5)
% surf(wdiffyyyy1)
% figure(6)
% surf(wdiffxx1)
% figure(7)
% surf(wdiffxxxx1)







interpx2=Y(1,:);
interpy21=mode2rn(1,:);

  a =     -0.4564 ;
       alfa =      -1.229 ; 
       b =      0.4913  ;
       beta =      -2.553;  
       d =      -0.396 ;
       e =        1.06 ;
       
f2=a*cosh((alfa/0.912)*x2)+b*cos((alfa/0.912)*x2)+d*sinh((alfa/0.912)*x2)...
       +e*sin((alfa/0.912)*x2)+  a*cosh((beta/0.912)*x2)+b*cos((beta/0.912)*x2)+...
       d*sinh((beta/0.912)*x2)+e*sin((beta/0.912)*x2);
func2=eval(subs(f2,y));
mode2n=func2.*([x-0.15])';
mode2s=mode2n./max(max(abs(mode2n)));

% a =     -0.2659  ;
% alfa =      -2.739  ;
% b =      0.2837  ;
% d =     -0.1994  ;
% e =      0.5476  ;
% 
% f2= a*cosh((alfa/0.912)*x2)+b*cos((alfa/0.912)*x2)+d*sinh((alfa/0.912)*x2)+e*sin((alfa/0.912)*x2);
% 
% func2=eval(subs(f2,y));
% mode2n=func2.*([x-0.15])';
% mode2s=mode2n./max(max(abs(mode2n)));
% fi2=sqrt(1/trapz(y,trapz(x,mode2s.*mode2s,2)))*mode2s;
% [wdiffx2,wdiffxx2,wdiffxxx2,wdiffxxxx2,wdiffy2,wdiffyy2,wdiffyyy2,wdiffyyyy2]=differentiation(x,y,fi2);
% 
% x2=x(1:length(x)-1)
% interpy22=wdiffy2(:,1);
% func2=feval(fittedmodel2,x2);
% wdiffy2i=func2.*([x2-0.15])';
% [wdiffx2,wdiffxx2,wdiffxxx2,wdiffxxxx2,wdiffy2,wdiffyy2,wdiffyyy2,wdiffyyyy2]=differentiation(x,y,wdiffy2i);

% % [ics,ips]=meshgrid(x,y);
% % mode2s=feval(fittedmodel,ics,ips);
% 
% figure(1)
% surf(fi2)
% figure(2)
% surf(wdiffy2)
% figure(3)
% surf(wdiffyy2)
% figure(4)
% surf(wdiffyyy2)
% figure(5)
% surf(wdiffyyyy2)
% figure(6)
% surf(wdiffxx2)
% figure(7)
% surf(wdiffxxxx2)

%per secondo modo non funziona





% interpy3=mode3rn(1,:);
% interpx3=Y(1,:);

a =     -0.5246  ;
alfa =      -4.609  ;
b =      0.5263  ;
d =     -0.5345 ;
e =      0.5664  ;

f3= a*cosh((alfa/0.912)*x3)+b*cos((alfa/0.912)*x3)+d*sinh((alfa/0.912)*x3)+e*sin((alfa/0.912)*x3);
func3=eval(subs(f3,y));
mode3n=func3.*ones(length(x))';
mode3s=mode3n./max(max(abs(mode3n)));


% [ics,ips]=meshgrid(x,y);
% mode3s=feval(fittedmodel,ics,ips);
% fi3=sqrt(1/trapz(y,trapz(x,mode3s.*mode3s,2)))*mode3s;
% [wdiffx3,wdiffxx3,wdiffxxx3,wdiffxxxx3,wdiffy3,wdiffyy3,wdiffyyy3,wdiffyyyy3]=differentiation(x,y,fi3);
% figure(1)
% surf(fi3)
% figure(2)
% hold on
% surf(wdiffy3)
% figure(3)
% hold on
% surf(wdiffyy3)
% figure(4)
% hold on
% surf(wdiffyyy3)
% figure(5)
% hold on
% surf(wdiffyyyy3)
% figure(6)
% surf(wdiffxx3)
% figure(7)
% surf(wdiffxxxx3)




% interpy4=mode4rn(1,:);
% interpx4=Y(1,:);
a =      0.5412  ;
alfa =      -7.688  ;
b =     -0.5429  ;
d =      0.5408  ;
e =     -0.6062  ;

f4= a*cosh((alfa/0.912)*x4)+b*cos((alfa/0.912)*x4)+d*sinh((alfa/0.912)*x4)+e*sin((alfa/0.912)*x4);
func4=eval(subs(f4,y));
mode4n=func4.*ones(length(x))';
mode4s=mode4n./max(max(abs(mode4n)));
% 
% fi4=sqrt(1/trapz(y,trapz(x,mode4s.*mode4s,2)))*mode4s;
% [wdiffx4,wdiffxx4,wdiffxxx4,wdiffxxxx4,wdiffy4,wdiffyy4,wdiffyyy4,wdiffyyyy4]=differentiation(x,y,fi4);
% 
% figure(1)
% surf(fi4)
% figure(2)
% surf(wdiffy4)
% figure(3)
% surf(wdiffyy4)
% figure(4)
% surf(wdiffyyy4)
% figure(5)
% surf(wdiffyyyy4)
% figure(6)
% surf(wdiffxx4)
% figure(7)
% surf(wdiffxxxx4)

% viene circa con gaussiana ottavo ordine


% interpy5=mode5rn(1,:);
% interpx5=Y(1,:);

a =      0.3546  ;
alfa =       5.184  ;
b =     -0.3942  ;
d =     -0.3555  ;
e =      0.8046  ;

f5= a*cosh((alfa/0.912)*x5)+b*cos((alfa/0.912)*x5)+d*sinh((alfa/0.912)*x5)+e*sin((alfa/0.912)*x5);
func5=eval(subs(f5,y));
mode5n=func5.*([x-0.15])';
mode5s=mode5n./max(max(abs(mode5n)));

y=linspace(0,0.912,length_coor)';
x=linspace(0,0.3,length_coor)';

D=2252;
density=76.3;
thickness=0.0169;
rho=density*thickness;


% fi1=sqrt(1/trapz(y,trapz(x,mode1n.*mode1n,2)))*mode1n;
% fi2=10000000*sqrt(1/trapz(y,trapz(x,mode2n.*mode2n,2)))*mode2n;
% fi3=sqrt(1/trapz(y,trapz(x,mode3n.*mode3n,2)))*mode3n;
% fi4=sqrt(1/trapz(y,trapz(x,mode4n.*mode4n,2)))*mode4n;
% fi5=8000000000*sqrt(1/trapz(y,trapz(x,mode5n.*mode5n,2)))*mode5n;

fi1=sqrt(1/trapz(y,trapz(x,rho*mode1s.*mode1s,2)))*mode1s;
fi2=sqrt(1/trapz(y,trapz(x,rho*mode2s.*mode2s,2)))*mode2s;
fi3=sqrt(1/trapz(y,trapz(x,rho*mode3s.*mode3s,2)))*mode3s;
fi4=sqrt(1/trapz(y,trapz(x,rho*mode4s.*mode4s,2)))*mode4s;
fi5=sqrt(1/trapz(y,trapz(x,rho*mode5s.*mode5s,2)))*mode5s;

[wdiffx1,wdiffxx1,wdiffxxx1,wdiffxxxx1,wdiffy1,wdiffyy1,wdiffyyy1,wdiffyyyy1]=differentiation(x,y,fi1);
[wdiffx2,wdiffxx2,wdiffxxx2,wdiffxxxx2,wdiffy2,wdiffyy2,wdiffyyy2,wdiffyyyy2]=differentiation(x,y,fi2);
[wdiffx3,wdiffxx3,wdiffxxx3,wdiffxxxx3,wdiffy3,wdiffyy3,wdiffyyy3,wdiffyyyy3]=differentiation(x,y,fi3);
[wdiffx4,wdiffxx4,wdiffxxx4,wdiffxxxx4,wdiffy4,wdiffyy4,wdiffyyy4,wdiffyyyy4]=differentiation(x,y,fi4);
[wdiffx5,wdiffxx5,wdiffxxx5,wdiffxxxx5,wdiffy5,wdiffyy5,wdiffyyy5,wdiffyyyy5]=differentiation(x,y,fi5);

fi1=fi1(1:length(x)-4,1:length(x)-4);
fi2=fi2(1:length(x)-4,1:length(x)-4);
fi3=fi3(1:length(x)-4,1:length(x)-4);
fi4=fi4(1:length(x)-4,1:length(x)-4);
fi5=fi5(1:length(x)-4,1:length(x)-4);

wdiffxx1=wdiffxx1(1:length(x)-4,1:length(x)-4);
wdiffxxxx1=wdiffxxxx1(1:length(x)-4,1:length(x)-4);
wdiffyy1=wdiffyy1(1:length(x)-4,1:length(x)-4);
wdiffyyyy1=wdiffyyyy1(1:length(x)-4,1:length(x)-4);

wdiffxx2=wdiffxx2(1:length(x)-4,1:length(x)-4);
wdiffxxxx2=wdiffxxxx2(1:length(x)-4,1:length(x)-4);
wdiffyy2=wdiffyy2(1:length(x)-4,1:length(x)-4);
wdiffyyyy2=wdiffyyyy2(1:length(x)-4,1:length(x)-4);

wdiffxx3=wdiffxx3(1:length(x)-4,1:length(x)-4);
wdiffxxxx3=wdiffxxxx3(1:length(x)-4,1:length(x)-4);
wdiffyy3=wdiffyy3(1:length(x)-4,1:length(x)-4);
wdiffyyyy3=wdiffyyyy3(1:length(x)-4,1:length(x)-4);

wdiffxx4=wdiffxx4(1:length(x)-4,1:length(x)-4);
wdiffxxxx4=wdiffxxxx4(1:length(x)-4,1:length(x)-4);
wdiffyy4=wdiffyy4(1:length(x)-4,1:length(x)-4);
wdiffyyyy4=wdiffyyyy4(1:length(x)-4,1:length(x)-4);

wdiffxx5=wdiffxx5(1:length(x)-4,1:length(x)-4);
wdiffxxxx5=wdiffxxxx5(1:length(x)-4,1:length(x)-4);
wdiffyy5=wdiffyy5(1:length(x)-4,1:length(x)-4);
wdiffyyyy5=wdiffyyyy5(1:length(x)-4,1:length(x)-4);

x=x(1:length(x)-4);
y=y(1:length(y)-4);

der_of_1=wdiffxxxx1+2*wdiffxx1.*wdiffyy1+wdiffyyyy1;
der_of_2=wdiffxxxx2+2*wdiffxx2.*wdiffyy2+wdiffyyyy2;
der_of_3=wdiffxxxx3+2*wdiffxx3.*wdiffyy3+wdiffyyyy3;
der_of_4=wdiffxxxx4+2*wdiffxx4.*wdiffyy4+wdiffyyyy4;
der_of_5=wdiffxxxx5+2*wdiffxx5.*wdiffyy5+wdiffyyyy5;



% lambda1=D*trapz(y,trapz(x,fi1.*(wdiffxxxx1+2*wdiffxx1.*wdiffyy1+wdiffyyyy1),2))/rho*trapz(y,trapz(x,fi1.*fi1,2));
% lambda2=D*trapz(y,trapz(x,fi2.*(wdiffxxxx2+2*wdiffxx2.*wdiffyy2+wdiffyyyy2),2))/rho*trapz(y,trapz(x,fi2.*fi2,2));
% lambda3=D*trapz(y,trapz(x,fi3.*(wdiffxxxx3+2*wdiffxx3.*wdiffyy3+wdiffyyyy3),2))/rho*trapz(y,trapz(x,fi3.*fi3,2));
% lambda4=D*trapz(y,trapz(x,fi4.*(wdiffxxxx4+2*wdiffxx4.*wdiffyy4+wdiffyyyy4),2))/rho*trapz(y,trapz(x,fi4.*fi4,2));
% lambda5=D*trapz(y,trapz(x,fi5.*(wdiffxxxx5+2*wdiffxx5.*wdiffyy5+wdiffyyyy5),2))/rho*trapz(y,trapz(x,fi5.*fi5,2));
% 
% freq1=sqrt(lambda1)/(2*pi)
% freq2=sqrt(lambda2)/(2*pi)
% freq3=sqrt(lambda3)/(2*pi)
% freq4=sqrt(lambda4)/(2*pi)
% freq5=sqrt(lambda5)/(2*pi)
% 
% freq=[freq1 freq2 freq3 freq4 freq5]

K11=D*trapz(y,trapz(x,fi1.*der_of_1,2));
K12=D*trapz(y,trapz(x,fi1.*der_of_2,2));
K13=D*trapz(y,trapz(x,fi1.*der_of_3,2));
K14=D*trapz(y,trapz(x,fi1.*der_of_4,2));
K15=D*trapz(y,trapz(x,fi1.*der_of_5,2));
K21=D*trapz(y,trapz(x,fi2.*der_of_1,2));
K22=D*trapz(y,trapz(x,fi2.*der_of_2,2));
K23=D*trapz(y,trapz(x,fi2.*der_of_3,2));
K24=D*trapz(y,trapz(x,fi2.*der_of_4,2));
K25=D*trapz(y,trapz(x,fi2.*der_of_5,2));
K31=D*trapz(y,trapz(x,fi3.*der_of_1,2));
K32=D*trapz(y,trapz(x,fi3.*der_of_2,2));
K33=D*trapz(y,trapz(x,fi3.*der_of_3,2));
K34=D*trapz(y,trapz(x,fi3.*der_of_4,2));
K35=D*trapz(y,trapz(x,fi3.*der_of_5,2));
K41=D*trapz(y,trapz(x,fi4.*der_of_1,2));
K42=D*trapz(y,trapz(x,fi4.*der_of_2,2));
K43=D*trapz(y,trapz(x,fi4.*der_of_3,2));
K44=D*trapz(y,trapz(x,fi4.*der_of_4,2));
K45=D*trapz(y,trapz(x,fi4.*der_of_5,2));
K51=D*trapz(y,trapz(x,fi5.*der_of_1,2));
K52=D*trapz(y,trapz(x,fi5.*der_of_2,2));
K53=D*trapz(y,trapz(x,fi5.*der_of_3,2));
K54=D*trapz(y,trapz(x,fi5.*der_of_4,2));
K55=D*trapz(y,trapz(x,fi5.*der_of_5,2));



M11=rho*trapz(y,trapz(x,fi1.*fi1,2));
M12=rho*trapz(y,trapz(x,fi1.*fi2,2));
M13=rho*trapz(y,trapz(x,fi1.*fi3,2));
M14=rho*trapz(y,trapz(x,fi1.*fi4,2));
M15=rho*trapz(y,trapz(x,fi1.*fi5,2));
M21=rho*trapz(y,trapz(x,fi2.*fi1,2));
M22=rho*trapz(y,trapz(x,fi2.*fi2,2));
M23=rho*trapz(y,trapz(x,fi2.*fi3,2));
M24=rho*trapz(y,trapz(x,fi2.*fi4,2));
M25=rho*trapz(y,trapz(x,fi2.*fi5,2));
M31=rho*trapz(y,trapz(x,fi3.*fi1,2));
M32=rho*trapz(y,trapz(x,fi3.*fi2,2));
M33=rho*trapz(y,trapz(x,fi3.*fi3,2));
M34=rho*trapz(y,trapz(x,fi3.*fi4,2));
M35=rho*trapz(y,trapz(x,fi3.*fi5,2));
M41=rho*trapz(y,trapz(x,fi4.*fi1,2));
M42=rho*trapz(y,trapz(x,fi4.*fi2,2));
M43=rho*trapz(y,trapz(x,fi4.*fi3,2));
M44=rho*trapz(y,trapz(x,fi4.*fi4,2));
M45=rho*trapz(y,trapz(x,fi4.*fi5,2));
M51=rho*trapz(y,trapz(x,fi5.*fi1,2));
M52=rho*trapz(y,trapz(x,fi5.*fi2,2));
M53=rho*trapz(y,trapz(x,fi5.*fi3,2));
M54=rho*trapz(y,trapz(x,fi5.*fi4,2));
M55=rho*trapz(y,trapz(x,fi5.*fi5,2));

K=[K11 K12 K13 K14 K15;K21 K22 K23 K24 K25;K31 K32 K33 K34 K35;K41 K42 K43 K44 K45;K51 K52 K53 K54 K55];

M=[M11 M12 M13 M14 M15;M21 M22 M23 M24 M25;M31 M32 M33 M34 M35;M41 M42 M43 M44 M45;M51 M52 M53 M54 M55];

[eigvec,lambda]=eig(M\K);

freq=(sqrt(lambda))/(2*pi)


y=linspace(0,0.912,length_coor)';
x=linspace(0,0.3,length_coor)';
fi1_c=sqrt(1/trapz(y,trapz(x,rho*mode1s.*mode1s,2)))*mode1s;
fi2_c=sqrt(1/trapz(y,trapz(x,rho*mode2s.*mode2s,2)))*mode2s;
fi3_c=sqrt(1/trapz(y,trapz(x,rho*mode3s.*mode3s,2)))*mode3s;
fi4_c=sqrt(1/trapz(y,trapz(x,rho*mode4s.*mode4s,2)))*mode4s;
fi5_c=sqrt(1/trapz(y,trapz(x,rho*mode5s.*mode5s,2)))*mode5s;

fi4=(eigvec(1,1)*fi1_c+eigvec(2,1)*fi2_c+eigvec(3,1)*fi3_c+eigvec(4,1)*fi4_c+eigvec(5,1)*fi5_c);
fi3=(eigvec(1,2)*fi1_c+eigvec(2,2)*fi2_c+eigvec(3,2)*fi3_c+eigvec(4,2)*fi4_c+eigvec(5,2)*fi5_c);
fi5=eigvec(1,3)*fi1_c+eigvec(2,3)*fi2_c+eigvec(3,3)*fi3_c+eigvec(4,3)*fi4_c+eigvec(5,3)*fi5_c;
fi1=-(eigvec(1,4)*fi1_c+eigvec(2,4)*fi2_c+eigvec(3,4)*fi3_c+eigvec(4,4)*fi4_c+eigvec(5,4)*fi5_c);
fi2=-(eigvec(1,5)*fi1_c+eigvec(2,5)*fi2_c+eigvec(3,5)*fi3_c+eigvec(4,5)*fi4_c+eigvec(5,5)*fi5_c);



lambda1=lambda(2,2);
lambda2=lambda(4,4);
lambda3=lambda(3,3);
lambda4=lambda(1,1);
lambda5=lambda(5,5);



