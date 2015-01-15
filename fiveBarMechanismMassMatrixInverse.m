
function fiveBarMechanismMassMatrixInverse

%Parameters

l1=.1;
l2=.1;
l3=l1;
l4=l2;
m1= .05;
m2=.05;
m3=m1;
m4=m2;
IG1= m1*l1^2/12;
IG2=m2*l2^2/12;
IG3=IG1;
IG4=IG2;
scale=.1;
int=10;

 for beta=0:(2*pi/int):2*pi;
    for r=0.01:(.199-.01)/int:.199;



       x=r*cos(beta);
       y=r*sin(beta) ;

       %INVERSE KINEMATICS
       phi=atan2(y,x);


       th2=acos(((x^2+y^2)-l1^2-l2^2)/(2*l1*l2))
       alpha=th2/(-2);
       th1=phi+alpha
       th3=phi-alpha
       th4=2*alpha

        %%%%%%%%

%         %FORWARD KINEMATICS
x1=l1*cos(th1);
y1=l1*sin(th1);
x2=l1*cos(th3);
y2=l1*sin(th3);
xop=x1+l2*cos(th1+th2);
yop=y1+l2*sin(th1+th2);
xop_vec=[xop;yop];
x1_vec=[x1;y1];
x2_vec=[x2;y2] ;

%%%%

  % Mass matrix

  M11=m1*l1^2/4+m2*l1^2+m2*l2^2/4+m2*l1*l2*cos(th2)+IG1+IG2;
  M12=m2*(l1*l2/2*cos(th2)+l2^2/4)+IG2;
  M21=m2*(l2^2/4+l1*l2/2*cos(th2))+IG2;
  M22=m2*l2^2/4+IG2;

  M33=m3*l3^2/4+m4*l3^2+m4*l4^2/4+m4*l3*l4*cos(th4)+IG3+IG4;
  M34=m4*(l3*l4/2*cos(th4)+l4^2/4)+IG4;
  M43=m4*(l4^2/4+l3*l4/2*cos(th4))+IG4;
  M44=m4*l4^2/4+IG4;



  M=[M11,M12,0,0;M21,M22,0,0;0,0,M33,M34;0,0,M43,M44;];


  J1=[-l1*sin(th1)-l2*sin(th1+th2), -l2*sin(th1+th2); l1*cos(th1)+l2*cos(th1+th2), l2*cos(th1+th2)];
  J2=[-l3*sin(th3)-l4*sin(th3+th4), -l4*sin(th3+th4); l3*cos(th3)+l4*cos(th3+th4), l4*cos(th3+th4)];

  %COMMENTED OUT FOR THE SAKE OF CHECKING THE SINGLE MASS MATRIX IF IT
  %WORKS WITH THE ELLIPSE DISPLAY

  C=[J1(:,1) -J2(:,1)];
  D=[-J1(:,2) J2(:,2)];


 Dhat=D\C;

 A=[1,0;Dhat(1,:);0,1;Dhat(2,:);];

 J=J1*[1,0;Dhat(1,:)];
 %JO=J2*[0,1;Dhat(2,:)];

MStar=(A'*M*A)\eye(2,2);
EMM=J*MStar;
EMM=EMM*J';
EMM=EMM\eye(2)

 [V,L]=eig(EMM);

% V=eye(2);
% L=eye(2);


%%%%%%%%%%%%%%%%%%%%%%%

 a=scale*L(1,1);
 b=scale*L(2,2);
%  a=scale;
%  b=scale;

  R=V ;
  dot(V(:,1),V(:,2));
  V*V';

  clf;
 X= drawEllipse(a,b,R,xop_vec);
 Y= drawDevice(x1_vec, xop_vec, x2_vec);
  hold on;
  axis([-.25 .25 -.25 .25])
  daspect([1 1 1])
plot(X(:,1), X(:,2), 'b-')
plot(Y(:,1), Y(:,2), 'r--o')

waitforbuttonpress;
pause(.1)

   end
end
clc;

end



function X=drawEllipse(a,b,R,x)

%drawEllipse(semi major axis, semi minor axis, angle of rotation, x(1)= x,
%x(2)=y)
   X= zeros(36,2);

    alpha = linspace(0, 360, 36)' .* (pi / 180);
    sinalpha = sin(alpha);
    cosalpha = cos(alpha);

    for i=1:36
   y= x+R*[a*cosalpha(i), b*sinalpha(i)]';

   X(i,1)=y(1);
   X(i,2)=y(2);
    end





end

function X=drawDevice(x1_vec,xop_vec,x2_vec)

%drawEllipse(semi major axis, semi minor axis, angle of rotation, x(1)= x,
%x(2)=y)
   X= zeros(4,2);

 X(1,:)=[0,0];
 X(2,:)=x1_vec';
 X(3,:)=xop_vec';
 X(4,:)=x2_vec';
 X(5,:)=X(1,:);




end
