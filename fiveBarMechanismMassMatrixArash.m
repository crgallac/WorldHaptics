function fiveBarMechanismMassMatrixArash

L1=.1;
L2=.1;
m1=.05; 
m2=.05; 
I1=m1*L1^2/12;
I2=m2*L2^2/12;
int=10; 
scale=.1; 

th=[-pi/4;pi/4]; 
beta=0; 

for beta=0:pi/int:2*pi;  
    
q1=th(1)+beta;
q3=th(2)+beta;

alph=(q3-q1)/2;
psi=(q1+q3)/2;
phi=asin(L1*sin(alph)/L2);
s=L1*cos(alph)/(L2*cos(phi));

c1=I1+1/4*m1*L1^2+m2*L1^2+1/2*(1+s^2)*(I2+1/4*m2*L2^2)+1/2*m2*L1*L2*(1-s)*cos(alph+phi);
c12=(1/4*m2*L2^2+I2)*(1-s^2)+m2*L1*L2*(1+s)*cos(alph+phi);

M=[c1 c12/2;c12/2 c1];

a=-L1*sin(alph)*(1+s);
b=L1*cos(alph)+L2*cos(phi);

%operation point

xop_vec=[b*cos(psi); b*sin(psi)]; 

Q=[-1 1;1 1]*[cos(psi)/a sin(psi)/a;-sin(psi)/b cos(psi)/b];

M_E=Q'*M*Q
%invQ=Q\eye(2,2);
%M_E=invQ'*M*invQ;

[V,L]=eig(M_E);
L
chol(M_E);


%%%%%%%%%%%%%%%%%%%%%%%

 ae=scale*L(1,1);
 be=scale*L(2,2); 
%  a=scale;
%  b=scale;
 
  R=V';  
  %det(R)
  norm(V(:,1))
  norm(V(:,2))
  dot(V(:,1),V(:,2))
  V*V'
% R=[cos(phi), -sin(phi); sin(phi), cos(phi);]; 
  
 %clf; 
 X= drawEllipse(ae,be,R,xop_vec);    
 Y= drawDevice( xop_vec);    
  hold on; 
  axis([-.2 .2 -.2 .2])
  daspect([1 1 1])
plot(X(:,1), X(:,2), 'b-')
plot(Y(:,1), Y(:,2), 'r--o')
 
waitforbuttonpress;  
pause(.001)
  
end
clc;

end



function X=drawEllipse(a,b,R,x)

%drawEllipse(semi major axis, semi minor axis, angle of rotation, x(1)= x,
%x(2)=y)
numofpoints=100;

   X= zeros(numofpoints,2);  
   
    alpha = linspace(0, 360, numofpoints)' .* (pi / 180);
    sinalpha = sin(alpha);
    cosalpha = cos(alpha);

    for i=1:numofpoints
   y= x+R*[a*cosalpha(i), b*sinalpha(i)]'; 
   
   X(i,1)=y(1); 
   X(i,2)=y(2); 
    end
    
    



end

function X=drawDevice(xop_vec)

%drawEllipse(semi major axis, semi minor axis, angle of rotation, x(1)= x,
%x(2)=y)
   X= zeros(3,2);  
   
 X(1,:)=[0,0];
 X(2,:)=xop_vec'; 
 X(3,:)=X(1,:); 
    



end