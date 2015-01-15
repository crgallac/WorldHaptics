%for tomorrow we need to figure out why there is a lack of symmetry in this
%code i believe it comes down to my poor use of ambiguous angles perhaps?

%try drawing the location of xop from the point of x2

function fiveBarMechanismMassMatrixGeneralWithDistance

%Initialize windows and object handles
clf;
s1=subplot(1,2,1);

%Creating a color map for the workspace analysis
R1=zeros(1,25); 
G1=linspace(0,1,25);
B1=linspace(1,0,25); 

R2=linspace(0,1,25); 
G2=linspace(1,0,25); 
B2=zeros(1,25); 

CNA=linspace(1,10,50);%%% is 10 the appropriate bound? 
CNMAX=25; 

%COLORMAPPING
map=[CNA(1:25)' R1' G1' B1'; CNA(26:50)' R2' G2' B2'];


% hsv(128)

%Dynamic Parameters 
l1=.1; %link 1 of the left serial manipulator
l2=.12;%link 2 of the left serial manipulator
l3=.12;%link 1 of the right serial manipulator
l4=.1;%link 2 of the right serial manipulator
d=0.05; %distance between first revolute joints
m1= .05; %mass of first link of the left serial manipulator
m2=.05;%mass of second link of the left serial manipulator
m3=.05;%mass of first link of the right serial manipulator
m4=.05;%mass of second link of the right serial manipulator
IG1= m1*l1^2/12; %moment of inertia about l1 center of mass
IG2=m2*l2^2/12;%moment of inertia about l2 center of mass
IG3=IG1;%moment of inertia about l3 center of mass
IG4=IG2;%moment of inertia about l4 center of mass

%additional Parameters
scale=.1; %scaling of the effective mass matrix ellipse
int=20; %number of intervals used for the for loops
ll=l1+l2; 
lr=l3+l4; 

if(l1>lr)
rtot=ll;
else
rtot=lr; 
end

ltot=rtot+rtot/3; 
%for loop for parameterization of a circle
 for beta=0:(pi/int):pi; 
    for r=0.01:((ltot-1e-4)-.01)/int:ltot;


%%%%%%%%%%%% KINEMATICS %%%%%%%%%%%%%%

%parameterization of a circle
       x=r*cos(beta);
       y=r*sin(beta);

       
       eta=atan2(y,x);  %calculates angle to end effector from given points
       lhat=sqrt(x^2+y^2); %calculates the distance to the end effector from the origin
      
       
       %LEFT SERIAL MANIPULATOR
       phi1=atan2(y,(x+d/2)); %calculates the angle of the line from the first revolute joint to the end effector
       lsquaredleft= (lhat*sin(eta))^2+(lhat*cos(eta)+d/2)^2; %calculates the distance from the first revolute joint to the end effector 
       beta1=-acos((lsquaredleft-l1^2-l2^2)/(2*l1*l2)); %calculates the value of the angle q2
       psi1=asin(-l2/sqrt(lsquaredleft)*sin(beta1)); %calculates the angle between the phi and q1
       if(psi1>pi/2) psi1=psi1-pi/2; end %this if statement ensures that the angle of psi1 is the correct one as the asin function is odd
       th1=phi1+psi1; %q1
       th2=beta1; %q2
       
       %RIGHT SERIAL MANIPULATOR
       % below is the same structure as above
       phi2=atan2(y,(x-d/2));
       lsquaredright=(lhat*sin(eta))^2+(lhat*cos(eta)-d/2)^2; 
       beta2=acos((lsquaredright-l3^2-l4^2)/(2*l3*l4));
       psi2= asin(l4/sqrt(lsquaredright)*sin(beta2));
       if(psi2>pi/2) psi2=psi2-pi/2; end
       th3=phi2-psi2; 
       th4=beta2;

%%%%%%%%%%%%   %FORWARD KINEMATICS
x1=l1*cos(th1)-d/2; %position of the 2nd revolute joint of the left serial manipulator
y1=l1*sin(th1);
x2=l3*cos(th3)+d/2;%position of the 2nd revolute joint of the right serial manipulator
y2=l3*sin(th3);
xop1=x1+l2*cos(th1+th2); %position of the end effector by way of the left serial manipulator
yop1=y1+l2*sin(th1+th2);
xop2=x2+l4*cos(th3+th4); %position of the end effector by way of the right serial manipulator
yop2=y2+l4*sin(th3+th4);

%1x2 vector forms of the above forward kinematics
xop1_vec=[xop1;yop1]; 
xop2_vec=[xop2;yop2];
x1_vec=[x1;y1];
x2_vec=[x2;y2] ;

magn=norm(xop2_vec-xop1_vec,2); %the distance between the left and right operational positions

if(magn<1e-3) %this check makes sure that the distance adheres to the constraints that the left and right operation points be the same


%%%%

  %%%% Mass matrix of the two decoupled manipulators using inputs (t1,t2,t3,and t4) 

  %left manipulator inertia matrix M=M(th1,th2) 
  M11=m1*l1^2/4+m2*l1^2+m2*l2^2/4+m2*l1*l2*cos(th2)+IG1+IG2;
  M12=m2*(l1*l2/2*cos(th2)+l2^2/4)+IG2;
  M21=m2*(l2^2/4+l1*l2/2*cos(th2))+IG2;
  M22=m2*l2^2/4+IG2;

  %right manipulator inertia matrix M=M(th3,th4)
  M33=m3*l3^2/4+m4*l3^2+m4*l4^2/4+m4*l3*l4*cos(th4)+IG3+IG4;
  M34=m4*(l3*l4/2*cos(th4)+l4^2/4)+IG4;
  M43=m4*(l4^2/4+l3*l4/2*cos(th4))+IG4;
  M44=m4*l4^2/4+IG4;



  M=[M11,M12,0,0;M21,M22,0,0;0,0,M33,M34;0,0,M43,M44;];

%left jacobian z1=J1q1 where z and q are at the velocity level
  J1=[-l1*sin(th1)-l2*sin(th1+th2), -l2*sin(th1+th2); l1*cos(th1)+l2*cos(th1+th2), l2*cos(th1+th2)];
%right jacobian z2=J2q2
  J2=[-l3*sin(th3)-l4*sin(th3+th4), -l4*sin(th3+th4); l3*cos(th3)+l4*cos(th3+th4), l4*cos(th3+th4)];

%qp=D^-1*C*qa where qp are the passive joints and qa are the active
  C=[J1(:,1) -J2(:,1)];
  D=[-J1(:,2) J2(:,2)];


 Dhat=D\C; %solve for D^-1*C

 A=[1,0;Dhat(1,:);0,1;Dhat(2,:);]; %q=A*qa where q= [q1,q2,q3,q4]' and qa= [q1, q3]

 %Express the Jacobian of the device in the operational space with only qa
 %as the joint space coordinates
 J=J1*[1,0;Dhat(1,:)];  % this comes from the equation zdot=J1*q_left where q_left = F*q where F selects the q1 and q2 vars from q and q= Aqa
 %JO=J2*[0,1;Dhat(2,:)];

 
 %Build the effective mass matrix (J*M^-1*J')^-1
MStar=(A'*M*A)\eye(2,2);
EMM=J*MStar;
EMM=EMM*J';
EMM=EMM\eye(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% ANIMATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Diagonalize the effective mass matrix to get the principle directions and
%the eigenvalues 

 [V,L]=eig(EMM);

% V=eye(2);
% L=eye(2);


%%%%%%%%%%%%%%%%%%%%%%%

%semiminor and semimajor axes of ellipse 
 a=scale*L(1,1);
 b=scale*L(2,2);
 
%condition number i.e. the ratio of the semimajor axis over the semiminor
%axis. This is a measure of the isotropy of the effective mass  
 CN=cond(L); 

 %Rotation matrix to rotate an drawn ellispe to align with the eigenvectors
  R=V ;
  
  %below was a check of orthogonality
%   dot(V(:,1),V(:,2));
%   V*V';

  %%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 X= drawEllipse(a,b,R,xop1_vec);
 Y= drawDevice(x1_vec, xop1_vec,xop2_vec, x2_vec,d);


%Check to ensure that the configuration is a real one the device can
%acheive
if(imag(X(:,1))==0)
    
%PLOT THE DEVICE 

 subplot 121
  hold on;  
  axis([-ltot ltot -ltot ltot]) %sets the axes to be displayed 
  daspect([1 1 1]) %makes sure it's square
  cla(s1); %clears the subplot 121 each time so that the device is animated
plot(X(:,1), X(:,2), 'b-') %plot the mass ellipse 
text(-.1,-.23,['P op = (', num2str(xop1),',',num2str(yop1),')'],'FontSize',9) %display the operational point
plot(Y(:,1), Y(:,2), 'r--o') %plot the device by drawing both the left and right device. It is important to do this to ensure that the device is in a real configuration
plot([-d/2,d/2],[0,0], 'g--') % plot the line which represents the distance between the pair of first revolute joints. 
plot(x,y,'m--x', 'markersize', 15) %plots the operation point
plot(0,0,'o', 'markersize', 5, 'markerfacecolor','c'); %plots the origin

%WORKSPACE PLOT

subplot 122
  
  hold on %makes sure the workspace animation is held from the last so as to represent a built up image. 
  axis([-ltot ltot -ltot ltot]) 
  daspect([1 1 1])
 
  if( CN>CNMAX) CN=CNMAX; end; %low pass filter for values of the condition number
  
  %COLOR MAPPING CHECK. This finds the value of the condition number
  %closest to the color map developed during the initialization and allows
  %for the representative collor to be picked
  
  j=50; 
  errorold=CNMAX; 
for i=1:50
    errornew=abs(CN-map(i,1)); 
    if (errornew<errorold) errorold=errornew; 
    else
        j=i-1;
        break 
    end 
end
  

col=map(j,2:4);


%the code snippet below scales the size of the colored marker depending on
%how large you drag the window open. 
f=gcf; 
winsz=get(f, 'Position'); %finds the size of the current figure 
winsz=norm(winsz(3:4),2)/2.138636013911671e+03; %finds the size of the diagonal element of the open window
mkrsz=4+r/.2*winsz*18; %linear scaling

%plot the operational point and use the symmetrry about the x axis to
%reduce computational cost. 
plot(x,y,'s','markersize',mkrsz,'markerfacecolor',col,'markeredgecolor',col)
plot(x,-y,'s','markersize',mkrsz,'markerfacecolor',col,'markeredgecolor',col)

% waitforbuttonpress;
 pause(.0001)
else
    
%if the configuration was not possible plot a small black dot to for visual
%aid
subplot 122
hold on
axis([-ltot ltot -ltot ltot])
daspect([1 1 1])
  
col=[0 0 0];   

%plot the black dots mentioned above
plot(x,y,'--x','markersize',2,'markerfacecolor',col,'markeredgecolor',col)
plot(x,-y,'--x','markersize',2,'markerfacecolor',col,'markeredgecolor',col)
%waitforbuttonpress;
  pause(.0001)

end %end of configuration check
end % end of operational point distance check



    end %end of radial for loop
 end %end of angular for loop
 
 % plot a colorbar to show the values of the condition numbers seen on the
 % workspace plot. These colors show the regions of relative isotropy
subplot 122    
   hold off
   colormap(map(:, 2:4)); 
   colorbar('EastOutside','YTick', linspace(1,50,5),'YTickLabel', {num2str(CNMAX/5), num2str(2*CNMAX/5), num2str(3*CNMAX/5), num2str(4*CNMAX/5), num2str(5*CNMAX/5)}); 
hold on
 clc

end



function X=drawEllipse(a,b,R,x)

%drawEllipse(semi major axis, semi minor axis, angle of rotation, x(1)= x,
%x(2)=y)

   X= zeros(36,2); %draw the ellipse with 36 points !arbitrary

    alpha = linspace(0, 360, 36)' .* (pi / 180); % create a linear space for the parametrization angle (i.e. x=acos(alpha) and y=bsin(alpha))
    sinalpha = sin(alpha);
    cosalpha = cos(alpha);

    for i=1:36
   y= x+R*[a*cosalpha(i), b*sinalpha(i)]'; %rotates the ksi and eta coordinates of the ellipse by R from the associated directions of the eigenvectors to the cartesian space of the device

   %return the values of the parameterized ellipse in cartesian space
   X(i,1)=y(1); 
   X(i,2)=y(2);
    end
end

function X=drawDevice(x1_vec,xop1_vec,xop2_vec,x2_vec,d)

%draw the device 

   X= zeros(6,2);

 X(1,:)=[-d/2,0]; %first joint of the left serial manipulator
 X(2,:)=x1_vec'; %second joint of the left serial manipulator
 X(3,:)=xop1_vec'; %operation point of the left serial manipulator
 X(4,:)=xop2_vec';  %operation point of the right serial manipulator
 X(5,:)=x2_vec';%second joint of the right serial manipulator
 X(6,:)=[d/2,0];%first joint of the right serial manipulator





end
