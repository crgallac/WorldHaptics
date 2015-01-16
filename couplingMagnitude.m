function couplingMagnitude()


%parameterize the entire workspace of the device for the direction of
%uncoupling 

%for tomorrow we need to figure out why there is a lack of symmetry in this
%code i believe it comes down to my poor use of ambiguous angles perhaps?

%try drawing the location of xop from the point of x2


%Initialize windows and object handles
% clf;
% s1=subplot(1,2,1);
% % 
% %Creating a color map for the workspace analysis
% R1=zeros(1,25); 
% G1=linspace(0,1,25);
% B1=linspace(1,0,25); 
% 
% R2=linspace(0,1,25); 
% G2=linspace(1,0,25); 
% B2=zeros(1,25); 
% 
% CNA=linspace(1,10,50);%%% is 10 the appropriate bound? 
% CNMAX=2; 
% 
% %COLORMAPPING
% map=[CNA(1:25)' R1' G1' B1'; CNA(26:50)' R2' G2' B2'];


% hsv(128)

%Dynamic Parameters 
 l1=.147; %link 1 of the left serial manipulator
 l2=.199;%link 2 of the left serial manipulator
 l3=.147;%link 1 of the right serial manipulator
 l4=.199;%link 2 of the right serial manipulator
 d=0.0; %distance between first revolute joints
 m1= .3046; %mass of first link of the left serial manipulator
 m2=.047;%mass of second link of the left serial manipulator
 m3=.3046;%mass of first link of the right serial manipulator
 m4=.047;%mass of second link of the right serial manipulator
 IG1= m1*l1^2/12; %moment of inertia about l1 center of mass
 IG2=m2*l2^2/12;%moment of inertia about l2 center of mass
 IG3=IG1;%moment of inertia about l3 center of mass
 IG4=IG2;%moment of inertia about l4 center of mass

 Q=eye(2); 
 phi=0; 
 
 maxCoup=0; 
 
  l=[l1,l2,l3,l4]; 
 m=[m1,m2,m3,m4]; 

 i=1; 
 j=1; 
 
 xvec=[0,0];
 uvec=[0,0];
 
%additional Parameters
scale=.1; %scaling of the effective mass matrix ellipse
int=40; %number of intervals used for the for loops
ll=l1+l2; 
lr=l3+l4; 

if(l1>lr)
rtot=ll;
else
rtot=lr; 
end

[X,Y]= meshgrid(-.4:.01:.4, 0:.01:.4); 

ltot=rtot+rtot/3; 
%for loop for parameterization of a circle
%  for beta=0:(pi/int):pi; 
%     for r=0.01:((ltot-1e-4)-.01)/int:ltot;
 for x=-.4:.01:.4; 
     j=1; 
    for y=0:.01:.4;


%%%%%%%%%%%% KINEMATICS %%%%%%%%%%%%%%

%parameterization of a circle
%        x=r*cos(beta);
%        y=r*sin(beta);

       
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

  th=[th1 th2 th3 th4];
  th_dot=[0 0 0 0]; 

if(magn<1e-3) %this check makes sure that the distance adheres to the constraints that the left and right operation points be the same

    xvec(i,:)=[x,y]; 

%%%% DYNAMICS

  MatOut= matOut(m,l,th, th_dot);
  
    M=MatOut(:,1:4); 
    c=MatOut(:,5);

    Q=[cos(phi) sin(phi); -sin(phi) cos(phi);]; %The matrix rotating the acceleration into the operational space
    
    G=[1 0; 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)) 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)); 0 1; 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * cos(th1 + th2) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (l1 * cos(th1) + l2 * cos(th1 + th2)) 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * cos(th1 + th2) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (-l3 * cos(th3) - l4 * cos(th3 + th4));];
%     Gdot= [0 0; -0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (l1 * cos(th1) + l2 * cos(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot)) -0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * cos(th3) * th3_dot + l4 * cos(th3 + th4) * (th3_dot + th4_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l3 * sin(th3) * th3_dot + l4 * sin(th3 + th4) * (th3_dot + th4_dot)); 0 0; -0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l2 * cos(th1 + th2) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (th1_dot + th2_dot) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * cos(th1 + th2) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l2 * sin(th1 + th2) * (l1 * cos(th1) + l2 * cos(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * (l1 * cos(th1) + l2 * cos(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot)) -0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l2 * cos(th1 + th2) * (l3 * sin(th3) + l4 * sin(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (th1_dot + th2_dot) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * cos(th1 + th2) * (l3 * cos(th3) * th3_dot + l4 * cos(th3 + th4) * (th3_dot + th4_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l2 * sin(th1 + th2) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (l3 * sin(th3) * th3_dot + l4 * sin(th3 + th4) * (th3_dot + th4_dot));];

    R= [cos(phi) * (-l1 * sin(th1) - l2 * sin(th1 + th2) - l2 * sin(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)))) + sin(phi) * (l1 * cos(th1) + l2 * cos(th1 + th2) + l2 * cos(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)))) -cos(phi) * l2 * sin(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))) + sin(phi) * l2 * cos(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))); -sin(phi) * (-l1 * sin(th1) - l2 * sin(th1 + th2) - l2 * sin(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)))) + cos(phi) * (l1 * cos(th1) + l2 * cos(th1 + th2) + l2 * cos(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)))) sin(phi) * l2 * sin(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))) + cos(phi) * l2 * cos(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)));];
%     Rdot  = getRdot(m, l, th, th_dot, phi);
    R_inv=R\eye(2); 
%     Rdot_inv=R_inv*Rdot*R_inv; 

       %Dynamics in task space
   
     H1= Q'*R_inv'*G'*M*G*R_inv*Q\eye(2);
     EMM1=Q'*R_inv'*G'*M*G*R_inv*Q; %%%For plotting purposes   %%%%%UNITS ARE WIERD
%      Fa_in_v1=Q'*(R_inv'*G'*M*G*Rdot_inv+R_inv'*G'*M*Gdot*R_inv)*Q;
%      Fa_c1= Q'*R_inv'*G'*c; 
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%% ANIMATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Diagonalize the effective mass matrix to get the principle directions and
%the eigenvalues 

 [V,L]=eig(EMM1);

% V=eye(2);
% L=eye(2);


%%%%%%%%%%%%%%%%%%%%%%%

%semiminor and semimajor axes of ellipse 
 a=scale*L(1,1);
 b=scale*L(2,2);
 
%condition number i.e. the ratio of the semimajor axis over the semiminor
%axis. This is a measure of the isotropy of the effective mass  

% if(EMM1(1,2)>maxCoup)
%     maxCoup=EMM1(1,2); 
% end

%  CN=abs(EMM1(1,2)); 
 
[U, S, V]= svd(EMM1); 

 mMax(i,j)=max(max(S));
 
 mMax(i,j)
 
 condN(i,j)=cond(S);

 

else
   
    mMax(i,j)=NaN; 

end % end of operational point distance check




j=j+1; 

    end %end of radial for loop
    i=i+1; 
 end %end of angular for loop
 

figure 

mMax

handle1=contourf(X,Y, condN'); 

  set('AlphaData',~isnan(condN'))
condN

end


% 
% function X=drawEllipse(a,b,R,x)
% 
% %drawEllipse(semi major axis, semi minor axis, angle of rotation, x(1)= x,
% %x(2)=y)
% 
%    X= zeros(36,2); %draw the ellipse with 36 points !arbitrary
% 
%     alpha = linspace(0, 360, 36)' .* (pi / 180); % create a linear space for the parametrization angle (i.e. x=acos(alpha) and y=bsin(alpha))
%     sinalpha = sin(alpha);
%     cosalpha = cos(alpha);
% 
%     for i=1:36
%    y= x+R*[a*cosalpha(i), b*sinalpha(i)]'; %rotates the ksi and eta coordinates of the ellipse by R from the associated directions of the eigenvectors to the cartesian space of the device
% 
%    %return the values of the parameterized ellipse in cartesian space
%    X(i,1)=y(1); 
%    X(i,2)=y(2);
%     end
% end
% 
% function X=drawDevice(x1_vec,xop1_vec,xop2_vec,x2_vec,d)
% 
% %draw the device 
% 
%    X= zeros(6,2);
% 
%  X(1,:)=[-d/2,0]; %first joint of the left serial manipulator
%  X(2,:)=x1_vec'; %second joint of the left serial manipulator
%  X(3,:)=xop1_vec'; %operation point of the left serial manipulator
%  X(4,:)=xop2_vec';  %operation point of the right serial manipulator
%  X(5,:)=x2_vec';%second joint of the right serial manipulator
%  X(6,:)=[d/2,0];%first joint of the right serial manipulator
% 
% 
% 
% 
% 
% end



function Mat_out= matOut(m,L,theta,v)

 IG1= m(1)*L(1)^2/12; %moment of inertia about l1 center of mass
 IG2=m(2)*L(2)^2/12;%moment of inertia about l2 center of mass
 IG3=IG1;%moment of inertia about l3 center of mass
 IG4=IG2;%moment of inertia about l4 center of mass
 
tau=[0 0 0 0]; 

t1 = cos(theta(2));
t2 = m(2) * L(1);
t3 = m(2) * L(2) ^ 2;
t4 = 0.1e1 / 0.4e1;
t5 = m(2) * L(2) * (t4 * L(2) + L(1) * t1 / 0.2e1) + IG2;
t6 = theta(1) + theta(2);
t7 = L(1) * sin(theta(1));
t8 = L(2) * sin(t6);
t9 = L(1) * cos(theta(1));
t6 = L(2) * cos(t6);
t10 = theta(3) + theta(4);
t11 = L(3) * cos(theta(3));
t12 = L(4) * cos(t10);
t13 = t2 * L(2) * sin(theta(2)) / 0.2e1;
t14 = L(3) * sin(theta(3));
t10 = L(4) * sin(t10);
t15 = v(1) ^ 2;
t16 = cos(theta(4));
t17 = m(4) * L(3);
t18 = m(4) * L(4) ^ 2;
t19 = m(4) * L(4) * (t4 * L(4) + L(3) * t16 / 0.2e1) + IG2;
t20 = v(1) + v(2);
t21 = v(3) + v(4);
t22 = v(3) ^ 2;
t21 = t21 ^ 2;
t20 = t20 ^ 2;
t23 = t17 * L(4) * sin(theta(4)) / 0.2e1;
Mat_out = [t4 * (m(1) * L(1) ^ 2 + t3) + t2 * (t1 * L(2) + L(1)) + IG1 + IG2 t5 0 0 -t13 * v(2) * (2 * v(1) + v(2)) tau(1) -t7 - t8 t9 + t6 -t11 - t12 + t9 + t6; t5 t3 * t4 + IG2 0 0 t13 * t15 0 -t8 t6 -t10 - t14 + t7 + t8; 0 0 t4 * (m(3) * L(3) ^ 2 + t18) + t17 * (t16 * L(4) + L(3)) + IG1 + IG2 t19 -t23 * v(4) * (2 * v(3) + v(4)) tau(3) t10 + t14 -t11 - t12 t11 * t22 + t12 * t21 - t9 * t15 - t6 * t20; 0 0 t19 t18 * t4 + IG2 t23 * t22 0 t10 -t12 t10 * t21 + t14 * t22 - t7 * t15 - t8 * t20;];


end




