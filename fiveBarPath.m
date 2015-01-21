function fiveBarPath

%Dynamic Parameters 
global l1 l2 l3 l4 d m1 m2 m3 m4 IG1 IG2 IG3 IG4
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
 
 
 l=[l1,l2,l3,l4]; 
 m=[m1,m2,m3,m4]; 


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


%Path parameterization
p_length=.15; %meter
dur=1; % second ----> this will be made into a variable for later models. 

a=4*p_length/(dur^2);  % this is the acceleration in the operational space assuming constant accel then decel to stop on the point
acc=[1 0]; %acceleration . 
v0=[0 0]; %velocity 
z0=[.15 0]; %position
phi=0; 

Q=[cos(phi) sin(phi); -sin(phi) cos(phi);]; %The matrix rotating the acceleration into the operational space

acc=(Q'*acc')'
v0=(Q'*v0')'; 
z0=(Q'*z0')'; 

dt=.01; %milliseconds
i=1; 

v=zeros(dur/dt,2);%initialize arrays
z=zeros(dur/dt,2); 

rdot=zeros(dur/dt,2);
r=zeros(dur/dt,2); 

th0=[0 0 0 0]; 

 v(i,:)=v0; 
    z(i,:)=z0; 
    
 rdot(i,:)=(Q'*v(i,:)')' ;%this rotates the operational space coordingates into the cartesian coords
 r(i,:)= (Q'*z(i,:)')'; 
  

 for t=0:dt:dur
        
    %%%%% KINEMATICS
    
    x=z0(1); 
    y=z0(2); 
    
           eta=atan2(y,x);  %calculates angle to end effector from given points
       lhat=sqrt(x^2+y^2); %calculates the distance to the end effector from the origin
      
   %%%%% INVERSE KINEMATICS
       
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

       th=[th1 th2 th3 th4];
       theta=th; 
       
       if(t==0)
       th0=th; 
       end
       
       th_dot= (th-th0)/dt;
     
       th1_dot=th_dot(1); 
       th2_dot=th_dot(2);
       th3_dot=th_dot(3); 
       th4_dot=th_dot(4); 
       
       th0=th; 
       
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


    if(t==dur/2) 
        acc=-acc; 
    end
    
%%%%%%Device Dynamics
    
  MatOut= matOut(m,l,th, th_dot);
  
    M=MatOut(:,1:4); 
    c=MatOut(:,5);

    Q=[cos(phi) sin(phi); -sin(phi) cos(phi);]; %The matrix rotating the acceleration into the operational space
    
    G=[1 0; 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)) 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)); 0 1; 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * cos(th1 + th2) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (l1 * cos(th1) + l2 * cos(th1 + th2)) 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * cos(th1 + th2) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (-l3 * cos(th3) - l4 * cos(th3 + th4));];
    Gdot= [0 0; -0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (l1 * cos(th1) + l2 * cos(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot)) -0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * cos(th3) * th3_dot + l4 * cos(th3 + th4) * (th3_dot + th4_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l3 * sin(th3) * th3_dot + l4 * sin(th3 + th4) * (th3_dot + th4_dot)); 0 0; -0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l2 * cos(th1 + th2) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (th1_dot + th2_dot) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * cos(th1 + th2) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l2 * sin(th1 + th2) * (l1 * cos(th1) + l2 * cos(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * (l1 * cos(th1) + l2 * cos(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot)) -0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l2 * cos(th1 + th2) * (l3 * sin(th3) + l4 * sin(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (th1_dot + th2_dot) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * cos(th1 + th2) * (l3 * cos(th3) * th3_dot + l4 * cos(th3 + th4) * (th3_dot + th4_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l2 * sin(th1 + th2) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l2 * sin(th1 + th2) * (l3 * sin(th3) * th3_dot + l4 * sin(th3 + th4) * (th3_dot + th4_dot));];

    R= [cos(phi) * (-l1 * sin(th1) - l2 * sin(th1 + th2) - l2 * sin(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)))) + sin(phi) * (l1 * cos(th1) + l2 * cos(th1 + th2) + l2 * cos(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)))) -cos(phi) * l2 * sin(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))) + sin(phi) * l2 * cos(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))); -sin(phi) * (-l1 * sin(th1) - l2 * sin(th1 + th2) - l2 * sin(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)))) + cos(phi) * (l1 * cos(th1) + l2 * cos(th1 + th2) + l2 * cos(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)))) sin(phi) * l2 * sin(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))) + cos(phi) * l2 * cos(th1 + th2) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)));];
    Rdot  = getRdot(m, l, th, th_dot, phi);
    R_inv=R\eye(2); 
    Rdot_inv=R_inv*Rdot*R_inv; 
    
    H=R_inv'*G'*M*G*R_inv\eye(2);
    EMM=R_inv'*G'*M*G*R_inv; %%%For plotting purposes   %%%%%UNITS ARE WIERD
    
    Fa_in_v= (R_inv'*G'*M*G*Rdot_inv+R_inv'*G'*M*Gdot*R_inv)*Q;
    
   Fa_c= R_inv'*G'*c;
   

    
    %%%%%%Operational Space trajectory
    
        %in cartesian space coordinates
    v(i,1)=acc(1)*dt+v0(1); 
    z(i,1)=acc(1)/2*dt^2+v0(1)*dt+z0(1); 
    
    
   
        
  
 
    %%%%%%Admissible Space trajectory (in cartesian frame)
    
     acc1= H*(-1)*(Fa_in_v*z0'+Fa_c);
      acc1=Q'*acc1; 
     acc(2)=acc1(2);      
     v(i,2)=acc(2)*dt+v0(2); 
     z(i,2)=acc(2)/2*dt^2+v0(2)*dt+z0(2); 
     
  %%% in operational frame coordinates
      rdot(i,:)= (Q'*v(i,:)')';
    r(i,:)=(Q'*z(i,:)')';
     
%           waitforbuttonpress
    %%%%

    v0=v(i,:); 
    z0=z(i,:); 
     

  %%%%%%%%

%   [EMM1, MComparison, A, MStar]= effMassMatrix(th1,th2,th3,th4); 
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

 Y= drawDevice(x1_vec, xop1_vec,xop2_vec, x2_vec,d);
 X= drawEllipse(a,b,R,xop1_vec);


%Check to ensure that the configuration is a real one the device can
%acheive
 if(imag(X(:,1))==0)
    
%PLOT THE DEVICE 

% % % %  subplot 121
  hold on;  
  axis([-ltot ltot -ltot ltot]) %sets the axes to be displayed 
  daspect([1 1 1]) %makes sure it's square
  cla
%   cla(s1); %clears the subplot 121 each time so that the device is animated
plot(X(:,1), X(:,2), 'b-') %plot the mass ellipse 
text(-.1,-.23,['P op = (', num2str(xop1),',',num2str(yop1),')'],'FontSize',9) %display the operational point
plot(Y(:,1), Y(:,2), 'r--o') %plot the device by drawing both the left and right device. It is important to do this to ensure that the device is in a real configuration
plot([-d/2,d/2],[0,0], 'g--') % plot the line which represents the distance between the pair of first revolute joints. 
plot(x,y,'m--x', 'markersize', 15) %plots the operation point
plot(0,0,'o', 'markersize', 5, 'markerfacecolor','c'); %plots the origin    
 
% % % end
% % % % end %this is the end of the iff statment to check for 
%  waitforbuttonpress;


   
    
end

end
 i=i+1;
 pause(.1);
%  waitforbuttonpress; 
 end
 
 
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

function [EMM, M,A, MStar]= effMassMatrix(th1,th2,th3,th4)

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
  

 invD1= [cos(th3 + th4) / l2 / (sin(th1) * cos(th3 + th4) + sin(th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) sin(th3 + th4) / l2 / (sin(th1) * cos(th3 + th4) + sin(th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)); cos(th1 + th2) / l4 / (sin(th1) * cos(th3 + th4) + sin(th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) sin(th1 + th2) / l4 / (sin(th1) * cos(th3 + th4) + sin(th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2));]


 Dhat=invD1*C; %solve for D^-1*C
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


end

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

function Rdot = getRdot(m, L, theta, v, phi)

m1=m(1);
m2=m(2); 
m3=m(3); 
m4=m(4); 
l1=L(1); 
l2=L(2); 
l3=L(3);
l4=L(4); 
th1=theta(1); 
th2=theta(2); 
th3=theta(3); 
th4=theta(4); 
th1_dot=v(1); 
th2_dot=v(2); 
th3_dot=v(3); 
th4_dot=v(4); 

Rdot= [cos(phi) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2))) - l2 * sin(th1 + th2) * (-0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (l1 * cos(th1) + l2 * cos(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot)))) + sin(phi) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot) - l2 * sin(th1 + th2) * (th1_dot + th2_dot) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2))) + l2 * cos(th1 + th2) * (-0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (l1 * cos(th1) + l2 * cos(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot)))) -cos(phi) * l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))) - cos(phi) * l2 * sin(th1 + th2) * (-0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * cos(th3) * th3_dot + l4 * cos(th3 + th4) * (th3_dot + th4_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l3 * sin(th3) * th3_dot + l4 * sin(th3 + th4) * (th3_dot + th4_dot))) - sin(phi) * l2 * sin(th1 + th2) * (th1_dot + th2_dot) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))) + sin(phi) * l2 * cos(th1 + th2) * (-0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * cos(th3) * th3_dot + l4 * cos(th3 + th4) * (th3_dot + th4_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l3 * sin(th3) * th3_dot + l4 * sin(th3 + th4) * (th3_dot + th4_dot))); -sin(phi) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2))) - l2 * sin(th1 + th2) * (-0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (l1 * cos(th1) + l2 * cos(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot)))) + cos(phi) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot) - l2 * sin(th1 + th2) * (th1_dot + th2_dot) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2))) + l2 * cos(th1 + th2) * (-0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (-l1 * sin(th1) - l2 * sin(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (-l1 * cos(th1) * th1_dot - l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (l1 * cos(th1) + l2 * cos(th1 + th2)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (l1 * cos(th1) + l2 * cos(th1 + th2)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l1 * sin(th1) * th1_dot - l2 * sin(th1 + th2) * (th1_dot + th2_dot)))) sin(phi) * l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))) + sin(phi) * l2 * sin(th1 + th2) * (-0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * cos(th3) * th3_dot + l4 * cos(th3 + th4) * (th3_dot + th4_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l3 * sin(th3) * th3_dot + l4 * sin(th3 + th4) * (th3_dot + th4_dot))) - cos(phi) * l2 * sin(th1 + th2) * (th1_dot + th2_dot) * (0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4))) + cos(phi) * l2 * cos(th1 + th2) * (-0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * cos(th3 + th4) * (l3 * sin(th3) + l4 * sin(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) * (l3 * sin(th3) + l4 * sin(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (l3 * cos(th3) * th3_dot + l4 * cos(th3 + th4) * (th3_dot + th4_dot)) - 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) ^ 2 * l4 * sin(th3 + th4) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) * (l2 * (cos(th1) * th1_dot + cos(th2) * th2_dot) * l4 * cos(th3 + th4) - l2 * sin(th1 + th2) * l4 * sin(th3 + th4) * (th3_dot + th4_dot) - l4 * cos(th3 + th4) * (th3_dot + th4_dot) * l2 * cos(th1 + th2) + l4 * sin(th3 + th4) * l2 * sin(th1 + th2) * (th1_dot + th2_dot)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * cos(th3 + th4) * (th3_dot + th4_dot) * (-l3 * cos(th3) - l4 * cos(th3 + th4)) + 0.1e1 / (l2 * sin(th1 + th2) * l4 * cos(th3 + th4) - l4 * sin(th3 + th4) * l2 * cos(th1 + th2)) * l4 * sin(th3 + th4) * (l3 * sin(th3) * th3_dot + l4 * sin(th3 + th4) * (th3_dot + th4_dot)));];


end
