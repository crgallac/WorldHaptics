function fiveBarMechanismMassMatrix 

%Parameters

l1=.1; 
l2=.1; 
m1= .05; 
m2=.05; 
IG1= m1*l1^2/12; 
IG2=m2*l2^2/12; 
scale=10; 
int=10; 

clf;

for th3=0:pi/int:pi-pi/int; 
   for th1=th3+pi/int:pi/(3*int): th3+2*pi;
    
%         th2= -asin(l1/l2*sin((th1-th3)/2))-((th1-th3)/2);
%         th4= pi/2-th2; 
        
x1=l1*cos(th1);
y1=l1*sin(th1);
x2=l1*cos(th3); 
y2=l1*sin(th3); 

       thstar=asin(l1/l2*sin((th1-th3)/2)); 
    
       if(y1<-y2)
       thstar=2*pi-thstar; 
       end
       
      th2=-2*thstar; 
      th4=2*thstar; 


xop=x1+l2*cos(th1+th2); 
yop=y1+l2*sin(th1+th2); 

x_vec=[xop;yop];
x1_vec=[x1;y1]; 
x2_vec=[x2;y2]; 
        
     H = [m1 * l1 ^ 2 + IG1 + m2 * (l1 ^ 2 + l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th2)) + IG2 - (m2 * (l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th2) / 0.2e1) + IG2) * (sin(th1) * cos(th3 + th4) * l1 - cos(th1) * sin(th3 + th4) * l1 + sin(th1 + th2) * cos(th3 + th4) * l2 - sin(th3 + th4) * cos(th1 + th2) * l2) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) - (sin(th1) * cos(th3 + th4) * l1 - cos(th1) * sin(th3 + th4) * l1 + sin(th1 + th2) * cos(th3 + th4) * l2 - sin(th3 + th4) * cos(th1 + th2) * l2) * (m2 * (l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th2) / 0.2e1) + IG2 - (l2 ^ 2 * m2 / 0.4e1 + IG2) * (sin(th1) * cos(th3 + th4) * l1 - cos(th1) * sin(th3 + th4) * l1 + sin(th1 + th2) * cos(th3 + th4) * l2 - sin(th3 + th4) * cos(th1 + th2) * l2) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2))) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) + l1 ^ 2 * (sin(th1) * cos(th1 + th2) - cos(th1) * sin(th1 + th2)) ^ 2 / l2 ^ 2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) ^ 2 * (l2 ^ 2 * m2 / 0.4e1 + IG2) (m2 * (l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th2) / 0.2e1) + IG2) * l1 * (sin(th3) * cos(th3 + th4) - cos(th3) * sin(th3 + th4)) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) - (sin(th1) * cos(th3 + th4) * l1 - cos(th1) * sin(th3 + th4) * l1 + sin(th1 + th2) * cos(th3 + th4) * l2 - sin(th3 + th4) * cos(th1 + th2) * l2) / l2 ^ 2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) ^ 2 * (l2 ^ 2 * m2 / 0.4e1 + IG2) * l1 * (sin(th3) * cos(th3 + th4) - cos(th3) * sin(th3 + th4)) - l1 * (sin(th1) * cos(th1 + th2) - cos(th1) * sin(th1 + th2)) * (m2 * (l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th4) / 0.2e1) + IG2 + (l2 ^ 2 * m2 / 0.4e1 + IG2) * (sin(th3) * cos(th1 + th2) * l1 - cos(th3) * sin(th1 + th2) * l1 - sin(th1 + th2) * cos(th3 + th4) * l2 + sin(th3 + th4) * cos(th1 + th2) * l2) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2))) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)); l1 * (sin(th3) * cos(th3 + th4) - cos(th3) * sin(th3 + th4)) * (m2 * (l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th2) / 0.2e1) + IG2 - (l2 ^ 2 * m2 / 0.4e1 + IG2) * (sin(th1) * cos(th3 + th4) * l1 - cos(th1) * sin(th3 + th4) * l1 + sin(th1 + th2) * cos(th3 + th4) * l2 - sin(th3 + th4) * cos(th1 + th2) * l2) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2))) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) - (m2 * (l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th4) / 0.2e1) + IG2) * l1 * (sin(th1) * cos(th1 + th2) - cos(th1) * sin(th1 + th2)) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) - (sin(th3) * cos(th1 + th2) * l1 - cos(th3) * sin(th1 + th2) * l1 - sin(th1 + th2) * cos(th3 + th4) * l2 + sin(th3 + th4) * cos(th1 + th2) * l2) / l2 ^ 2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) ^ 2 * (l2 ^ 2 * m2 / 0.4e1 + IG2) * l1 * (sin(th1) * cos(th1 + th2) - cos(th1) * sin(th1 + th2)) l1 ^ 2 * (sin(th3) * cos(th3 + th4) - cos(th3) * sin(th3 + th4)) ^ 2 / l2 ^ 2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) ^ 2 * (l2 ^ 2 * m2 / 0.4e1 + IG2) + m1 * l1 ^ 2 / 0.4e1 + IG1 + m2 * (l1 ^ 2 + l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th4)) + IG2 + (m2 * (l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th4) / 0.2e1) + IG2) * (sin(th3) * cos(th1 + th2) * l1 - cos(th3) * sin(th1 + th2) * l1 - sin(th1 + th2) * cos(th3 + th4) * l2 + sin(th3 + th4) * cos(th1 + th2) * l2) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2)) + (sin(th3) * cos(th1 + th2) * l1 - cos(th3) * sin(th1 + th2) * l1 - sin(th1 + th2) * cos(th3 + th4) * l2 + sin(th3 + th4) * cos(th1 + th2) * l2) * (m2 * (l2 ^ 2 / 0.4e1 + l1 * l2 * cos(th4) / 0.2e1) + IG2 + (l2 ^ 2 * m2 / 0.4e1 + IG2) * (sin(th3) * cos(th1 + th2) * l1 - cos(th3) * sin(th1 + th2) * l1 - sin(th1 + th2) * cos(th3 + th4) * l2 + sin(th3 + th4) * cos(th1 + th2) * l2) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2))) / l2 / (sin(th1 + th2) * cos(th3 + th4) - sin(th3 + th4) * cos(th1 + th2));];
     
     th1
     th3
     th2
     th4
    % H   
     
     [V,D]=eig(H); 
     
%       V
%       D
     
     cphi= dot(V(1:2,1),[1,0]); 
       
     phi=acos(cphi); 
     
     R=V'*[1,0; 0,1;];
    
     
   a=scale*D(1,1);
   b=scale*D(2,2); 
   
   
   if(rank(H)>1)
 X= drawEllipse(a,b,R,x_vec);    
 Y= drawDevice(x1_vec, x_vec, x2_vec);    
  hold on; 
  axis([-.2 .2 -.2 .2])
plot(X(:,1), X(:,2), 'b-x')
plot(Y(:,1), Y(:,2), 'r--o')
waitforbuttonpress
clc;
clf;
   end 
    end
    

end
end



function X=drawEllipse(a,b,R,x)

%drawEllipse(semi major axis, semi minor axis, angle of rotation, x(1)= x,
%x(2)=y)
   X= zeros(36,2);  
   
    alpha = linspace(0, 360, 36)' .* (pi / 180);
    sinalpha = sin(alpha);
    cosalpha = cos(alpha);

    for i=1:36
   y= x+R'*[a*cosalpha(i), b*sinalpha(i)]'; 
   
   X(i,1)=y(1); 
   X(i,2)=y(2); 
    end
    
    



end

function X=drawDevice(x1_vec,x_vec,x2_vec)

%drawEllipse(semi major axis, semi minor axis, angle of rotation, x(1)= x,
%x(2)=y)
   X= zeros(4,2);  
   
 X(1,:)=[0,0];
 X(2,:)=x1_vec';
 X(3,:)=x_vec'; 
 X(4,:)=x2_vec'; 
 X(5,:)=X(1,:); 
    



end