function pongDemo()

close all

radius=.1; 
cursorPos=[0 0]; 
wall=[-1.2 0; -1.2 2.2; 1.2 2.2; 1.2 0;]; 
hole=[-1 0; -1 2; 1 2; 1 0;]; 


hold on
area(wall(:,1), wall(:,2));
area(hole(:,1), hole(:,2), 'FaceColor', [1 1 1]); 
daspect([1,1,1]);
axis ([-1.5 1.5 0 2.5])


[X,Y]=createCircle(radius,[cursorPos(1,1), cursorPos(1,2)]);
h= fill(X,Y,'y'); 

 set(gcf,'WindowButtonMotionFcn',@drawscene)
        
    function drawscene(src,evnt)
        cursorPos = get(gca,'CurrentPoint');
        [X,Y]=createCircle(radius,[cursorPos(1,1), cursorPos(1,2)]);
        xdat = X;
        ydat = Y;
        set(h,'XData',xdat,'YData',ydat);
        drawnow
    end
    
    

end


function [X,Y]=createCircle(radius,cursorPos)
 

numPoints=100; %Number of points making up the circle

%Define circle in polar coordinates (angle and radius)
theta=linspace(0,2*pi,numPoints); %100 evenly spaced points between 0 and 2pi
rho=ones(1,numPoints)*radius; %Radius should be 1 for all 100 points

[X,Y]=pol2cart(theta, rho); 

X=X+cursorPos(1); 
Y=Y+cursorPos(2); 


end


