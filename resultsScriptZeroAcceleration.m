
%%% script to get results and desired plots 



%% Input Parameters
p_length=0.1; %meter
% dur=.4; % second ----> this will be made into a variable for later models. 
%%% this 0.4 seconds is approximately constant for ballistic trajectories 
phi=pi/2; 
offset=[0 0.2]'; % in the operation space
dt=.001; %seconds
rdot0=[0 0]; %velocity 
 

    
        durArr=[.36 .4 .44];
        rdot0Arr=p_length./durArr
        phiArr=[pi/2 pi/4 0];
        
   
       szRdot= size(rdot0Arr);
       szPhi= size(phiArr);
       
       
numOfRdot= szRdot(2);
numOfAng= szPhi(2);

p_length=0; 


for i=1:numOfRdot
    for j=1:numOfAng

dur=durArr(1,i);
phi=phiArr(1,j);
rdot0(1,1)=rdot0Arr(1,i); 

 sz= int8(dur/dt);

data= {zeros(sz+1,2),zeros(sz+1,2),zeros(sz+1,2),zeros(sz+1,2),zeros(sz+1,2),zeros(sz+1,2), 0, 0, 0, [0 0] };

[r, rdot, rddot, F, dr, W, a]= fiveBarPath_task2op_function(p_length, dur, phi, offset, dt, rdot0);

data{1}=r; 
data{2}=rdot;
data{3}=rddot;
data{4}=F;
data{5}=dr;
data{6}=W; 
data{7}=dur;
data{8}=rdot0(1,1);
data{9}=phi; 
data{10}=offset;



Acc0{i,j}= data; 


    end
end


Acc0

%C{1,1} accesses the cell that contains the data
%C{1,1}{1} access the specific data vector
%C{1,1}{1}(1,1) accesses a specific index of the vector


% [r, rdot, rddot, F, dr, W]= fiveBarPath_task2op_function(p_length, dur, phi, offset, dt, rdot0);
% 




%  Wnet=[0 0]; 
%  for i=1:(dur/dt)
%  
%      Wnet= Wnet+W(i,:); 
%      
%  end