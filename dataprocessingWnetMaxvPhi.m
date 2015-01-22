%data processing for IP


DATA={Dir, RevDir, Acc0};
a=.1;

WnetMax=0; 
Wnet=[0 0]; 
for i=1:3
    for j=1:3
        for k=1:3
        
        


            Work= DATA{i}{j,k}{6};
%             waitforbuttonpress
            
                  
            Work=abs(Work) ;
            Wnet=sum(Work(:,1))+sum(Work(:,2));
            w_rat=abs(sum(Work(:,1))/(sum(Work(:,2))+sum(Work(:,1))));
%             waitforbuttonpress
            
if(Wnet>WnetMax)
   WnetMax=Wnet; 
end
       

            WData{i}{j,k}={DATA{i}{j,k}{9}, Wnet}; %acceleratoin to velocity terms
                   
            
            
            
            
        end
    end
    
%     data{1}=r; 
% data{2}=rdot;
% data{3}=rddot;
% data{4}=F;
% data{5}=dr;
% data{6}=W; 
% data{7}=dur;
% data{8}=a;
% data{9}=phi; 
% data{10}=offset;

end



for i=1:3
    for j=1:3
 Xacc(i,j)=WData{1}{i,j}{1};
 Yacc(i,j)=WData{1}{i,j}{2}/WnetMax;
 Xrev(i,j)=WData{2}{i,j}{1};
 Yrev(i,j)=WData{2}{i,j}{2}/WnetMax;
 Xvel(i,j)=WData{3}{i,j}{1};
 Yvel(i,j)=WData{3}{i,j}{2}/WnetMax;
 
    end
end
 

WData

%(IP vs angle) 1, 2, 3 --> .36 .4 .44 duration these can be converted to
%accelerations 

hold on
h1=plot(Xacc(1,:),Yacc(1,:),'x', Xacc(2,:),Yacc(2,:),'x', Xacc(3,:),Yacc(3,:),'x');
h2=plot(Xrev(1,:),Yrev(1,:),'+', Xrev(2,:),Yrev(2,:),'+', Xrev(3,:),Yrev(3,:),'+');
h3=plot(Xvel(1,:),Yvel(1,:),'o', Xvel(2,:),Yvel(2,:),'o', Xvel(3,:),Yvel(3,:),'o');

hold off
% axis([-.2 5 0 16])
% set(gca, 'XTick', [0, pi/4, pi/2])
% set(gca, 'XTickLabel', {'0','pi/4', 'pi/2'})
% set(h1, 'MarkerSize',10)
% set(h2, 'MarkerSize',10)
% set(h3, 'MarkerSize',10)
%  xlabel('Angle (rads)')
%  ylabel('IP (bits/sec)')











