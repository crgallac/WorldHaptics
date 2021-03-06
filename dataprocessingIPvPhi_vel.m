%data processing for IP


DATA={Dir, RevDir, Acc0};
a=.1;


for i=1:3
    for j=1:3
        for k=1:3
        
        


            r= DATA{i}{j,k}{1};
            sz1=size(r);
            w=abs(r(sz1(1),2));
            
            ID= log2((a+w)/w);
            MT=DATA{i}{j,k}{7};
            
            IP= ID/MT;
            

            IPData{i}{j,k}={ DATA{i}{j,k}{9}, DATA{i}{j,k}{8}, ID, MT, IP}; %[phi, a (v), ID, MT, IP]
                   
            
            
            
            
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
 Xacc(i,j)=IPData{1}{i,j}{1};
 Yacc(i,j)=IPData{1}{i,j}{5};
 Xrev(i,j)=IPData{2}{i,j}{1};
 Yrev(i,j)=IPData{2}{i,j}{5};
 Xvel(i,j)=IPData{3}{i,j}{1};
 Yvel(i,j)=IPData{3}{i,j}{5};
 
    end
end
 

IPData

%(IP vs angle) 1, 2, 3 --> .36 .4 .44 duration these can be converted to
%accelerations 
hold on
h1=plot(Xacc(1,:),Yacc(1,:),'x', Xacc(2,:),Yacc(2,:),'x', Xacc(3,:),Yacc(3,:),'x');
h2=plot(Xrev(1,:),Yrev(1,:),'+', Xrev(2,:),Yrev(2,:),'+', Xrev(3,:),Yrev(3,:),'+');
h3=plot(Xvel(1,:),Yvel(1,:),'o', Xvel(2,:),Yvel(2,:),'o', Xvel(3,:),Yvel(3,:),'o');

axis([-.2 5 0 16])
% set(gca, 'XTick', [0, pi/4, pi/2])
% set(gca, 'XTickLabel', {'0','pi/4', 'pi/2'})
% set(h, 'MarkerSize',10)
 xlabel('Angle (rads)')
 ylabel('IP (bits/sec)')











