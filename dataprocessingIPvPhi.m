%data processing for 

DATA={Dir, RevDir, Acc0};
a=.1;



    for i=1:2
        for j=1:3
        
        


            r= DATA{i}{j}{1};
            sz1=size(r);
            w=abs(r(sz1(1),2));
            
            ID= log2((a+w)/w);
            MT=DATA{i}{j}{7};
            
            IP= ID/MT;
            

            IPData{i}{j}={ DATA{i}{j}{9}, DATA{i}{j}{8}, ID, MT, IP}; %[phi, a (v), ID, MT, IP]
                   
            
            
            
            
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





for i=1:2
    for j=1:3
 Xacc(i,j)=IPData{i}{j}{1};
 Yacc(i,j)=IPData{1}{j}{5};
 Xrev(i,j)=IPData{2}{j}{1};
 Yrev(i,j)=IPData{2}{j}{5};
%  Xvel(i,j)=IPData{3}{i,j}{1};
%  Yvel(i,j)=IPData{3}{i,j}{5};
 
    end
end
 

IPData

%(IP vs angle) 1, 2, 3 --> .36 .4 .44 duration these can be converted to
%accelerations 
% close all

Xrev(:,:)=Xrev(:,:)-pi; 
hold on
 h1=plot(Xacc(1,:),Yacc(1,:),'x', Xacc(2,:),Yacc(2,:),'x');
% legend(h1,{'forward: 3.09 m/s^2', 'forward: 2.5 m/s^2','forward: 2.07 m/s^2'}); 
%  set(h, 'String', {'forward: 3.09 m/s^2', 'forward: 2.5 m/s^2','forward: 2.07 m/s^2'})

h2=plot(Xrev(1,:),Yrev(1,:),'+', Xrev(2,:),Yrev(2,:),'+');
% h3=plot(Xvel(1,:),Yvel(1,:),'o', Xvel(2,:),Yvel(2,:),'o', Xvel(3,:),Yvel(3,:),'o');
% h=legend('show');
% set(h, 'String', {'forward: 3.09 m/s^2', 'forward: 2.5 m/s^2','forward: 2.07 m/s^2', char(10), 'reversed: 3.09 m/s^2', 'reversed: 2.5 m/s^2','reversed: 2.07 m/s^2'}); 
% columnlegend(2,{'3.09 m/s^2', '2.5 m/s^2','2.07 m/s^2', '0.28 m/s', '0.25 m/s','0.23 m/s'},'best');

axis([-.2 pi/2+.2 0 5])
set(gca, 'XTick', [0, pi/4, pi/2])
set(gca, 'XTickLabel', {'0 (Path 3)','pi/4 (Path 2)', 'pi/2 (Path 1)'})
set(gca, 'Xdir','reverse')
% set(h1, 'MarkerSize',10)
set(h2, 'MarkerSize',10)
% set(h3, 'MarkerSize',7)
 xlabel('Angle (rads)')
 ylabel('IP (bits/sec)')











