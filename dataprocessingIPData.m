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



    for j=1:3
 Xacc1(j)=IPData{1}{1,j}{1} ;
 Yacc1(j)=IPData{1}{1,j}{5};
 
 
    end

        for j=1:3
 Xacc2(j)=IPData{1}{2,j}{1} ;
 Yacc2(j)=IPData{1}{2,j}{5};
 
        end
    
            for j=1:3
 Xacc3(j)=IPData{1}{3,j}{1} ;
 Yacc3(j)=IPData{1}{3,j}{5};
 
            end


IPData

%(IP vs angle) 1, 2, 3 --> .36 .4 .44 duration these can be converted to
%accelerations 

h=plot([Xacc1],[Yacc1],'o', [Xacc2],[Yacc2],'x', [Xacc3],[Yacc3],'+')
axis([-.2 1.8 0 14])
set(gca, 'XTick', [0, pi/4, pi/2])
set(gca, 'XTickLabel', {'0','pi/4', 'pi/2'})
set(h, 'MarkerSize',10)
xlabel('Angle (rads)')
ylabel('IP (bits/sec)')











