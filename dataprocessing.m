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

plot([Xacc1],[Yacc1],'o', [Xacc2],[Yacc2],'x', [Xacc3],[Yacc3],'+')











