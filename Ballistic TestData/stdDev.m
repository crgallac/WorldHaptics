function ws= stdDev(w1)


sz=size(w1); 

n=sz(1);

for i=1:3
    for j=1:2
sum=norm(w1(:,i,j),2)^2; 
sum1=sum/n; 
ws(j,i)=sqrt(sum1); 

    end
end




end

