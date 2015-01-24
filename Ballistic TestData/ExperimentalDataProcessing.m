% ExperimentalDataProcessing

% % Each test subject has a file that contains a 5x12 array, one column for each movement.
% % 
% % The 5 rows are as follows:
% % 
% % 1. Direction  0 = forward
% % 	            1 = backwards
% % 2. Angle       0,pi/4, p/2 for forward
% % 		+pi for backwards
% % 3. Target Diameter (m)
% % 4. Time (s)
% % 5. Distance from centre of target when plane crossed


% 
% %% Data to Cell Array
% for k=1:12
% matFileName = sprintf('BallisticTest_%d.mat', k);
% 	if exist(matFileName, 'file')
% 		matData{k} = load(matFileName);
% 	else
% 		fprintf('File %s does not exist.\n', matFileName);
%     end
% end
% 
% 
 %% Structure Data
% 
% for i=1:12
    
%    ExpData{i}=struct2cell(matData{i}) 
%    edat{i}=cell2mat(ExpData{i});  
%     
% end

% % l=0;
% % w=zeros(12,3,2);
% % t=w;
% % 
% % for i=1:12 %test subject
% %     for    j=1:12 %individual test info
% %         
% %         out='individual loop'
% %         if(edat{i}(1,j)==0) %Forward
% %             d='forward'
% %             if(edat{i}(3,j)==0.0064)
% %                 
% %                 l=1;%Small
% % %                 waitforbuttonpress
% %             else
% %                 l=2;%Big
% % %                 waitforbuttonpress
% %             end
% %             
% %             i;
% %             j;
% %             f=fix(edat{i}(2,j)*100)/100; 
% %             if(f==fix(pi/2*100)/100)
% %                 p=1;
% % %                   waitforbuttonpress
% %                 w(i,1,l)=edat{i}(5,j); %distance from target  %trial numbe
% %                 t(i,1,l)=edat{i}(4,j); %time
% % 
% %             elseif(f==fix(pi/4*100)/100)
% %                 p=2;
% % %                 waitforbuttonpress
% %                 w(i,2,l)=edat{i}(5,j); %distance from target
% %                 t(i,2,l)=edat{i}(4,j); %time
% %             
% %             elseif (f==fix(0.0*100)/100)  %%%% 0 degrees ... Think i can say else since I'm just testing the forward case ... may have if the roundoff isn't close enough to satisfy the = sign
% %                 p=3;
% % %                     waitforbuttonpress
% %                 w(i,3,l)=edat{i}(5,j) ;%distance from target
% %                 t(i,3,l)=edat{i}(4,j); %time
% %             end
% %             
% %         elseif (edat{i}(1,j)==1)
% %             d='reverse';
% %           
% %         end
% %     
% %         
% %     end
% % end
% % 
% % l=0;
% % w1=zeros(12,3,2);
% % t1=w;
% % 
% % for i=1:12 %test subject
% %     for    j=1:12 %individual test info
% %         
% %         out='individual loop'
% %         if(edat{i}(1,j)==1) %reverse
% %             d='reverse'
% %             if(edat{i}(3,j)==0.0064)
% %                 
% %                 l=1;%Small
% % %                 waitforbuttonpress
% %             else
% %                 l=2;%Big
% % %                 waitforbuttonpress
% %             end
% %             
% %             i;
% %             j;
% %             f=fix(edat{i}(2,j)*100)/100; 
% %             if(f==fix(3*pi/2*100)/100)
% %                 p=1;
% % %                   waitforbuttonpress
% %                 w1(i,1,l)=edat{i}(5,j); %distance from target  %trial numbe
% %                 t1(i,1,l)=edat{i}(4,j); %time
% % 
% %             elseif(f==fix(5*pi/4*100)/100)
% %                 p=2;
% % %                 waitforbuttonpress
% %                 w1(i,2,l)=edat{i}(5,j); %distance from target
% %                 t1(i,2,l)=edat{i}(4,j); %time
% %             
% %             elseif (f==fix(pi*100)/100)  %%%% 0 degrees ... Think i can say else since I'm just testing the forward case ... may have if the roundoff isn't close enough to satisfy the = sign
% %                 p=3;
% % %                     waitforbuttonpress
% %                 w1(i,3,l)=edat{i}(5,j) ;%distance from target
% %                 t1(i,3,l)=edat{i}(4,j); %time
% %             end
% %             
% %         elseif (edat{i}(1,j)==0)
% %             d='forward';
% %           
% %         end
% %     
% %         
% %     end
% % end

% % 
 %% Plotting Preparation
A=.1; 
W_big=0.0191/2;
W_small=0.0064/2; 

ID_big= log2((A+W_big)/W_big); 
ID_small= log2((A+W_small)/W_small);



w_std=stdDev(w);
% waitforbuttonpress
w_av=mean(w);
t_std=stdDev(t); 
t_av=mean(t); 

w_eff= 2*w_std; 

num=A+w_eff;
den=w_eff;

ID_eff=log2(num./w_eff); 


 
    

w1_std=stdDev(w1);
w1_av=mean(w1); 
t1_std=stdDev(t1); 
t1_av=mean(t1); 

w1_eff=2*w1_std;

num1=A+w1_eff;
den1=w1_eff;

% num1./den1
% waitforbuttonpress

ID1_eff=log2(num1./w1_eff)
% waitforbuttonpress
   
%% Plotting


%PlOTS the Time vs ID for the forward motions
X=zeros(24,1);
Y=zeros(24,1); 

X(1:12)=ID_small; 
X(13:24)=ID_big;

Y(1:12)=t(:,1,1); 
Y(13:24)=t(:,1,2); 

Y1(1:12)=t(:,2,1); 
Y1(13:24)=t(:,2,2); 

Y2(1:12)=t(:,3,1); 
Y2(13:24)=t(:,3,2); 

close all

% hold on
% plot(X,Y,'o',X,Y1,'x',X,Y2,'+'); 



%PlOTS the Time vs ID for the reverse motions
figure

X=zeros(24,1);
Y=zeros(24,1); 
Y1=zeros(24,1); 
Y2=zeros(24,1); 

X(1:12)=ID_small; 
X(13:24)=ID_big;

Y(1:12)=t1(:,1,1); 
Y(13:24)=t1(:,1,2); 

Y1(1:12)=t1(:,2,1); 
Y1(13:24)=t1(:,2,2); 

Y2(1:12)=t1(:,3,1); 
Y2(13:24)=t1(:,3,2); 

% hold on

% plot(X,Y,'o',X,Y1,'x',X,Y2,'+'); 

%%%%%%%%%%%PlOTS the Time vs adjusted ID for the forward motions
figure
%%% Adjusted plotting

X=zeros(24,1);
X1=zeros(24,1);
X2=zeros(24,1);
Y=zeros(24,1); 
Y1=zeros(24,1); 
Y2=zeros(24,1); 

X(1:12)= ID_eff(1,1); 
X(13:24)=ID_eff(2,1); 

X1(1:12)= ID_eff(1,2); 
X1(13:24)=ID_eff(2,2); 

X2(1:12)= ID_eff(1,3); 
X2(13:24)=ID_eff(2,3); 

Y(1:12)=t(:,1,1); 
Y(13:24)=t(:,1,2); 

Y1(1:12)=t(:,2,1); 
Y1(13:24)=t(:,2,2); 

Y2(1:12)=t(:,3,1); 
Y2(13:24)=t(:,3,2); 

plot(X,Y,'o',X1,Y1,'x',X2,Y2,'+'); 


%%%%%%%%%%%PlOTS the Time vs adjusted ID for the reverse motions

figure
%%% Adjusted plotting

X=zeros(24,1);
X1=zeros(24,1);
X2=zeros(24,1);
Y=zeros(24,1); 
Y1=zeros(24,1); 
Y2=zeros(24,1); 

X(1:12)= ID1_eff(1,1); 
X(13:24)=ID1_eff(2,1); 

X1(1:12)= ID1_eff(1,2); 
X1(13:24)=ID1_eff(2,2); 

X2(1:12)= ID1_eff(1,3); 
X2(13:24)=ID1_eff(2,3); 

Y(1:12)=t1(:,1,1); 
Y(13:24)=t1(:,1,2); 

Y1(1:12)=t1(:,2,1); 
Y1(13:24)=t1(:,2,2); 

Y2(1:12)=t1(:,3,1); 
Y2(13:24)=t1(:,3,2); 

plot(X,Y,'o',X1,Y1,'x',X2,Y2,'+'); 

t_av_=zeros(2,3); 
t1_av_=zeros(2,3); 

for i=1:3
    for j=1:2
t_av_(j,i)=t_av(1,i,j);
t1_av_(j,i)=t1_av(1,i,j)'; 
    end
end

IP=ID_eff./t_av_; 
IP1=ID1_eff./t1_av_; 

IP=ID_eff./t_tot_av; 
IP1=ID1_eff./t_tot_av; 


Phi=[0, pi/4, pi/2; pi, pi/4, pi/2;];

% hold on
figure
 h1=plot(Phi(:,1),IP(:,1),'x', Phi(:,2),IP(:,2),'x', Phi(:,3),IP(:,3),'x');
% legend(h1,{'forward: 3.09 m/s^2', 'forward: 2.5 m/s^2','forward: 2.07 m/s^2'}); 
%  set(h, 'String', {'forward: 3.09 m/s^2', 'forward: 2.5 m/s^2','forward: 2.07 m/s^2'})


axis([-.2 pi/2+.2 0 5])
set(gca, 'XTick', [0, pi/4, pi/2])
set(gca, 'XTickLabel', {'0 (Path 3)','pi/4 (Path 2)', 'pi/2 (Path 1)'})
% set(gca, 'Xdir','reverse')
 xlabel('Angle (rads)')
 ylabel('IP (bits/sec)')
set(h1, 'MarkerSize',10)

figure
% h2=plot(Xrev(1,:),Yrev(1,:),'+', Xrev(2,:),Yrev(2,:),'+', Xrev(3,:),Yrev(3,:),'+');
 h2=plot(Phi(:,1),IP1(:,1),'+', Phi(:,2),IP1(:,2),'+', Phi(:,3),IP1(:,3),'+');
% h=legend('show');
% set(h, 'String', {'forward: 3.09 m/s^2', 'forward: 2.5 m/s^2','forward: 2.07 m/s^2', char(10), 'reversed: 3.09 m/s^2', 'reversed: 2.5 m/s^2','reversed: 2.07 m/s^2'}); 
% columnlegend(2,{'3.09 m/s^2', '2.5 m/s^2','2.07 m/s^2', '0.28 m/s', '0.25 m/s','0.23 m/s'},'best');

axis([-.2 pi/2+.2 0 5])
set(gca, 'XTick', [0, pi/4, pi/2])
set(gca, 'XTickLabel', {'0 (Path 3)','pi/4 (Path 2)', 'pi/2 (Path 1)'})
% set(gca, 'Xdir','reverse')
% set(h1, 'MarkerSize',10)
set(h2, 'MarkerSize',10)
% set(h3, 'MarkerSize',7)
 xlabel('Angle (rads)')
 ylabel('IP (bits/sec)')

 
 %%%%%%%%%%%%%%OUTPUT TABLE CONTAINING DATA FROM THE EXPERIMENT MOST
 %%%%%%%%%%%%%%NOTABLY THE W_effective %%%% WHICH SHOWS HOW PEOPLE MISSED
 %%%%%%%%%%%%%%THE TARGET 
 
 
 
 
 
 