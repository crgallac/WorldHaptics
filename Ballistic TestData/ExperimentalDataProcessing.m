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



%% Data to Cell Array
for k=1:12
matFileName = sprintf('BallisticTest_%d.mat', k);
	if exist(matFileName, 'file')
		matData{k} = load(matFileName);
	else
		fprintf('File %s does not exist.\n', matFileName);
    end
end


%% Calculate Deviation


for i=1:12
    
   ExpData{i}=struct2cell(matData{i}) 
    
    
end