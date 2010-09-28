%data is imported from Textfile.txt in the form of an n*d matrix
data = importdata('/users/tmadison/PRM_Code/SpatioTemporalStudy/Problems/Manipulator-Hook/SampleDataPoints.txt',' ');
alpha = 0.001; %parameter for pgmeans simulation; can be changed
numProjections = 12; %parameter for pgmeans simulation; can be changed
finalResult = pgmeans_simulation(data, alpha, numProjections) %Run PGmeans

%Write PGmeans result to textfile
fileName = '/users/tmadison/PRM_Code/SpatioTemporalStudy/Problems/Manipulator-Hook/pgmeansClusterOutput.txt';
dlmwrite(fileName, finalResult, 'delimiter', '\t');



%Euclidean Distance
eDist = 0;
%nodeId = 0;
%nodeInfo = zeros(1,4);

dataSize = size(data);
numOfRowsN = dataSize(1,1);
numOfColsN = dataSize(1,2);

finalResultSize = size(finalResult);
numOfRowsC = finalResultSize(1,1);
numOfColsC = finalResultSize(1,2);

matrix = zeros(numOfRowsN,1);
fName = '/users/tmadison/PRM_Code/SpatioTemporalStudy/Problems/Manipulator-Hook/ClusterInformation.txt';

%Write the number of clusters to textfile
fName2 = '/users/tmadison/PRM_Code/SpatioTemporalStudy/src/IncrementalExpansion/numClusters.txt';
fName3 = '/users/tmadison/PRM_Code/SpatioTemporalStudy/Problems/Manipulator-Hook/numClusters.txt';
dlmwrite(fName2, numOfRowsC);
dlmwrite(fName3, numOfRowsC);

%BEGIN: Nested for-loop
%Calculate which clusters the datapoints are closest to
for i=1:numOfRowsN
    minDist = 3;
    closestCluster = 0;
    for j=1:numOfRowsC
    	%Calculate the Euclidean Distance
    	eDist = sqrt(((data(i,1)-finalResult(j,1))^2) + ((data(i,2)-finalResult(j,2))^2) + ((data(i,3)-finalResult(j,3))^2) + ((data(i,4)-finalResult(j,4))^2));
        if (eDist < minDist)
            minDist = eDist;
            closestCluster = j;
        end
    end
    %nodeId = i;
    %nodeInfo = data(i,:);
    matrix(i,:) = [closestCluster];
end
dlmwrite(fName, matrix, 'delimiter', '\t');
