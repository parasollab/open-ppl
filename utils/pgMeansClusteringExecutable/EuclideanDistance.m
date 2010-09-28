eDist = 0;
nodeId = 0;
nodeInfo = zeros(1,4);
dataSize = size(data);
numOfRowsN = dataSize(1,1);
numOfColsN = dataSize(1,2);
finalResult = pgmeans_simulation(data, alpha, numProjections)
finalResultSize = size(finalResult);
numOfRowsC = finalResultSize(1,1);
numOfColsC = finalResultSize(1,2);
matrix = zeros(numOfRowsN,1);
fName = '/users/tmadison/PRM_Code/SpatioTemporalStudy/src/IncrementalExpansion/ClusterInformation.txt';

%Nested for-loop
for i=1:numOfRowsN
    clusters = zeros(1,4);
    minDist = 10;
    closestCluster = 0;
    for j=1:numOfRowsC
    	%Calculate the Euclidean Distance
    	eDist = sqrt(((data(i,1)-finalResult(j,1))^2) + ((data(i,2)-finalResult(j,2))^2) + ((data(i,3)-finalResult(j,3))^2) + ((data(i,4)-finalResult(j,4))^2));
        if (eDist < minDist)
            minDist = eDist;
            closestCluster = j;
        end
    end
    nodeId = i;
    nodeInfo = data(i,:);
    matrix(i,:) = [closestCluster];
end
dlmwrite(fName, matrix, 'delimiter', '\t');

