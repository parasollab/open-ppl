%%Anshul Agrawal -START
%This function takes in the cluster data(where every node is assigned a cluster number) and calculates the variance in the Sampled Data Points 
function [finalVariance] = calcVariance(Clusters,SampleDataPoints)


clusterMatrix = [Clusters,SampleDataPoints];
[a,b] = sort(clusterMatrix(:,1));
sortedClustMat = [a,b];

dataSize= size(SampleDataPoints);
nPts = dataSize(1);
dimensions=dataSize(2);
clusterCount = max(Clusters);

variance = zeros(clusterCount,dimensions);
SumAndMean = zeros(clusterCount,dimensions);
finalVariancesClusters = zeros(1,clusterCount);
clusterSize = zeros(1,clusterCount);
for j=1:nPts
    clusterSize(sortedClustMat(j,1)) = 1 + clusterSize(sortedClustMat(j,1));
end
for i = 1:dimensions

    %calculate the sum of eatch feature each cluster
    for j=1:nPts
        SumAndMean(sortedClustMat(j,1),i) = SampleDataPoints(sortedClustMat(j,2),i) + SumAndMean(sortedClustMat(j,1),i); 
    end
    
    SumAndMean;
    %calculate the means
    for j=1:clusterCount
        SumAndMean(j,i) = SumAndMean(j,i)/ clusterSize(j);
    end
    %
    %calvulcate the variance for each feature each cluster
    for j=1:nPts
        variance(sortedClustMat(j,1),i) = ((SampleDataPoints(sortedClustMat(j,2),i)-SumAndMean(sortedClustMat(j,1),i))^2)+variance(sortedClustMat(j,1),i);
    end
    %input('Continue?');
    variance;
    for j=1:clusterCount
        variance(j,i) =variance(j,i)/ clusterSize(j) ;
    end
    variance;
    %input('Continue?');
end
SumAndMean;
for i=1:clusterCount
    finalVariancesClusters(i) = mean(variance(i,:));
end


finalVariance = mean(finalVariancesClusters);