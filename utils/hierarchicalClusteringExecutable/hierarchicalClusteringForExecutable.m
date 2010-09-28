function  hierarchicalClusteringForExecutable(path)

%Anshul Agrawal - START
%Import Datapoints(sampled nodes separated by a ' ' delimiter)
%SampleDataPoints = importdata('../../Problems/Rigid-spinningtop-Maze/SampleDataPoints.txt',' ');
SampleDataPointsPath = strcat( path,'SampleDataPoints.txt');
fileName = strcat(path,'hierarchicalClusterOutput.txt');

SampleDataPoints = importdata(SampleDataPointsPath,' ');
%Initialize output file
maxClusters= size(SampleDataPoints);
Clusters = [1:maxClusters];
numClustersFile = strcat(path,'numClusters.txt');

%Create the the Distance and the Linkage Matrix to facilitate creation of the Dendrogram and performing Hierarchical clustering
DistanceArray = pdist(SampleDataPoints, 'euclidean');
DistanceMatrix = squareform(DistanceArray);
LinkageMatrix = linkage(DistanceArray,'ward');

%Write introduction data in the output file
outFile= fopen(fileName,'W');
fclose(outFile);

Variance = [1:maxClusters];
%Create clusters for each number between 1 to N where N is the number of nodes
%Calculate the variance for each cluster and then store it for its index.
for i=1:maxClusters(1)
    Clusters = cluster(LinkageMatrix,'MaxClust',i); %replace NumClusters with i when elbow criterion works
    Variance(i) = calcVariance(Clusters,SampleDataPoints);
end

%Plot the Variance vs # of clusters graph
number = plot_Isomap(Variance,'Variance vs # of Clusters','elbow.fig');
Clusters= cluster(LinkageMatrix,'MaxClust',number);;
dlmwrite(fileName, Clusters,'-append');
dlmwrite(numClustersFile, number);

%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%
%%Anshul Agrawal - END
end
