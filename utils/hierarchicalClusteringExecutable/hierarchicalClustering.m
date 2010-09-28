%%Anshul Agrawal - START

%Import Datapoints(sampled nodes separated by a ' ' delimiter)
SampleDataPoints = importdata('../../Problems/Rigid-spinningtop-Maze/SampleDataPoints.txt',' ');
%Initialize output file
maxClusters= size(SampleDataPoints);
Clusters = [1:maxClusters];
fileName = '../../Problems/Rigid-spinningtop-Maze/hierarchicalClusterOutput.txt';
numClustersFile = '../../Problems/Rigid-spinningtop-Maze/numClusters.txt';

%Create the the Distance and the Linkage Matrix to facilitate creation of the Dendrogram and performing Hierarchical clustering
DistanceArray = pdist(SampleDataPoints, 'euclidean');
DistanceMatrix = squareform(DistanceArray);
LinkageMatrix = linkage(DistanceArray,'ward');

%Create and Display a Dendrogram
dendrogram(LinkageMatrix,0,'ORIENTATION', 'top');
grid on;

%Display the quality of the clusters
cophenet(LinkageMatrix,DistanceArray)

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
figure;
number = plot_Isomap(Variance,'Variance vs # of Clusters','elbow.fig')
Clusters= cluster(LinkageMatrix,'MaxClust',number);
dlmwrite(fileName, Clusters,'-append');
dlmwrite(numClustersFile, number);
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot all the nodes in the same color
%%%%%%%%%%%%%%%% START %%%%%%%%%%%%%%%%%%%%%% 
% nPts= size(SampleDataPoints);
%     figure;
%     plot3(SampleDataPoints(:,2),SampleDataPoints(:,3),SampleDataPoints(:,4),'d');
%     hold on;
%     grid on;
%     box on;
%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot different visibility ratio points with different colors
%%%%%%%%%%%%%%%% START %%%%%%%%%%%%%%%%%%%%%%
% for i=1:nPts(1)
%     if(SampleDataPoints(i,1)<.33)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*b');
%     elseif(SampleDataPoints(i,1)>.33 && SampleDataPoints(i,1)<.66)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*g');
%     elseif(SampleDataPoints(i,1)>.66)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*r');
%     end
% end
%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot a user inputted cluster in blue
%%%%%%%%%%%%%%%% START %%%%%%%%%%%%%%%%%%%%%%
% numb=1;
% while numb > 0
%     numb= input ('enter cluster# to highlight')
%     if(Clusters(i)==numb)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*b');
%     end
% end
%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot different cluster points with different colors (upto 8 clusters)
%%%%%%%%%%%%%%%% START %%%%%%%%%%%%%%%%%%%%%%
% for i=1:nPts(1)
%     if(Clusters(i)==1)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*r');
%     elseif(Clusters(i)==2)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*g');
%     elseif(Clusters(i)==3)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*b');
%     elseif(Clusters(i)==4)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*c');
%     elseif(Clusters(i)==5)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*m');
%     elseif(Clusters(i)==6)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*y');
%     elseif(Clusters(i)==7)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*k');
%     elseif(Clusters(i)==8)
%         plot3(SampleDataPoints(i,2),SampleDataPoints(i,3),SampleDataPoints(i,4),'*w');

%     end
% end
%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%
%%Anshul Agrawal - END
