SampleDataPoints = rand(10,4);
DistanceArray = pdist(SampleDataPoints, 'euclidean');
DistanceMatrix = squareform(DistanceArray);
LinkageMatrix = linkage(DistanceArray);
Clusters = cluster(LinkageMatrix,'MaxClust',10);
calcVariance(Clusters,SampleDataPoints);