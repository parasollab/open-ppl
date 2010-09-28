#ifndef _KMEANS_H
#define _KMEANS_H

#include "PartitioningMethod.h"
#include "Partition.h"
#include "Kmeans/KMlocal.h"   

class KMeans : public PartitioningMethod
{
 public:
  KMeans();
  KMeans(XMLNodeReader& in_Node, MPProblem * in_pProblem);
  ~KMeans();

  virtual void ParseXML(XMLNodeReader& in_Node);

  virtual vector<Partition*> MakePartitions(Partition &p);

  void Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features);

 private:
};

#endif
