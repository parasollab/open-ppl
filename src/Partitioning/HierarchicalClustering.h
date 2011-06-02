#ifndef _HCLUSTERING_H
#define _HCLUSTERING_H

#include "PartitioningMethod.h"

class HierarchicalClustering : public PartitioningMethod
{
 public:
   HierarchicalClustering();
   HierarchicalClustering(XMLNodeReader& in_Node, MPProblem * in_pProblem);
   ~HierarchicalClustering();

   virtual vector<Partition*> MakePartitions(Partition &p);

 private:
   void Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features);
};

#endif
