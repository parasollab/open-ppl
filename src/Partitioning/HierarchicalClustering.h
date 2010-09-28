#ifndef _HCLUSTERING_H
#define _HCLUSTERING_H

#include "PartitioningMethod.h"
#include "Partition.h"  

class HierarchicalClustering : public PartitioningMethod
{
 public:
   HierarchicalClustering();
   HierarchicalClustering(XMLNodeReader& in_Node, MPProblem * in_pProblem);
   ~HierarchicalClustering();

   virtual void ParseXML(XMLNodeReader& in_Node);

   virtual vector<Partition*> MakePartitions(Partition &p);

   void Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features);

 private:
};

#endif
