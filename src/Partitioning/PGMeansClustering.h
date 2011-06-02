#ifndef _PGMEANS_H
#define _PGMEANS_H

#include "PartitioningMethod.h"

class PGMeansClustering : public PartitioningMethod
{
 public:
   PGMeansClustering();
   PGMeansClustering(XMLNodeReader& in_Node, MPProblem * in_pProblem);
   ~PGMeansClustering();

   virtual vector<Partition*> MakePartitions(Partition &p);

 private:
   void Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features);
};

#endif
