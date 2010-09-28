#ifndef _PGMEANS_H
#define _PGMEANS_H

#include "PartitioningMethod.h"
#include "Partition.h"   

class PGMeansClustering : public PartitioningMethod
{
 public:
   PGMeansClustering();
   PGMeansClustering(XMLNodeReader& in_Node, MPProblem * in_pProblem);
   ~PGMeansClustering();

   virtual void ParseXML(XMLNodeReader& in_Node);

   virtual vector<Partition*> MakePartitions(Partition &p);

   void Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features);

 private:
};

#endif
