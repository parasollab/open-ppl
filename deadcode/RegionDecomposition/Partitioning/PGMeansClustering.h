#ifndef _PGMEANS_H
#define _PGMEANS_H

#include "PartitioningMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class PGMeansClustering : public PartitioningMethod
{
 public:
   PGMeansClustering();
   PGMeansClustering(XMLNode& in_Node, MPProblem * in_pProblem);
   ~PGMeansClustering();

   virtual vector<Partition*> MakePartitions(Partition &p);

 private:
   void Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features);
};

#endif
