#ifndef _HCLUSTERING_H
#define _HCLUSTERING_H

#include "PartitioningMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class HierarchicalClustering : public PartitioningMethod
{
 public:
   HierarchicalClustering();
   HierarchicalClustering(XMLNode& in_Node, MPProblem * in_pProblem);
   ~HierarchicalClustering();

   virtual vector<Partition*> MakePartitions(Partition &p);

 private:
   void Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features);
};

#endif
