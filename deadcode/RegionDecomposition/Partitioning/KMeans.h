#ifndef _KMEANS_H
#define _KMEANS_H

#include "PartitioningMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class KMeans : public PartitioningMethod
{
 public:
  KMeans();
  KMeans(XMLNode& in_Node, MPProblem * in_pProblem);
  ~KMeans();

  virtual vector<Partition*> MakePartitions(Partition &p);

 private:
  void Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features);
};

#endif
