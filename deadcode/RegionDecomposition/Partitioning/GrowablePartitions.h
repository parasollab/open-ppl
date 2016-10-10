#ifndef _GROWABLEPARTITIONS_H
#define _GROWABLEPARTITIONS_H

#include "PartitioningMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class GrowablePartitions: public PartitioningMethod
{
 public:
  GrowablePartitions();
  GrowablePartitions(XMLNode& in_Node, MPProblem  *m);
  ~GrowablePartitions();

  virtual vector<Partition*> MakePartitions(Partition &p);

 protected:
  int FindClosestPartition(vector<pair<double, Partition*> > &vp, Partition *p,  double compare, double std);
  void RemovePartition(vector<pair<double, Partition*> > &vp, Partition* p);
  Partition* MergePartitions(Partition* a, Partition* b);
  vector<double> GetCenterOfPartition(Partition *p);
};

#endif
