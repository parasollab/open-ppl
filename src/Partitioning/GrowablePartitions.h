#ifndef _GROWABLEPARTITIONS_H
#define _GROWABLEPARTITIONS_H

#include "Partition.h"
#include "PartitioningMethod.h"

class GrowablePartitions: public PartitioningMethod
{
 public:
  GrowablePartitions();
  GrowablePartitions(XMLNodeReader& in_Node, MPProblem  *m);
  ~GrowablePartitions();

  virtual void ParseXML(XMLNodeReader& in_Node);

  virtual vector<Partition*> MakePartitions(Partition &p);

 protected:
  int FindClosestPartition(vector<pair<double, Partition*> > &vp, Partition *p,  double compare, double std);
  void RemovePartition(vector<pair<double, Partition*> > &vp, Partition* p);
  Partition* MergePartitions(Partition* a, Partition* b);
  vector<double> GetCenterOfPartition(Partition *p);
};

#endif
