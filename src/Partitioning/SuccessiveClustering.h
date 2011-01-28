#ifndef _SUCCESSIVECLUSTERING_H
#define _SUCCESSIVECLUSTERING_H

#include "PartitioningMethod.h"
#include "Partition.h"

class SuccessiveClustering : public PartitioningMethod
{
 public:
  SuccessiveClustering();
  SuccessiveClustering(XMLNodeReader& in_Node, MPProblem * in_pProblem);
  ~SuccessiveClustering();

  virtual void ParseXML(XMLNodeReader& in_Node);

  virtual vector<Partition*> MakePartitions(Partition &p);

 private:
   vector<string> m_PartitioningMethods;
};

#endif
