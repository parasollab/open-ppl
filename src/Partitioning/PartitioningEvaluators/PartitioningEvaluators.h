#ifndef PARTITIONINGEVALUATORS_H_
#define PARTITIONINGEVALUATORS_H_

#include "MPUtils.h"

class PartitioningEvaluator;
class Partition;

class PartitioningEvaluators : public MPBaseObject{
 public:
   PartitioningEvaluators();
   PartitioningEvaluators(XMLNodeReader& in_Node, MPProblem* mp);
   ~PartitioningEvaluators(){};

   PartitioningEvaluator* GetEvaluator(string s);

   vector<vector<double> > Evaluate(string filename, vector<Partition*> part);

 private:
   vector<PartitioningEvaluator*> all;
   vector<PartitioningEvaluator*> selected;
};

#endif
