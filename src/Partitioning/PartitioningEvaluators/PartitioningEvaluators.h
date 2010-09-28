#ifndef PARTITIONINGEVALUATORS_H_
#define PARTITIONINGEVALUATORS_H_

#include "PartitioningEvaluator.h"
#include "Partition.h"
#include "MPProblem.h"

#include "STDEvaluator.h"
#include "AVGEvaluator.h"

class PartitioningEvaluator;

class PartitioningEvaluators{
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
