#ifndef AVGEVALUATOR_H_
#define AVGEVALUATOR_H_

#include "PartitioningEvaluator.h"
#include "Partition.h"

class AVGEvaluator : public PartitioningEvaluator{
 public: 
   AVGEvaluator();
   AVGEvaluator(XMLNodeReader& in_Node, MPProblem* mp);
   virtual ~AVGEvaluator(){}

   virtual void ParseXML(XMLNodeReader& in_Node);

   virtual vector<double> Evaluate(vector<Partition*> part);
};

#endif
