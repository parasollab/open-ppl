#ifndef AVGEVALUATOR_H_
#define AVGEVALUATOR_H_

#include "PartitioningEvaluator.h"

class AVGEvaluator : public PartitioningEvaluator{
 public:
   AVGEvaluator();
   AVGEvaluator(XMLNode& in_Node, MPProblem* mp);
   virtual ~AVGEvaluator(){}

   virtual void ParseXML(XMLNode& in_Node);

   virtual vector<double> Evaluate(vector<Partition*> part);
};

#endif
