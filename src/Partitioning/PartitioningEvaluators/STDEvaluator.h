#ifndef STDEVALUATOR_H_
#define STDEVALUATOR_H_

#include "PartitioningEvaluator.h"

class STDEvaluator : public PartitioningEvaluator{
 public:
   STDEvaluator();
   STDEvaluator(XMLNode& in_Node, MPProblem* mp);
   virtual ~STDEvaluator(){}

   virtual void ParseXML(XMLNode& in_Node);

   virtual vector<double> Evaluate(vector<Partition*> part);
};

#endif
