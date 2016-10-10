#ifndef STDEVALUATOR_H_
#define STDEVALUATOR_H_

#include "PartitioningEvaluator.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class STDEvaluator : public PartitioningEvaluator{
 public:
   STDEvaluator();
   STDEvaluator(XMLNode& in_Node, MPProblem* mp);
   virtual ~STDEvaluator(){}

   virtual void ParseXML(XMLNode& in_Node);

   virtual vector<double> Evaluate(vector<Partition*> part);
};

#endif
