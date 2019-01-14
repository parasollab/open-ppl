#ifndef AVGEVALUATOR_H_
#define AVGEVALUATOR_H_

#include "PartitioningEvaluator.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class AVGEvaluator : public PartitioningEvaluator{
 public:
   AVGEvaluator();
   AVGEvaluator(XMLNode& in_Node, MPProblem* mp);
   virtual ~AVGEvaluator(){}

   virtual void ParseXML(XMLNode& in_Node);

   virtual vector<double> Evaluate(vector<Partition*> part);
};

#endif
