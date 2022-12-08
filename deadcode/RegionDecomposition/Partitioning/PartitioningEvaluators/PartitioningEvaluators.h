#ifndef PARTITIONINGEVALUATORS_H_
#define PARTITIONINGEVALUATORS_H_

#include "MPUtils.h"

class PartitioningEvaluator;
class Partition;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class PartitioningEvaluators : public MPBaseObject{
 public:
   PartitioningEvaluators();
   PartitioningEvaluators(XMLNode& in_Node, MPProblem* mp);
   ~PartitioningEvaluators(){};

   PartitioningEvaluator* GetEvaluator(string s);

   vector<vector<double> > Evaluate(string filename, vector<Partition*> part);

 private:
   vector<PartitioningEvaluator*> all;
   vector<PartitioningEvaluator*> selected;
};

#endif
