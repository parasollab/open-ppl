#ifndef PARTITIONINGMETHODS_H_
#define PARTITIONINGMETHODS_H_

#include "MPUtils.h"

class PartitioningMethod;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class PartitioningMethods : public MPBaseObject {
 public:
   PartitioningMethods();
   PartitioningMethods(XMLNode& in_Node, MPProblem* in_pProblem);

   PartitioningMethod* GetPartitioningMethod(string s);

 private:
   vector<PartitioningMethod*> all;
   vector<PartitioningMethod*> selected;
};

#endif
