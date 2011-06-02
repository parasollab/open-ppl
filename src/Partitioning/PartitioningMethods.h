/*This file contains the class for loading and storing PartitioningMethods.
 */

#ifndef PARTITIONINGMETHODS_H_
#define PARTITIONINGMETHODS_H_

#include "util.h"

class PartitioningMethod;

class PartitioningMethods : public MPBaseObject {
 public:
   PartitioningMethods();
   PartitioningMethods(XMLNodeReader& in_Node, MPProblem* in_pProblem);

   PartitioningMethod* GetPartitioningMethod(string s);

 private:
   vector<PartitioningMethod*> all;
   vector<PartitioningMethod*> selected;
};

#endif
