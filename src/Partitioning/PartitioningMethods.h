/*This file contains the class for loading and storing PartitioningMethods.
 */

#ifndef PARTITIONINGMETHODS_H_
#define PARTITIONINGMETHODS_H_

#include "PartitioningMethod.h"
#include "GrowablePartitions.h"
#include "KMeans.h"
#include "HierarchicalClustering.h"
#include "PGMeansClustering.h"

class PartitioningMethod;

class PartitioningMethods {
 public:
   PartitioningMethods();
   PartitioningMethods(XMLNodeReader& in_Node, MPProblem* in_pProblem);

   PartitioningMethod* GetPartitioningMethod(string s);

 private:
   vector<PartitioningMethod*> all;
   vector<PartitioningMethod*> selected;
};

#endif
