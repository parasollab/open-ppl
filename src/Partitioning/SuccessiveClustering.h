#ifndef _SUCCESSIVECLUSTERING_H
#define _SUCCESSIVECLUSTERING_H

#include "PartitioningMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class SuccessiveClustering : public PartitioningMethod
{
 public:
  SuccessiveClustering();
  SuccessiveClustering(XMLNode& in_Node, MPProblem * in_pProblem);
  ~SuccessiveClustering();

  virtual void ParseXML(XMLNode& in_Node);

  virtual vector<Partition*> MakePartitions(Partition &p);

 private:
   vector<string> m_PartitioningMethods;
};

#endif
