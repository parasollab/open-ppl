#ifndef PARTITIONEVALUATOR_H_
#define PARTITIONEVALUATOR_H_

#include "Partition.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class PartitioningEvaluator : public MPBaseObject{
 public:
   PartitioningEvaluator():MPBaseObject(){}
   PartitioningEvaluator(XMLNode& in_Node, MPProblem* mp):MPBaseObject(in_Node, mp){}
   virtual ~PartitioningEvaluator(){}

   virtual void ParseXML(XMLNode& in_Node)=0;

   virtual vector<double> Evaluate(vector<Partition*> part)=0;

   string GetFeature(){return m_Feature;}
   void SetFeature(string f){m_Feature = f;}

 protected:

   string m_Feature;
};

#endif
