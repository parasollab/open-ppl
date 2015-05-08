#ifndef CFGFEATURE_H_
#define CFGFEATURE_H_

#include "MPFeature.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class CfgFeature : public MPFeature {
 public:
   CfgFeature();
   CfgFeature(int index);
   CfgFeature(XMLNode& in_Node, MPProblem* in_pProblem);
   virtual ~CfgFeature(){}

   virtual void ParseXML(XMLNode& in_Node);

   virtual vector<double> Collect(vector<VID>& vids);

 private:
   int m_index;
};

#endif
