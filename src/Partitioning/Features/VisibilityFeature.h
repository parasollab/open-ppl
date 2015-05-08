#ifndef VISIBILITYFEATURE_H_
#define VISIBILITYFEATURE_H_

#include "Roadmap.h"
#include "CfgTypes.h"
#include "Weight.h"

#include "MPFeature.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class VisibilityFeature : public MPFeature {
 public:
   VisibilityFeature();
   VisibilityFeature(int _k, string _nf, string _dm);
   VisibilityFeature(XMLNode& in_Node, MPProblem* mp);
   virtual ~VisibilityFeature(){}

   virtual void ParseXML(XMLNode& in_Node);

   virtual vector<double> Collect(vector<VID>& vids);

 private:
   int k;
   string nfLabel;
   string dmLabel;
   string m_lpLabel;
};

#endif
