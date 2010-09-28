#ifndef VISIBILITYFEATURE_H_
#define VISIBILITYFEATURE_H_

#include "Roadmap.h"
#include "CfgTypes.h"
#include "Weight.h"

#include "MPFeature.h"

class VisibilityFeature : public MPFeature {
 public:
   VisibilityFeature();
   VisibilityFeature(XMLNodeReader& in_Node, MPProblem* mp);
   ~VisibilityFeature(){}
   
   virtual void ParseXML(XMLNodeReader& in_Node);
   
   virtual vector<double> Collect(vector<VID>& vids);

 private:
   int k;
};

#endif
