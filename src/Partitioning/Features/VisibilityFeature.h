#ifndef VISIBILITYFEATURE_H_
#define VISIBILITYFEATURE_H_

#include "Roadmap.h"
#include "CfgTypes.h"
#include "Weight.h"

#include "MPFeature.h"

class VisibilityFeature : public MPFeature {
 public:
   VisibilityFeature();
   VisibilityFeature(int _k, string _nf, string _dm);
   VisibilityFeature(XMLNodeReader& in_Node, MPProblem* mp);
   virtual ~VisibilityFeature(){}
   
   virtual void ParseXML(XMLNodeReader& in_Node);
   
   virtual vector<double> Collect(vector<VID>& vids);

 private:
   int k;
   string nfLabel;
   string dmLabel;
};

#endif
