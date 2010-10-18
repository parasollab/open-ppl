#ifndef FEATURES_H_
#define FEATURES_H_

#include "Roadmap.h"
#include "CfgTypes.h"
#include "MPFeature.h"
#include "Weight.h"

#include "CfgFeature.h"
#include "VisibilityFeature.h"
#include "ClearanceFeature.h"
#include "CSpaceClearanceFeature.h"

class Features{
 public:
   Features();
   Features(XMLNodeReader& in_Node, MPProblem* mp);
   ~Features(){}

   MPFeature* GetFeature(string s);

   int GetFeatureIndex(string s);

   vector<vector<double> > Collect(vector<VID>& vids);
   vector<vector<double> > Collect(vector<string> features, vector<VID>& vids);

 private:
   vector<MPFeature*> all;
   vector<MPFeature*> selected;
};

#endif
