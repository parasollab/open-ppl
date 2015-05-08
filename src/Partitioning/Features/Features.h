#ifndef FEATURES_H_
#define FEATURES_H_

#include "MPFeature.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class Features: public MPBaseObject{
  public:
    Features();
    Features(XMLNode& in_Node, MPProblem* mp);
    ~Features(){}

    virtual void ParseXML(XMLNode& in_Node);

    MPFeature* GetFeature(string s);

    int GetFeatureIndex(string s);

    vector<vector<double> > Collect(vector<VID>& vids);
    vector<vector<double> > Collect(vector<string> features, vector<VID>& vids);

  private:
    vector<MPFeature*> all;
    vector<MPFeature*> selected;
};

#endif
