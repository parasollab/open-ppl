#ifndef FEATURE_H_
#define FEATURE_H_

#include "Roadmap.h"
#include "CfgTypes.h"

typedef RoadmapGraph<CfgType,WeightType>::VID VID;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class MPFeature : public MPBaseObject {
  public:
    MPFeature():MPBaseObject(){}
    MPFeature(XMLNode& in_Node, MPProblem* mp):MPBaseObject(in_Node, mp){}
    virtual ~MPFeature(){}

    virtual void ParseXML(XMLNode& in_Node){}

    virtual vector<double> Collect(vector<VID>& vids)=0;
};

#endif
