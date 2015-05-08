#ifndef CLEARANCEFEATURE_H_
#define CLEARANCEFEATURE_H_

#include "MPFeature.h"
#include "DistanceMetrics.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class ClearanceFeature : public MPFeature {
  public:
    ClearanceFeature();
    ClearanceFeature(string _vc);
    ClearanceFeature(XMLNode& in_Node, MPProblem* in_PProblem);
    virtual ~ClearanceFeature(){}

    virtual void ParseXML(XMLNode& in_Node);

    virtual vector<double> Collect(vector<VID>& vids);
  private:
    string m_vc;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class CSpaceClearanceFeature : public MPFeature {
  public:
    CSpaceClearanceFeature();
    CSpaceClearanceFeature(string vc, string dm);
    CSpaceClearanceFeature(XMLNode& in_Node, MPProblem* in_PProblem);
    virtual ~CSpaceClearanceFeature(){}

    virtual void ParseXML(XMLNode& in_Node);

    virtual vector<double> Collect(vector<VID>& vids);


    double ApproxCSpaceClearance(Environment* env, StatClass& Stats,
        CDInfo& cdInfo,
        DistanceMetric* dm, int n,
        bool bComputePenetration,
        MPProblem* mp,
        int ignore_obstacle = -1) const;

  private:
    string m_vc;
    string m_dm;
};

#endif

