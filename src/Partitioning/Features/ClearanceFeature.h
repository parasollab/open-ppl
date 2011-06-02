#ifndef CLEARANCEFEATURE_H_
#define CLEARANCEFEATURE_H_

#include "MPFeature.h"
#include "DistanceMetrics.h"

class ClearanceFeature : public MPFeature {
  public:
    ClearanceFeature();
    ClearanceFeature(string _vc);
    ClearanceFeature(XMLNodeReader& in_Node, MPProblem* in_PProblem);
    virtual ~ClearanceFeature(){}

    virtual void ParseXML(XMLNodeReader& in_Node);

    virtual vector<double> Collect(vector<VID>& vids);
  private:
    string m_vc;
};

class CSpaceClearanceFeature : public MPFeature {
  public:
    CSpaceClearanceFeature();
    CSpaceClearanceFeature(string vc, string dm);
    CSpaceClearanceFeature(XMLNodeReader& in_Node, MPProblem* in_PProblem);
    virtual ~CSpaceClearanceFeature(){}

    virtual void ParseXML(XMLNodeReader& in_Node);

    virtual vector<double> Collect(vector<VID>& vids);


    double ApproxCSpaceClearance(Environment* env, Stat_Class& Stats,
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

