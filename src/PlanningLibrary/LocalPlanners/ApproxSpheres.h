#ifndef APPROX_SPHERES_H
#define APPROX_SPHERES_H

#include "LocalPlannerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ApproxSpheres: public LocalPlannerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    ApproxSpheres(bool _saveIntermediates = false,
        const ClearanceUtility<MPTraits>& _c = ClearanceUtility<MPTraits>());

    ApproxSpheres(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& col,
        LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision = true,
        bool _savePath = false);

  protected:
    ClearanceUtility<MPTraits> m_clearUtil; //Clearance Utility
};

//Definitions for Constructors and Destructor
template<class MPTraits>
ApproxSpheres<MPTraits>::
ApproxSpheres(bool _saveIntermediates, const ClearanceUtility<MPTraits>& _c) :
  LocalPlannerMethod<MPTraits>(_saveIntermediates), m_clearUtil(_c) {
    this->SetName("ApproxSpheres");
  }

template<class MPTraits>
ApproxSpheres<MPTraits>::
ApproxSpheres(MPProblemType* _problem, XMLNode& _node) :
  LocalPlannerMethod<MPTraits>(_problem, _node),
  m_clearUtil(_problem, _node) {
    this->SetName("ApproxSpheres");
    if(!m_clearUtil.GetExactClearance())
      throw ParseException(_node.Where(),
          "Clearance type needs to be 'exact'.");
  }

template<class MPTraits>
void
ApproxSpheres<MPTraits>::
Print(ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  m_clearUtil.Print(_os);
  _os << "\n\tDistance Metric Label: " << m_clearUtil.GetDistanceMetricLabel()
    << " " << endl;
}

template<class MPTraits>
bool
ApproxSpheres<MPTraits>::
IsConnected(const CfgType& _c1, const CfgType& _c2, CfgType& col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision,
    bool _savePath) {
  StatClass* _stats = this->GetStatClass();
  DistanceMetricPointer _dm =
    this->GetDistanceMetric(m_clearUtil.GetDistanceMetricLabel());
  Environment* _env = this->GetEnvironment();

  //clear lpOutput
  _lpOutput->Clear();

  _stats->IncLPAttempts( this->GetNameAndLabel() );
  int cdCounter = 0;

  double dist, c1Clearance, c2Clearance;
  dist = 0;
  c1Clearance = 0;
  c2Clearance = 0;

  //calculate the distance between the two cfgs
  dist = _dm->Distance(_c1,_c2);

  CDInfo c1Info, c2Info;
  CfgType c1 = _c1;
  CfgType c2 = _c2;
  CfgType tmp;

  //Get clearance for both cfgs
  m_clearUtil.CollisionInfo(c1, tmp, _env->GetBoundary(), c1Info);
  c1Clearance = c1Info.m_minDist;

  m_clearUtil.CollisionInfo(c2, tmp, _env->GetBoundary(), c2Info);
  c2Clearance = c2Info.m_minDist;

  _stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);

  //Check if clearance is greater than Distance, if so, return true, else return
  //false
  if (c1Clearance + c2Clearance >= dist) {
    _stats->IncLPConnections(this->GetNameAndLabel());
    return true;
  }
  else
    return false;
}

#endif
