#ifndef BASICEXTENDER_H_
#define BASICEXTENDER_H_

#include "ExtenderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Basic straight-line extension.
/// @tparam MPTraits Motion planning universe
///
/// Extends in straight-line through @cspace from \f$q_{near}\f$ towards
/// \f$q_{dir}\f$ until either \f$q_{dir}\f$ is reached, a distance of
/// \f$\Delta q\f$ is extended, or @cobst is reached.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class KinodynamicExtender : public ExtenderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType StateType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    KinodynamicExtender(const string& _dmLabel = "", const string& _vcLabel = "",
        double _delta = 1.0);
    KinodynamicExtender(MPProblemType* _problem, XMLNode& _node);

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual bool Extend(const StateType& _near, const StateType& _dir,
        StateType& _new, LPOutput<MPTraits>& _lpOutput);

  protected:
    string m_dmLabel;
    string m_vcLabel;
    double m_delta;
};

template<class MPTraits>
KinodynamicExtender<MPTraits>::KinodynamicExtender(const string& _dmLabel,
    const string& _vcLabel, double _delta) :
  ExtenderMethod<MPTraits>(), m_dmLabel(_dmLabel), m_vcLabel(_vcLabel),
  m_delta(_delta) {
    this->SetName("KinodynamicExtender");
  }

template<class MPTraits>
KinodynamicExtender<MPTraits>::KinodynamicExtender(MPProblemType* _problem,
    XMLNode& _node) :
  ExtenderMethod<MPTraits>(_problem, _node) {
    this->SetName("KinodynamicExtender");
    ParseXML(_node);
  }

template<class MPTraits>
void
KinodynamicExtender<MPTraits>::ParseXML(XMLNode& _node) {
  m_dmLabel = _node.Read("dmLabel",true,"", "Distance metric label");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
  m_delta = _node.Read("delta", false, 1.0, 0.0, MAX_DBL, "Delta distance");
}

template<class MPTraits>
void
KinodynamicExtender<MPTraits>::Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\tdistance metric : \"" << m_dmLabel << "\"" << endl;
  _os << "\tvalidity checker : \"" << m_vcLabel << "\"" << endl;
  _os << "\tdelta = " << m_delta << endl;
}

template<class MPTraits>
bool
KinodynamicExtender<MPTraits>::Extend(const StateType& _near, const StateType& _dir,
    StateType& _new, LPOutput<MPTraits>& _lpOutput) {

  //Setup...primarily for collision checks that occur later on
  Environment* env = this->GetEnvironment();
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
  string callee("KinodynamicExtender::Expand");

  StateType incr, tick = _near, previous = _near;
  bool collision = false;
  int nTicks, ticker = 0;

  incr.FindIncrement(tick, _dir, &nTicks,
      env->GetPositionRes(), env->GetOrientationRes());

  _lpOutput.m_edge.first.SetWeight(nTicks);
  _lpOutput.m_edge.second.SetWeight(nTicks);

  //Move out from start towards dir, bounded by number of ticks allowed at a
  //given resolution and the distance m_delta: the maximum distance to grow
  while(!collision && dm->Distance(_near, tick) <= m_delta &&
        ticker <= nTicks) {
    previous = tick;
    tick += incr;
    if(!env->InBounds(tick) || !(vc->IsValid(tick, callee)))
      collision = true; //return previous tick, as it is collision-free
    ++ticker;
  }
  if(previous != _near) {
    _new = previous;//Last Cfg pushed back is the final tick allowed
    return true;
  }
  else{
    if(this->m_debug)
      cout << "Could not expand !" << endl;
    return false;
  }
}

#endif
