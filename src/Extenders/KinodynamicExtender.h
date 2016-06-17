#ifndef BASICEXTENDER_H_
#define BASICEXTENDER_H_

#include "ExtenderMethod.h"

#include "Environment/NonHolonomicMultiBody.h"

#include "Environment/Control.h"

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

    virtual double GetDelta() const {return m_delta;}

    virtual bool Extend(const StateType& _near, const StateType& _dir,
        StateType& _new, LPOutput<MPTraits>& _lpOutput);

  protected:
    bool ExtendBestControl(const StateType&, const StateType&, size_t, double, StateType&, LPOutput<MPTraits>&);
    bool ExtendRandomControl(const StateType&, const StateType&, size_t, double, StateType&, LPOutput<MPTraits>&);

    void SetOutput(const string&, size_t, const vector<double>&, bool, LPOutput<MPTraits>&);
    string m_dmLabel;
    string m_vcLabel;
    double m_delta;   ///< Time step
    bool m_fixed;     ///< True is fixed step at m_delta
                      ///< false is varyiable time-step in (0, m_delta)
    bool m_best;      ///< True is best control selection
                      ///< false is random control selection
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

  m_delta = _node.Read("delta", true, 10.0, 0.0, MAX_DBL,
      "Delta Time as multiple of environment resolution");
  m_fixed = _node.Read("fixed", true, true,
      "Fixed time-step or variable time-step.");
  m_best = _node.Read("best", true, false, "Best control or random control.");
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
KinodynamicExtender<MPTraits>::
Extend(const StateType& _near, const StateType& _dir, StateType& _new,
    LPOutput<MPTraits>& _lpOutput) {

  //Setup...primarily for collision checks that occur later on
  Environment* env = this->GetEnvironment();

  double delta = m_fixed ? m_delta : m_delta*DRand();
  size_t nTicks = ceil(delta);
  double dt = delta*env->GetTimeRes()/nTicks;


  if(m_best) {
    return ExtendBestControl(_near, _dir, nTicks, dt, _new, _lpOutput);
  }
  else {
    return ExtendRandomControl(_near, _dir, nTicks, dt, _new, _lpOutput);
  }
  return false;
}

template<typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ExtendBestControl(const StateType& _near, const StateType& _dir, size_t _nTicks, double _dt, StateType& _new,
      LPOutput<MPTraits>& _lpOutput) {

  string callee("KinodynamicExtender::Expand");

  Environment* env = this->GetEnvironment();
  shared_ptr<NonHolonomicMultiBody> robot = dynamic_pointer_cast<NonHolonomicMultiBody>(env->GetRobot(0));
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);



  const vector<shared_ptr<Control>>& control = robot->AvailableControls();
  double distBest = numeric_limits<double>::infinity();

  for(auto& c : control) {
    //reset variables
    StateType tick = _near;
    size_t ticker = 0;
    bool collision = false;
    _lpOutput.m_intermediates.clear();

    //apply control
    const vector<double>& cont = c->GetControl();
    while(!collision && ticker < _nTicks) {
      tick = tick.Apply(cont, _dt);
      if(!env->InBounds(tick) || !vc->IsValid(tick, callee)) {
        collision = true;
        ++ticker;
        _lpOutput.m_intermediates.push_back(tick);
      }

      //if success, save
      if(!collision) {
        double dist = dm->Distance(tick, _dir);
        if(dist < distBest) {
          distBest = dist;
          _new = tick;
          SetOutput("RRTExpand", _nTicks, cont, true, _lpOutput);
	  _lpOutput.AddIntermediatesToWeights(true);
	}
      }
    }
  }
  if(distBest == numeric_limits<double>::infinity()) {
    return false;
  }
  else {
    return true;
  }
  return false;
}

template<typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ExtendRandomControl(const StateType& _near, const StateType& _dir, size_t _nTicks, double _dt, StateType& _new,
      LPOutput<MPTraits>& _lpOutput) {
  string callee("KinodynamicExtender::Expand");

  Environment* env = this->GetEnvironment();

  shared_ptr<NonHolonomicMultiBody> robot = dynamic_pointer_cast<NonHolonomicMultiBody>(env->GetRobot(0));
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);


  StateType tick = _near;
  size_t ticker = 0;
  bool collision = false;

  vector<double> control = robot->GetRandomControl();

  while(!collision && ticker < _nTicks) {

    tick = tick.Apply(control, _dt);
    if(!env->InBounds(tick) || !vc->IsValid(tick, callee))
      collision = true; //return previous tick, as it is collision-free
      ++ticker;
      _lpOutput.m_intermediates.push_back(tick);
    }
    if(!collision) {
      _new = tick;
      SetOutput("RRTExpand", _nTicks, control, true, _lpOutput);
      return true;
    }
    else
      return false;
}

template<typename MPTraits>
void
KinodynamicExtender<MPTraits>::
SetOutput(const string& _label, size_t _nTicks, const vector<double>& _control, bool _add, LPOutput<MPTraits>& _lpOutput) {
     _lpOutput.SetLPLabel(_label);
    _lpOutput.m_edge.first.SetWeight(_nTicks);
    _lpOutput.m_edge.second.SetWeight(_nTicks);
    _lpOutput.m_edge.first.SetControl(_control);
    _lpOutput.m_edge.first.SetTimeStep(_nTicks);
    _lpOutput.AddIntermediatesToWeights(_add);
}

#endif
