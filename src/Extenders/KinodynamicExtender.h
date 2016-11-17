#ifndef KINODYNAMIC_EXTENDER_H_
#define KINODYNAMIC_EXTENDER_H_

#include "ExtenderMethod.h"

#include "Environment/NonHolonomicMultiBody.h"

#include "Environment/Control.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend from a start state by applying controls.
/// @tparam MPTraits Motion planning universe
///
/// Extends in straight-line through @cspace from \f$q_{near}\f$ towards
/// \f$q_{dir}\f$ until either \f$q_{dir}\f$ is reached, a distance of
/// \f$\Delta q\f$ is extended, or @cobst is reached.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class KinodynamicExtender : public ExtenderMethod<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       StateType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///\name Construction
    ///@{

    KinodynamicExtender(const string& _dmLabel = "", const string& _vcLabel = "",
        double _min = .001, double _timeStep = 1, bool _fixed = true,
        bool _best = false);

    KinodynamicExtender(MPProblemType* _problem, XMLNode& _node);

    virtual ~KinodynamicExtender() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;

    ///@}
    ///\name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const StateType& _start, const StateType& _end,
        StateType& _new, LPOutput<MPTraits>& _lp) override;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Compute the maximum distance that this extender could push a
    ///        configuration on first request.
    virtual double GetMaxDistance() const override;

    ///@}
    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Applies a single control at a starting state.
    /// \param[in]  _start      The start configuration.
    /// \param[in]  _con        The Control used.
    /// \param[out] _collision  Determine if the control is valid.
    StateType ApplyControl(const StateType& _start, const vector<double>& _con,
        bool& _collision);

    /// @}

  protected:

    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Try each control and choose the one that produces a configuration
    ///        closest to _end.
    bool ExtendBestControl(const StateType& _start, const StateType& _end,
        size_t _ticks, double _dt, StateType& _new,
        LPOutput<MPTraits>& _lp) const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Extend with a random control.
    bool ExtendRandomControl(const StateType& _start, const StateType& _end,
        size_t _ticks, double _dt, StateType& _new,
        LPOutput<MPTraits>& _lp) const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Applies a single control at a starting state for a fixed
    //         timestep.
    /// \param[in]  _start      The start configuration.
    /// \param[in]  _con        The Control used.
    /// \param[in]  _nTick      The number of tick in the extention
    /// \param[in]  _dt         The change in time of each extention
    /// \param[out] _lp         The ouput of the local planner
    /// \param[out] _collision  Determine if the control is valid.
    StateType ApplyControl(const StateType& _start, const vector<double>& _con,
        double _nTick, double _dt, LPOutput<MPTraits>& _lp,
        bool& _collision) const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Set the lpOutput data, including label, timestep, distance, and
    ///        intermediates.
    /// \param[in]  _ticks   The number of ticks in this extension.
    /// \param[in]  _control The control used to generate this extension.
    /// \param[in]  _start   The start configuration.
    /// \param[in]  _end     The ending configuration.
    /// \param[out] _lp      The lpOutput object to set.
    void SetOutput(size_t _ticks, const vector<double>& _control,
        const StateType& _start, const StateType& _end,
        LPOutput<MPTraits>& _lp) const;

    ///@}
    ///\name MPObject Labels
    ///@{

    string m_dmLabel;  ///< The distance metric to use.
    string m_vcLabel;  ///< The validity checker to use.

    ///@}
    ///\name Extender Properties
    ///@{

    double m_timeStep; ///< The number of time resolutions to use for a step.
    bool m_fixed;      ///< True for fixed time step, false for variable.
    bool m_best;       ///< True for best control, false for random control.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
KinodynamicExtender<MPTraits>::
KinodynamicExtender(const string& _dmLabel, const string& _vcLabel,
    double _min, double _timeStep, bool _fixed, bool _best) :
    ExtenderMethod<MPTraits>(_min), m_dmLabel(_dmLabel), m_vcLabel(_vcLabel),
    m_timeStep(_timeStep), m_fixed(_fixed), m_best(_best) {
  this->SetName("KinodynamicExtender");
}


template <typename MPTraits>
KinodynamicExtender<MPTraits>::
KinodynamicExtender(MPProblemType* _problem, XMLNode& _node) :
    ExtenderMethod<MPTraits>(_problem, _node) {
  this->SetName("KinodynamicExtender");
  ParseXML(_node);
}

/*---------------------------- MPBaseObject Overrides ------------------------*/

template <typename MPTraits>
void
KinodynamicExtender<MPTraits>::
ParseXML(XMLNode& _node) {
  m_dmLabel = _node.Read("dmLabel",true,"", "Distance metric label");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
  m_timeStep = _node.Read("timeStep", true, 10.0, 0.0, MAX_DBL, "Delta Time as "
      "multiple of environment resolution");
  m_fixed = _node.Read("fixed", true, true, "Fixed time-step or variable "
      "time-step.");
  m_best = _node.Read("best", true, false, "Best control or random control");

  // Ensure maxDist isn't used by requiring an impossible bound during parsing.
  _node.Read("maxDist", false, 0., 0., -1., "Max distance can't be specified "
      "for this object, it is computed automatically");
}


template <typename MPTraits>
void
KinodynamicExtender<MPTraits>::
Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\tdistance metric : \"" << m_dmLabel << "\"" << endl
      << "\tvalidity checker : \"" << m_vcLabel << "\"" << endl
      << "\ttimeStep = " << m_timeStep << endl;
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

template <typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
Extend(const StateType& _start, const StateType& _end, StateType& _new,
    LPOutput<MPTraits>& _lp) {
  size_t nTicks = ceil(m_timeStep);
  double dt = m_timeStep * this->GetEnvironment()->GetTimeRes() / nTicks;

  if(!m_fixed)
    nTicks = max(nTicks / 2., nTicks * (DRand() + DRand()) / 2.);

  if(m_best)
    return ExtendBestControl(_start, _end, nTicks, dt, _new, _lp);
  else
    return ExtendRandomControl(_start, _end, nTicks, dt, _new, _lp);
}


template <typename MPTraits>
double
KinodynamicExtender<MPTraits>::
GetMaxDistance() const {
  static bool distanceCached = false;
  if(!distanceCached) {
    // Approximate max distance as the largest timestep * max velocity.
    auto env = this->GetEnvironment();
    shared_ptr<NonHolonomicMultiBody> robot =
        dynamic_pointer_cast<NonHolonomicMultiBody>(env->GetRobot(0));
    const_cast<double&>(this->m_maxDist) = robot->GetMaxLinearVelocity() *
        m_timeStep * env->GetTimeRes();
    distanceCached = true;
  }
  return this->m_maxDist;
}

/*------------------------------ Helpers -------------------------------------*/

template <typename MPTraits>
typename KinodynamicExtender<MPTraits>::StateType
KinodynamicExtender<MPTraits>::
ApplyControl(const StateType& _start, const vector<double>& _con,
    bool& _collision) {
  size_t nTicks = ceil(m_timeStep);
  double dt = m_timeStep * this->GetEnvironment()->GetTimeRes() / nTicks;
  LPOutput<MPTraits> lp; // if lp output is need then use the other funtion
  return ApplyControl(_start, _con, nTicks, dt, lp, _collision);
}

template <typename MPTraits>
typename KinodynamicExtender<MPTraits>::StateType
KinodynamicExtender<MPTraits>::
ApplyControl(const StateType& _start, const vector<double>& _con,
        double _nTicks, double _dt, LPOutput<MPTraits>& _lp,
        bool& _collision) const{

  string callee("KinodynamicExtender::ApplyControl");
  auto env = this->GetEnvironment();
  StateType tick = _start;
  size_t ticker = 0;
  bool collision = false;

  auto vc = this->GetValidityChecker(m_vcLabel);

  // applies the control
  while(!collision && ticker < _nTicks) {
    tick = tick.Apply(_con, _dt);
    if(!env->InBounds(tick) || !vc->IsValid(tick, callee))
      collision = true;
    ++ticker;
    _lp.m_intermediates.push_back(tick);
  }

  _collision = collision;
  return tick;
}

template <typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ExtendBestControl(const StateType& _start, const StateType& _end, size_t _nTicks,
    double _dt, StateType& _new, LPOutput<MPTraits>& _lp) const {
  string callee("KinodynamicExtender::Expand");

  Environment* env = this->GetEnvironment();
  shared_ptr<NonHolonomicMultiBody> robot =
      dynamic_pointer_cast<NonHolonomicMultiBody>(env->GetRobot(0));

  StateType tick;
  auto dm = this->GetDistanceMetric(m_dmLabel);

  const vector<shared_ptr<Control>>& control = robot->AvailableControls();
  double distBest = numeric_limits<double>::infinity();

  for(auto& c : control) {
    bool collision = false;

    //reset variables
    LPOutput<MPTraits> lp;

    //apply control
    tick = ApplyControl(_start, c->GetControl(), _nTicks, _dt, lp, collision);

    // If successful and better than the current best, save this extension.
    if(!collision) {
      double dist = dm->Distance(tick, _end);
      if(dist < distBest) {
        distBest = dist;
        _new = tick;
        _lp.m_intermediates = lp.m_intermediates;
        SetOutput(_nTicks, c->GetControl(), _start, _new, _lp);
      }
    }
  }
  return distBest != numeric_limits<double>::infinity() &&
         distBest >= this->m_minDist;
}


template <typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ExtendRandomControl(const StateType& _start, const StateType& _end,
    size_t _nTicks, double _dt, StateType& _new, LPOutput<MPTraits>& _lp) const {

  Environment* env = this->GetEnvironment();

  shared_ptr<NonHolonomicMultiBody> robot =
      dynamic_pointer_cast<NonHolonomicMultiBody>(env->GetRobot(0));

  StateType tick = _start;
  bool collision = false;
  const vector<double>& control = robot->GetRandomControl();
  tick = ApplyControl(_start, control, _nTicks, _dt, _lp, collision);

  if(!collision) {
    _new = tick;
    SetOutput(_nTicks, control, _start, _new, _lp);
    return _lp.m_edge.first.GetWeight() >= this->m_minDist;
  }
  else
    return false;
}


template <typename MPTraits>
void
KinodynamicExtender<MPTraits>::
SetOutput(size_t _nTicks, const vector<double>& _control,
    const StateType& _start, const StateType& _end,
    LPOutput<MPTraits>& _lp) const {
  _lp.SetLPLabel("RRTExpand");
  _lp.m_edge.first.SetControl(_control);
  _lp.m_edge.first.SetTimeStep(_nTicks);
  _lp.AddIntermediatesToWeights(true);

  double dist = 0;
  auto& intermediates = _lp.m_intermediates;

  // Use the distance metric to compute the actual path length.
  auto dm = this->GetDistanceMetric(m_dmLabel);
  if(intermediates.empty())
    dist = dm->Distance(_start, _end);
  else {
    dist = dm->Distance(_start, intermediates.front());
    for(auto cit1 = intermediates.begin(), cit2 = cit1 + 1;
        cit2 != intermediates.end(); ++cit1, ++cit2)
      dist += dm->Distance(*cit1, *cit2);
    dist += dm->Distance(intermediates.back(), _end);
  }

  _lp.m_edge.first.SetWeight(dist);
  _lp.m_edge.second.SetWeight(dist);
}

/*----------------------------------------------------------------------------*/

#endif
