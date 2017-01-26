#ifndef KINODYNAMIC_EXTENDER_H_
#define KINODYNAMIC_EXTENDER_H_

#include "ExtenderMethod.h"

#include "MPProblem/Robot/Control.h"
#include "MPProblem/Robot/DynamicsModel.h"


////////////////////////////////////////////////////////////////////////////////
/// Extender for nonholonomic robots.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class KinodynamicExtender : public ExtenderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    KinodynamicExtender(const string& _dmLabel = "", const string& _vcLabel = "",
        const double _min = .001, const size_t _steps = 10,
        const bool _fixed = true, const bool _best = false);

    KinodynamicExtender(XMLNode& _node);

    virtual ~KinodynamicExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    /// Compute the maximum distance that this extender could push a
    /// configuration on first request.
    virtual double GetMaxDistance() const override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Try each discrete control and choose the one that produces a
    /// configuration closest to _end.
    /// @param _start The configuration to start from.
    /// @param _end   The target configuration.
    /// @param _new   The output for the newly extended configuration.
    /// @param _lp    The output for the extension weight/intermediates.
    /// @param _steps The number of timesteps to use.
    /// @return True if the best extension went at least the minimum distance.
    bool ExtendBestControl(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp, const size_t _steps) const;

    /// Extend with a random control.
    /// @param _start The configuration to start from.
    /// @param _end   The target configuration.
    /// @param _new   The output for the newly extended configuration.
    /// @param _lp    The output for the extension weight/intermediates.
    /// @param _steps The number of timesteps to use.
    /// @return True if the extension went at least the minimum distance.
    bool ExtendRandomControl(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp, const size_t _steps) const;

    /// Extend with a given control.
    /// @param _start The configuration to start from.
    /// @param _end   The target configuration.
    /// @param _new   The output for the newly extended configuration.
    /// @param _lp    The output for the extension weight/intermediates.
    /// @param _steps The number of timesteps to use.
    /// @param _control The control to use.
    /// @return True if the extension went at least the minimum distance.
    bool ApplyControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
        LPOutput<MPTraits>& _lp, const size_t _steps, const Control& _control)
        const;

    ///@}
    ///@name MPObject Labels
    ///@{

    string m_dmLabel;  ///< The distance metric to use.
    string m_vcLabel;  ///< The validity checker to use.

    ///@}
    ///@name Extender Properties
    ///@{

    size_t m_numSteps; ///< The number of time resolutions to use for a step.
    bool m_fixed;      ///< True for fixed time step, false for variable.
    bool m_best;       ///< True for best control, false for random control.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
KinodynamicExtender<MPTraits>::
KinodynamicExtender(const string& _dmLabel, const string& _vcLabel,
    const double _min, const size_t _steps, const bool _fixed, const bool _best) :
    ExtenderMethod<MPTraits>(_min), m_dmLabel(_dmLabel), m_vcLabel(_vcLabel),
    m_numSteps(_steps), m_fixed(_fixed), m_best(_best) {
  this->SetName("KinodynamicExtender");
}


template <typename MPTraits>
KinodynamicExtender<MPTraits>::
KinodynamicExtender(XMLNode& _node) : ExtenderMethod<MPTraits>(_node) {
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
  m_numSteps = _node.Read("numSteps", true, size_t(10), size_t(1),
      std::numeric_limits<size_t>::max(), "Maximum number of timesteps to take "
      "for each extension");
  m_fixed = _node.Read("fixed", true, true, "Fixed time-step or variable "
      "time-step");
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
      << "\tnum steps = " << m_numSteps << endl;
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

template <typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp) {
  // Number of sub-steps to take.
  const size_t numSteps = m_fixed ? m_numSteps :
      std::max(3., m_numSteps * ((DRand() + DRand()) / 2.));

  if(this->m_debug)
    std::cout << "Extending with " << (m_best ? "best" : "random")
              << " control and " << (m_fixed ? "fixed" : "variable")
              << " timestep " << std::setprecision(4)
              << numSteps * this->GetEnvironment()->GetTimeRes()
              << " (" << numSteps << " x " << this->GetEnvironment()->GetTimeRes()
              << ")." << std::endl;

  if(m_best)
    return ExtendBestControl(_start, _end, _new, _lp, numSteps);
  else
    return ExtendRandomControl(_start, _end, _new, _lp, numSteps);
}


template <typename MPTraits>
double
KinodynamicExtender<MPTraits>::
GetMaxDistance() const {
  static bool distanceCached = false;
  if(!distanceCached) {
    // Approximate max distance as the largest timestep * max velocity.
    const double time = this->GetEnvironment()->GetTimeRes() * m_numSteps;
    auto robot = this->GetTask()->GetRobot();
    const_cast<double&>(this->m_maxDist) = robot->GetMaxLinearVelocity() * time;
    distanceCached = true;
  }
  return this->m_maxDist;
}

/*------------------------------ Helpers -------------------------------------*/

template<typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ExtendBestControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps) const {
  auto dm = this->GetDistanceMetric(m_dmLabel);

  // Get the set of controls.
  auto robot = _start.GetRobot();
  auto controls = robot->GetControlSet();

  // Find the best control.
  double bestDistance = numeric_limits<double>::infinity();
  for(const auto& c : *controls) {
    if(this->m_debug)
      std::cout << "\tBest distance: " << bestDistance << std::endl;

    // Apply the control.
    CfgType temp(robot);
    LPOutput<MPTraits> lp;
    const bool valid = ApplyControl(_start, _end, temp, lp, _steps, c);

    // Discard invalid extensions.
    if(!valid)
      continue;

    // Check distance to the target.
    const double distance = dm->Distance(temp, _end);
    if(this->m_debug)
      std::cout << "\t\tDistance to goal: " << distance << std::endl;

    // If this control gets us closer to the target, it's the new best.
    if(distance < bestDistance) {
      bestDistance = distance;
      _new = temp;
      _lp = lp;
    }
  }

  return bestDistance != numeric_limits<double>::infinity();
}


template<typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ExtendRandomControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps) const {
  // Get a random control.
  auto robot = _start.GetRobot();
  auto controls = robot->GetControlSet();
  const size_t controlIndex = rand() % controls->size();

  // Apply the control.
  return ApplyControl(_start, _end, _new, _lp, _steps, (*controls)[controlIndex]);
}


template <typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ApplyControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps, const Control& _control)
    const {
  if(this->m_debug)
    std::cout << "\tTrying control " << _control << std::endl;

  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto robot = _start.GetRobot();
  auto dynamicsModel = robot->GetDynamicsModel();

  // Get the length of each sub-step.
  auto env = this->GetEnvironment();
  const double dt = env->GetTimeRes();

  // Let _new represent the last known good cfg.
  _new = _start;

  // Apply the control until:
  // a) We collide
  // b) We've done it _steps times
  double distance = 0;
  for(size_t i = 0; i < _steps; ++i) {
    // Advance from the last known good cfg.
    CfgType temp = dynamicsModel->Test(_new, _control, dt);

    // Quit if we collided.
    if(!env->InBounds(temp) or !vc->IsValid(temp, "KinodynamicExtender"))
      break;

    // Update distance, intermediates, and last known good cfg.
    distance += dm->Distance(_new, temp);
    if(i > 0)
      _lp.m_intermediates.push_back(_new);
    _new = temp;
  }

  _lp.SetLPLabel("RRTExpand");
  _lp.AddIntermediatesToWeights(true);
  _lp.m_edge.first.SetWeight(distance);
  _lp.m_edge.first.SetControl(_control);
//  _lp.m_edge.first.SetTimeStep(_steps);


  const bool valid = distance >= this->m_minDist;
  if(this->m_debug)
    std::cout << "\t\tExtension was " << (valid ? "valid" : "invalid") << "."
              << "\n\t\tExtended " << distance << " >= " << this->m_minDist
              << " units." << std::endl;
  return valid;
}

/*----------------------------------------------------------------------------*/

#endif
