#ifndef PMPL_KINODYNAMIC_EXTENDER_H_
#define PMPL_KINODYNAMIC_EXTENDER_H_

#include "ExtenderMethod.h"

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Robot/Control.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/MicroSimulator.h"


////////////////////////////////////////////////////////////////////////////////
/// Extender for nonholonomic robots. Extends an initial configuration by
/// applying a control for some number of time steps.
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

    KinodynamicExtender(const std::string& _dmLabel = "",
        const std::string& _vcLabel = "",
        const double _min = .001, const size_t _steps = 10,
        const bool _fixed = true, const bool _best = false);

    KinodynamicExtender(XMLNode& _node);

    virtual ~KinodynamicExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    /// Compute the maximum distance that this extender could push a
    /// configuration on first request.
    virtual double GetMaxDistance() const override;

    ///@}
    ///@name KinodynamicExtender Interface
    ///@{

    /// Apply a control to a starting state for m_numSteps.
    /// @param _start The start configuration.
    /// @param _con   The Control used.
    /// @return The resulting cfg after _start has been stepped m_numSteps
    ///         using _con as the control. If the extension fails, _start is
    ///         returned instead.
    CfgType ApplyControl(const CfgType& _start, const Control& _con);

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
    virtual bool ApplyControl(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp, const size_t _steps,
        const Control& _control) const;

    ///@}
    ///@name MPObject Labels
    ///@{

    std::string m_dmLabel;  ///< The distance metric to use.
    std::string m_vcLabel;  ///< The validity checker to use.

    ///@}
    ///@name Extender Properties
    ///@{

    size_t m_numSteps{10}; ///< The number of time resolutions to use for a step.
    bool m_fixed{true};    ///< True for fixed time step, false for variable.
    bool m_best{false};    ///< True for best control, false for random control.
    mutable bool m_distanceCached{false}; ///< Is the max distance cached?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
KinodynamicExtender<MPTraits>::
KinodynamicExtender(const std::string& _dmLabel, const std::string& _vcLabel,
    const double _min, const size_t _steps, const bool _fixed, const bool _best) :
    ExtenderMethod<MPTraits>(_min), m_dmLabel(_dmLabel), m_vcLabel(_vcLabel),
    m_numSteps(_steps), m_fixed(_fixed), m_best(_best) {
  this->SetName("KinodynamicExtender");
}


template <typename MPTraits>
KinodynamicExtender<MPTraits>::
KinodynamicExtender(XMLNode& _node) : ExtenderMethod<MPTraits>(_node) {
  this->SetName("KinodynamicExtender");

  m_dmLabel = _node.Read("dmLabel",true,"", "Distance metric label.");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label.");

  m_numSteps = _node.Read("numSteps", true, m_numSteps,
      size_t(1), std::numeric_limits<size_t>::max(),
      "Maximum number of time steps to take for each extension.");
  m_fixed = _node.Read("fixed", true, m_fixed,
      "Fixed or variable number of time steps.");
  m_best = _node.Read("best", true, m_best, "Best control or random control.");

  /// @TODO Ensure maxDist isn't used.
  //_node.Read("maxDist", true, 0., 0., -1., "Max distance can't be specified "
  //    "for this object, it is computed automatically.");
}

/*---------------------------- MPBaseObject Overrides ------------------------*/

template <typename MPTraits>
void
KinodynamicExtender<MPTraits>::
Initialize() {
  m_distanceCached = false;
}


template <typename MPTraits>
void
KinodynamicExtender<MPTraits>::
Print(std::ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\tdistance metric: " << m_dmLabel
      << "\n\tvalidity checker: " << m_vcLabel
      << "\n\tnum steps: " << m_numSteps << ", "
      << (m_fixed ? "fixed" : "variable")
      << "\n\tcontrol: " << (m_best ? "best" : "random")
      << std::endl;
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

template <typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp) {
  // Number of sub-steps to take. We average two random numbers to bias
  // selection toward the middle of the range.
  const size_t numSteps = m_fixed ? m_numSteps :
      std::max(1., m_numSteps * ((DRand() + DRand()) / 2.));

  if(this->m_debug)
    std::cout << "Extending with " << (m_best ? "best" : "random")
              << " control and " << (m_fixed ? "fixed" : "variable")
              << " timestep " << std::setprecision(4)
              << numSteps * this->GetEnvironment()->GetTimeRes()
              << " (" << numSteps << " x "
              << this->GetEnvironment()->GetTimeRes() << ")."
              << std::endl;

  if(m_best)
    return ExtendBestControl(_start, _end, _new, _lp, numSteps);
  else
    return ExtendRandomControl(_start, _end, _new, _lp, numSteps);
}


template <typename MPTraits>
double
KinodynamicExtender<MPTraits>::
GetMaxDistance() const {
  if(!m_distanceCached) {
    // Approximate max distance as the largest timestep * max velocity.
    const double time = this->GetEnvironment()->GetTimeRes() * m_numSteps;
    auto robot = this->GetTask()->GetRobot();
    const_cast<double&>(this->m_maxDist) = robot->GetMaxLinearVelocity() * time;
    m_distanceCached = true;
  }

  return this->m_maxDist;
}

/*------------------------------ Helpers -------------------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
KinodynamicExtender<MPTraits>::
ApplyControl(const CfgType& _start, const Control& _con) {
  auto robot = _start.GetRobot();
  CfgType temp(robot), end(robot);
  LPOutput<MPTraits> lp;

  if(ApplyControl(_start, end, temp, lp, m_numSteps, _con))
    return temp;
  else
    return _start;
}


template<typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ExtendBestControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps) const {
  auto env = this->GetEnvironment();

  // Ask the controller to find the best control.
  auto robot = _start.GetRobot();
  auto controller = robot->GetController();
  const Control c = (*controller)(_start, _end, _steps * env->GetTimeRes());

  // Apply the control.
  const bool valid = ApplyControl(_start, _end, _new, _lp, _steps, c);

  return valid;
}


template<typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ExtendRandomControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps) const {
  auto env = this->GetEnvironment();

  // Get a random control.
  auto robot = _start.GetRobot();
  auto controller = robot->GetController();
  const Control c = controller->GetRandomControl(_start,
      _steps * env->GetTimeRes());

  // Apply the control.
  return ApplyControl(_start, _end, _new, _lp, _steps, c);
}


template <typename MPTraits>
bool
KinodynamicExtender<MPTraits>::
ApplyControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps, const Control& _control)
    const {
  MethodTimer mt(this->GetStatClass(), "KinodynamicExtender::ApplyControl");

  if(this->m_debug)
    std::cout << "\tTrying control " << _control << std::endl;

  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto robot = _start.GetRobot();
  auto simulator = robot->GetMicroSimulator();

  // Get the length of each sub-step.
  auto env = this->GetEnvironment();
  const double dt = env->GetTimeRes();

  // Let _new represent the last known good cfg.
  _new = _start;

  // Apply the control until:
  // a) We collide
  // b) We've done it _steps times
  double distance = 0;
  size_t i = 0;
  for(; i < _steps; ++i) {
    // Advance from the last known good cfg.
    CfgType temp = simulator->Test(_new, _control, dt);

    // Quit if we collided.
    if(!temp.InBounds(env) or !vc->IsValid(temp, "KinodynamicExtender"))
      break;

    // Update distance, intermediates, and last known good cfg.
    distance += dm->Distance(_new, temp);
    if(i > 0 and i < _steps)
      _lp.m_intermediates.push_back(_new);
    _new = temp;
  }

  _lp.SetLPLabel("RRTExpand");
  _lp.AddIntermediatesToWeights(true);
  _lp.m_edge.first.SetWeight(distance);
  _lp.m_edge.first.SetControl(_control);
  _lp.m_edge.first.SetTimeSteps(i);

  const bool valid = distance >= this->m_minDist;

  if(this->m_debug)
    std::cout << "\t\tExtension was " << (valid ? "valid" : "invalid") << "."
              << "\n\t\tExtended " << distance
              << (valid ? " >= " : " < ") << this->m_minDist
              << " units."
              << "\n\t\tFrom: " << _start.PrettyPrint()
              << "\n\t\tTo:   " << _new.PrettyPrint()
              << std::endl;

  // There is some kind of error here which is producing ridiculous edges.
  if(distance > this->GetMaxDistance())
  {
    std::cerr << "Extended distance exceeds calculated maximum "
              << this->GetMaxDistance()
              << ", something is wrong."
              << std::endl;
    return false;
  }

  return valid;
}

/*----------------------------------------------------------------------------*/

#endif
