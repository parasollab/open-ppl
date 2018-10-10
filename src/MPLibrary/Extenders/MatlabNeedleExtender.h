#ifndef PMPL_MATLAB_NEEDLE_EXTENDER_H_
#define PMPL_MATLAB_NEEDLE_EXTENDER_H_

#include "ExtenderMethod.h"

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Robot/Control.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/MatlabMicroSimulator.h"


////////////////////////////////////////////////////////////////////////////////
/// Extender for the matlab-based needle model.
///
/// @warning Controls mean something entirely different for this kind of robot -
///          they are not generalized DOF forces/velocities.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MatlabNeedleExtender : public ExtenderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    MatlabNeedleExtender();

    MatlabNeedleExtender(XMLNode& _node);

    virtual ~MatlabNeedleExtender() = default;

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
    bool ApplyControl(const CfgType& _start, const CfgType& _end,
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

    size_t m_numSteps{3}; ///< The number of insertion steps to use for each extension.
    double m_insertionStep{.005}; ///< Insertion amount for each step.

    bool m_fixed{true};    ///< True for fixed time step, false for variable.
    bool m_best{false};    ///< True for best control, false for random control.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
MatlabNeedleExtender<MPTraits>::
MatlabNeedleExtender() {
  this->SetName("MatlabNeedleExtender");
}


template <typename MPTraits>
MatlabNeedleExtender<MPTraits>::
MatlabNeedleExtender(XMLNode& _node) : ExtenderMethod<MPTraits>(_node) {
  this->SetName("MatlabNeedleExtender");

  m_dmLabel = _node.Read("dmLabel",true,"", "Distance metric label.");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label.");

  m_numSteps = _node.Read("numSteps", true, m_numSteps,
      size_t(1), std::numeric_limits<size_t>::max(),
      "Maximum number of insertion steps to take for each extension.");
  m_fixed = _node.Read("fixed", true, m_fixed,
      "Fixed or variable number of insertion steps.");
  m_best = _node.Read("best", true, m_best, "Best control or random control.");
}

/*---------------------------- MPBaseObject Overrides ------------------------*/

template <typename MPTraits>
void
MatlabNeedleExtender<MPTraits>::
Initialize() {
}


template <typename MPTraits>
void
MatlabNeedleExtender<MPTraits>::
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
MatlabNeedleExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp) {
  // Number of sub-steps to take. We average two random numbers to bias
  // selection toward the middle of the range.
  const size_t numSteps = m_fixed ? m_numSteps
                                  : std::max(1., m_numSteps * DRand());

  if(this->m_debug)
    std::cout << "Extending with " << (m_best ? "best" : "random")
              << " control and " << (m_fixed ? "fixed" : "variable")
              << " insertion steps " << std::setprecision(4)
              << numSteps * m_insertionStep
              << " (" << numSteps << " x " << m_insertionStep << ")."
              << std::endl;

  if(m_best)
    return ExtendBestControl(_start, _end, _new, _lp, numSteps);
  else
    return ExtendRandomControl(_start, _end, _new, _lp, numSteps);
}

/*------------------------------ Helpers -------------------------------------*/

template<typename MPTraits>
bool
MatlabNeedleExtender<MPTraits>::
ExtendBestControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps) const {
  // Ask the controller to find the best control.
  auto robot = _start.GetRobot();
  auto controller = robot->GetController();
  // For this controller, we will pass the number of steps instead of the time
  // length of the steps.
  const Control c = (*controller)(_start, _end, _steps);

  // Apply the control.
  return ApplyControl(_start, _end, _new, _lp, _steps, c);
}


template<typename MPTraits>
bool
MatlabNeedleExtender<MPTraits>::
ExtendRandomControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps) const {
  // Get a random control.
  auto robot = _start.GetRobot();

  // For this controller, we will pass the number of steps instead of the time
  // length of the steps.
  auto controller = robot->GetController();
  const Control c = controller->GetRandomControl(_start, _steps);

  // Apply the control.
  return ApplyControl(_start, _end, _new, _lp, _steps, c);
}


template <typename MPTraits>
bool
MatlabNeedleExtender<MPTraits>::
ApplyControl(const CfgType& _start, const CfgType&, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t _steps, const Control& _control)
    const {
  MethodTimer mt(this->GetStatClass(), "MatlabNeedleExtender::ApplyControl");

  if(this->m_debug)
    std::cout << "\tTrying control " << _control << std::endl;

  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto robot = _start.GetRobot();
  auto simulator = robot->GetMatlabMicroSimulator();

  // Get the length of each sub-step.
  auto env = this->GetEnvironment();

  // Let _new represent the last known good cfg.
  _new = _start;

  // Apply the control until:
  // a) We collide
  // b) We've done it _steps times
  double distance = 0;
  size_t i = 0;
  CfgType temp;
  for(; i < _steps; ++i) {
    // Advance from the last known good cfg.
    temp = simulator->Test(_new, _control);

    // Quit if we collided.
    if(!temp.InBounds(env) or !vc->IsValid(temp, "MatlabNeedleExtender"))
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
              << "\n\t\tTemp: " << temp.PrettyPrint()
              << std::endl;

  this->GetStatClass()->SetStat("MatlabTime", simulator->GetMatlabTime());
  _new.m_witnessCfg.reset(new CfgType(_start));

  return valid;
}

/*----------------------------------------------------------------------------*/

#endif
