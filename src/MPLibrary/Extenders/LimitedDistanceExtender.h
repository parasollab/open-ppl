#ifndef LIMITED_DISTANCE_EXTENDER_H_
#define LIMITED_DISTANCE_EXTENDER_H_

#include "KinodynamicExtender.h"


////////////////////////////////////////////////////////////////////////////////
/// Limited Distance Extender for nonholonomic robots.
/// Extensions are limited by fixed or variable euclidean distance instead of
/// time steps.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LimitedDistanceExtender : public KinodynamicExtender<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    LimitedDistanceExtender(const string& _dmLabel = "",
        const string& _vcLabel = "", const size_t _minTimeSteps = 1,
        const double _min = .001, const size_t _steps = 10,
        const bool _fixed = true, const bool _best = false);

    LimitedDistanceExtender(XMLNode& _node);

    virtual ~LimitedDistanceExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name KinodynamicExtender Overrides
    ///@{

    /// Do not compute the value as in KinodynamicExtender.
    virtual double GetMaxDistance() const override;

  protected:

    /// Extend with a given control.
    /// @param _start The configuration to start from.
    /// @param _end   The target configuration.
    /// @param _new   The output for the newly extended configuration.
    /// @param _lp    The output for the extension weight/intermediates.
    /// @param _steps The number of timesteps to use.
    /// @param _control The control to use.
    /// @return True if the extension went at least the minimum distance.
    virtual bool ApplyControl(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp, const size_t _steps, const
        Control& _control) const override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_minControlTime{0};  ///< The minimum time between controls.
    size_t m_minTimeSteps{1};    ///< The minimum number of time steps permitted.

    /// In this class, this indicates fixed or variable *distance* rather than
    /// time interval.
    using KinodynamicExtender<MPTraits>::m_fixed;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LimitedDistanceExtender<MPTraits>::
LimitedDistanceExtender(const string& _dmLabel, const string& _vcLabel,
    const size_t _minTimeSteps, const double _min, const size_t _steps,
    const bool _fixed, const bool _best)
    : KinodynamicExtender<MPTraits>(_dmLabel, _vcLabel, _min, _steps, _fixed,
        _best), m_minTimeSteps(_minTimeSteps) {
  this->SetName("LimitedDistanceExtender");
}


template <typename MPTraits>
LimitedDistanceExtender<MPTraits>::
LimitedDistanceExtender(XMLNode& _node) : KinodynamicExtender<MPTraits>(_node) {
  this->SetName("LimitedDistanceExtender");

  m_fixed = _node.Read("fixed", true, true, "Fixed or variable distance.");

  // Read the num steps parameter with an impossible bound to make it clear that
  // you can't limit the total steps for this extender.
  _node.Read("numSteps", false, size_t(10), size_t(1), size_t(0),
      "Cannot specify the total timesteps for this object.");

  // Re-read the maxDist parameter with required = true so that we get an error
  // if it isn't defined.
  _node.Read("maxDist", true, 1., 0.,
      std::numeric_limits<double>::max(), "A maximum distance must be specified "
      "for this object.");

  m_minControlTime = _node.Read("minControlTime", true, 0.,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "Minimum time between commands for the robot (in seconds).");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
LimitedDistanceExtender<MPTraits>::
Initialize() {
  // Compute the minimum number of time steps needed to ensure the robot can
  // actually execute the generated commands.
  m_minTimeSteps = std::ceil(m_minControlTime /
                             this->GetEnvironment()->GetTimeRes());
}

/*------------------------- KinodynamicExtender Overrides --------------------*/

template <typename MPTraits>
double
LimitedDistanceExtender<MPTraits>::
GetMaxDistance() const {
  return ExtenderMethod<MPTraits>::GetMaxDistance();
}


template <typename MPTraits>
bool
LimitedDistanceExtender<MPTraits>::
ApplyControl(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, const size_t, const Control& _control)
    const {
  MethodTimer mt(this->GetStatClass(), "LimitedDistanceExtender::ApplyControl");

  if(this->m_debug)
    std::cout << "\tTrying control " << _control << std::endl;

  auto dm = this->GetDistanceMetric(this->m_dmLabel);
  auto vc = this->GetValidityChecker(this->m_vcLabel);
  auto robot = _start.GetRobot();
  auto dynamicsModel = robot->GetDynamicsModel();

  // Get the length of each sub-step.
  auto env = this->GetEnvironment();
  const double dt = env->GetTimeRes();

  // Let _new represent the last known good cfg.
  _new = _start;

  // First, apply the control(s) for the minimum number of time steps.
  // Keep track of euclidean distance.
  double distance = 0;
  size_t i = 0;
  for(; i < m_minTimeSteps; ++i) {
     // Advance from the last known good cfg.
    CfgType temp = dynamicsModel->Test(_new, _control, dt);

    // Quit if we collided.
    if(!temp.InBounds(env) or !vc->IsValid(temp, "LimitedDistanceExtender"))
      break;

    // Update distance, intermediates, and last known good cfg.
    distance += dm->Distance(_new, temp);
    if(i > 0)
      _lp.m_intermediates.push_back(_new);
    _new = temp;
  }

  double maxDist = this->GetMaxDistance();

  // Check if we are using a fixed or variable distance limit.
  if(!m_fixed) {
    const double delta = maxDist - distance;
    maxDist = distance + delta * DRand();
  }

  while(distance < maxDist) {
    // Advance from the last known good cfg.
    CfgType temp = dynamicsModel->Test(_new, _control, dt);

    // Quit if we collided.
    if(!temp.InBounds(env) or !vc->IsValid(temp, "LimitedDistanceExtender"))
      break;

    // Update distance.
    const double extendDist = dm->Distance(_new, temp);
    distance += extendDist;

    // Stop stepping if we've reached the distance limit or if we aren't making
    // progress. The latter occurs when the coast control is being applied while
    // the robot has 0 velocity causing this loop to be infinite.
    if(distance > maxDist or extendDist == 0)
      break;

    // Update intermediates and last known good Cfg.
    _lp.m_intermediates.push_back(_new);
    _new = temp;
    ++i;
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
              << " units." << std::endl;

  return valid;
}


#endif
