#ifndef ACTIVE_BODY_EXTENDER_H_
#define ACTIVE_BODY_EXTENDER_H_

#include "ExtenderMethod.h"
#include "MPProblem/RobotGroup/GroupUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Basic straight-line extension, allowing for the setting of active
///        bodies for assembly planning that will be moved together as a
///        subassembly. The first body in the vector is counted as the "leader"
///
/// This is a BROKEN disassembly planning extender. It will support subassembly
/// rotations once it is working.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ActiveBodyExtender : public ExtenderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    ActiveBodyExtender(const string& _dmLabel = "", const string& _vcLabel = "",
        double _min = .001, double _max = 1, bool _randomOrientation = true);

    ActiveBodyExtender(XMLNode& _node);

    virtual ~ActiveBodyExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    virtual void Initialize() const;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
           CfgType& _new, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo) override;

    ///@}

    void SetActiveBodies(const std::vector<unsigned int> _bodies) {
      m_activeBodies = _bodies;
    }

  protected:
    ///@name Helpers
    ///@{

    /// Basic utility for "extend" a RRT tree. Assumed to be given a start node
    /// and a goal node to grow towards. Resulting node extended towards the
    /// goal is passed by reference and modified.
    /// @param _start  Cfg to grow from.
    /// @param _end    Cfg to grow toward.
    /// @param _newCfg Return for newly created cfg.
    /// @param _delta  Maximum distance to grow
    /// @return True if the extension produced a valid configuration that is at
    ///         least the minimum distance away from the starting point.
    bool Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
        double _delta, LPOutput<MPTraits>& _lp,
        double _posRes, double _oriRes);
    bool Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
        double _delta, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
        double _posRes, double _oriRes);

    bool ExpandSubassembly(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
            double _delta, LPOutput<MPTraits>& _lp,
            double _posRes, double _oriRes);
    bool ExpandSubassembly(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
        double _delta, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
        double _posRes, double _oriRes);

    int GetCompositeIncrAndRot(const CfgType& _cfg, const CfgType& _target, CfgType& _incr,
                      CfgType& _incrUntouched,
                      mathtool::Orientation& _rotation, const double _posRes,
                      const double _oriRes);

    ///@}

    ///@name Internal State
    ///@{

    string m_dmLabel;         ///< The distance metric to use.
    string m_vcLabel;         ///< The validity checker to use.

    /// Important: This must be sorted so that the body to rotate ABOUT is the
    /// first element. There should also only be the bodies to move in here.
    std::vector<unsigned int> m_activeBodies;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ActiveBodyExtender<MPTraits>::
ActiveBodyExtender(const string& _dmLabel, const string& _vcLabel, double _min,
    double _max, bool _randomOrientation) :
    ExtenderMethod<MPTraits>(_min, _max), m_dmLabel(_dmLabel),
    m_vcLabel(_vcLabel) {
  this->SetName("ActiveBodyExtender");
}


template <typename MPTraits>
ActiveBodyExtender<MPTraits>::
ActiveBodyExtender(XMLNode& _node) : ExtenderMethod<MPTraits>(_node) {
  this->SetName("ActiveBodyExtender");

  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric label");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
ActiveBodyExtender<MPTraits>::
Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\tdistance metric : \"" << m_dmLabel << "\"" << endl
      << "\tvalidity checker : \"" << m_vcLabel << "\"" << endl;
}


template <typename MPTraits>
void
ActiveBodyExtender<MPTraits>::
Initialize() const {
  throw RunTimeException(WHERE, "This method is broken and should only be used"
                                " in the interest of fixing/testing it.");
}


/*------------------------- ExtenderMethod Overrides -------------------------*/

template <typename MPTraits>
bool
ActiveBodyExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
       LPOutput<MPTraits>& _lp) {
  if(m_activeBodies.empty())
    throw RunTimeException(WHERE, "Need active bodies if using disassembly extender!");
  Environment* env = this->GetEnvironment();

  _lp.SetLPLabel(this->GetLabel());//Not ideal, as this is an extender, but oh well.
  _lp.SetActiveBodies(m_activeBodies);

  bool expanded;

  ///@TODO: Comment out this condition if you want single parts to go through
  ///       the more expensive (but what SHOULD be equivalent subassembly
  ///       treatment):
  if(m_activeBodies.size() == 1)
    expanded = Expand(_start, _end, _new, this->m_maxDist, _lp,
                  env->GetPositionRes(), env->GetOrientationRes());
  else
    expanded = ExpandSubassembly(_start, _end, _new, this->m_maxDist, _lp,
                             env->GetPositionRes(), env->GetOrientationRes());
  m_activeBodies.clear(); // Require that bodies are set EVERY time
  return expanded;
}

template <typename MPTraits>
bool
ActiveBodyExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end,
       CfgType& _new, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo) {
  if(m_activeBodies.empty())
    throw RunTimeException(WHERE, "Need active bodies if using disassembly extender!");

  Environment* env = this->GetEnvironment();
  _lp.SetLPLabel(this->GetLabel());//Not ideal, as this is an extender, but oh well.
  _lp.SetActiveBodies(m_activeBodies);

  //Assume that all CD info is wanted, and clear out all data:
  _cdInfo.ResetVars(true);

  bool expanded;

  ///@TODO: Comment out this condition if you want single parts to go through
  ///       the more expensive (but what SHOULD be equivalent subassembly
  ///       treatment):
  if(m_activeBodies.size() == 1)
    expanded = Expand(_start, _end, _new, this->m_maxDist, _lp, _cdInfo,
                  env->GetPositionRes(), env->GetOrientationRes());
  else
    expanded = ExpandSubassembly(_start, _end, _new, this->m_maxDist, _lp, _cdInfo,
                             env->GetPositionRes(), env->GetOrientationRes());
  m_activeBodies.clear(); // Require that bodies are set EVERY time
  return expanded;
}

/*-------------------------------- Helpers? ----------------------------------*/

template <typename MPTraits>
bool
ActiveBodyExtender<MPTraits>::
Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
    double _delta, LPOutput<MPTraits>& _lp, double _posRes, double _oriRes) {
  CDInfo cdInfo;
  return Expand(_start, _end, _newCfg, _delta, _lp, cdInfo, _posRes, _oriRes);
}


template <typename MPTraits>
bool
ActiveBodyExtender<MPTraits>::
Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
    double _delta, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
    double _posRes, double _oriRes) {
  Environment* env = this->GetEnvironment();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);
  string callee("ActiveBodyExtender::Expand");

  CfgType incr(this->GetTask()->GetRobot()), tick = _start, previous = _start;
  bool collision = false;
  int nTicks, ticker = 0;

  incr.FindIncrement(tick, _end, &nTicks, _posRes, _oriRes);

  // Move out from start towards dir, bounded by number of ticks allowed at a
  // given resolution and the distance _delta: the maximum distance to grow
  while(!collision && dm->Distance(_start, tick) <= _delta &&
        ticker <= nTicks) {
    previous = tick;
    tick += incr;
    if(!tick.InBounds(env->GetBoundary()) || !(vc->IsValid(tick, _cdInfo, callee)))
      collision = true; //return previous tick, as it is collision-free
    ++ticker;
  }

  // Quit if we didn't expand at all.
  if(previous == _start) {
    if(this->m_debug)
      cout << "Could not expand !" << endl;
    return false;
  }

  // If we did expand, set _newCfg to the end of the extension.
  if(ticker == nTicks + 1)
    // Full expansion. We have to adjust _newCfg to be equal to _end because
    // of accumulated floating-point error from the division in FindIncrement and
    // adding incr to tick.
    // We do not specifically collision check _end because previous is within a
    // resolution of _end (they are very, very close) and it was already checked.
    _newCfg = _end;
  else
    // Collision reached. Use previous as it is the last collision-free tick
    _newCfg = previous;

  // Set edge weight according to distance metric.
  double distance = dm->Distance(_start, _newCfg);
  _lp.m_edge.first.SetWeight(distance);
  _lp.m_edge.second.SetWeight(distance);

  return distance >= this->m_minDist;
}

template <typename MPTraits>
bool
ActiveBodyExtender<MPTraits>::
ExpandSubassembly(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
                  double _delta, LPOutput<MPTraits>& _lp,
                  double _posRes, double _oriRes) {
  CDInfo cdInfo;
  return ExpandSubassembly(_start, _end, _newCfg, _delta, _lp, cdInfo,
                           _posRes, _oriRes);
}


template <typename MPTraits>
bool
ActiveBodyExtender<MPTraits>::
ExpandSubassembly(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
                  double _delta, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
                  double _posRes, double _oriRes) {
  Environment* env = this->GetEnvironment();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);
  string callee("ActiveBodyExtender::Expand");

  CfgType incr(this->GetTask()->GetRobot());
  CfgType incrUntouched(this->GetTask()->GetRobot());
  CfgType tick = _start;
  CfgType previous = _start;

  bool collision = false;
  int ticker = 0;
  const unsigned int refBodyNum = m_activeBodies[0]; // The body rotated about.

  ///@TODO move this sanity checking to DisassemblyMethod::Initialize probably.
  MultiBody* const mb = this->GetTask()->GetRobot()->GetMultiBody();
  const unsigned int numBodies = mb->GetNumBodies();
  const unsigned int posDofsPerBody = mb->PosDOF();
  const unsigned int oriDofsPerBody = mb->OrientationDOF();
  const unsigned int dofsPerBody = posDofsPerBody + oriDofsPerBody;

  // TODO clean up logic of this code.
  CfgType oneStep = _start; // The placeholder for computing steps of angles.


  // A little bit of sanity checking:
  if((dofsPerBody * numBodies) != mb->DOF() ||
     (posDofsPerBody + oriDofsPerBody) != dofsPerBody)
    throw RunTimeException(WHERE, "DOFs don't match up for multibody!");

  // TODO clean up the logic of the code here:
  mathtool::Orientation rotation;
  int nTicks = GetCompositeIncrAndRot(_start, _end, incr, incrUntouched, rotation,
                             _posRes, _oriRes);

  if(this->m_debug) {
    mathtool::EulerAngle e = convertFromMatrix(e,rotation.matrix());
    std::cout << this->GetNameAndLabel() << std::endl
              << "Active bodies = " << m_activeBodies << std::endl
              << "_start = " << _start.PrettyPrint(4) << std::endl
              << "_end = " << _end.PrettyPrint(4) << std::endl
              << "nTicks = " << nTicks << " | posRes = " << _posRes
              << " | oriRes = " << _oriRes << std::endl
              << "incrUntouched = " << incrUntouched.PrettyPrint(4) << std::endl
              << "incr = " << incr.PrettyPrint(4) << std::endl
              << "Euler angle of rotation = " << e << std::endl << std::endl;
  }

  // Move out from start towards dir, bounded by number of ticks allowed at a
  // given resolution and the distance _delta: the maximum distance to grow
  while(!collision && dm->Distance(_start, tick) <= _delta &&
        ticker <= nTicks) {
    previous = tick;
    tick += incr;

    oneStep += incrUntouched;

    // New attempt:
    previous.ConfigureRobot();
    mathtool::Transformation initialTransform = previous.GetMultiBody()->
                                  GetBody(refBodyNum)->GetWorldTransformation();

    oneStep.ConfigureRobot();
    mathtool::Transformation finalTransform = oneStep.GetMultiBody()->
                                  GetBody(refBodyNum)->GetWorldTransformation();

    mathtool::Transformation delta = -initialTransform * finalTransform;

    //Note that m_activeBodies[0] is the body to rotate about.
    rotation = delta.rotation();

    if(this->m_debug)
      std::cout << "tick before rotation = " << tick.PrettyPrint(4) << std::endl;


    // rotation is the rotation delta for m_activeBodies[0] by incr, then
    // gets applied to each body ABOUT m_activeBodies[0].
    tick = RotateCfgAboutBody<MPTraits>(m_activeBodies, tick, rotation,
                                        dofsPerBody, this->m_debug);

    if(this->m_debug) {
      std::cout << "tick after rotation = " << tick.PrettyPrint(6) << std::endl
                << "Interpolated dofs for first active body (body " << refBodyNum
                << ") =" << std::endl << "{ ";
      const CfgType interpCfg = _start + incrUntouched * (ticker + 1);
      for(unsigned int i = 0; i < dofsPerBody-1; ++i)
        std::cout << interpCfg[refBodyNum*dofsPerBody + i] << ", ";
      std::cout << interpCfg[refBodyNum*dofsPerBody + dofsPerBody-1] << " }"
                << std::endl << "Same dofs from tick (should match closely) = "
                << std::endl << "{ ";
      for(unsigned int i = 0; i < dofsPerBody-1; ++i)
        std::cout << tick[refBodyNum*dofsPerBody + i] << ", ";
      std::cout << tick[refBodyNum*dofsPerBody + dofsPerBody-1] << " }"
                << std::endl << std::endl;
    }

    if(!tick.InBounds(env->GetBoundary()) || !(vc->IsValid(tick, _cdInfo, callee)))
      collision = true; //return previous tick, as it is collision-free
    ++ticker;
  }

  // Quit if we didn't expand at all.
  if(previous == _start) {
    if(this->m_debug)
      std::cout << "Could not expand!" << std::endl;
    return false;
  }

  // If we did expand, set _newCfg to the end of the extension.
  if(ticker == nTicks + 1) {
    if(this->m_debug) {
      const CfgType finalDiff = _end - tick;
      std::cout << std::endl << "Successful expansion, setting _newCfg = _end."
                << std::endl << "Final difference (_end - tick) = "
                << finalDiff.PrettyPrint(4) << std::endl
                << "Checking error between _end and tick..." << std::endl;

      // Confirm tick is within resolution of _end:
      if(!tick.WithinResolution(_end, _posRes, _oriRes)) {
        const string output = "Error: debug test caught that the final cfg"
            " of a full extension was more than a resolution off (greater than "
            + std::to_string(_posRes) + " or " + std::to_string(_oriRes) +
            ") from the desired end cfg. Check preceeding output for values.";
        throw RunTimeException(WHERE, output);
        std::cout << output << std::endl << std::endl;
      }
    }

    // adjust _newCfg to be equal to _end because of minor float error that is
    // likely present in tick.
    _newCfg = _end;
  }
  else
    _newCfg = previous; // Collision, use previous as last collision-free tick

  // Set edge weight according to distance metric.
  double distance = dm->Distance(_start, _newCfg);
  _lp.m_edge.first.SetWeight(distance);
  _lp.m_edge.second.SetWeight(distance);

  return distance >= this->m_minDist;
}



template <typename MPTraits>
int
ActiveBodyExtender<MPTraits>::
GetCompositeIncrAndRot(const CfgType& _cfg, const CfgType& _target, CfgType& _incr,
              CfgType& _incrUntouched, mathtool::Orientation& _rotation,
              const double _posRes, const double _oriRes) {
  MultiBody* const mb = this->GetTask()->GetRobot()->GetMultiBody();
  const unsigned int posDofsPerBody = mb->PosDOF();
  const unsigned int dofsPerBody = posDofsPerBody + mb->OrientationDOF();

  //This function returns (via references) the incr with the delta translation
  // for the rotated-about body placed into each translation component for _incr
  //_rotation will hold the rotation delta that FindIncrement finds for the
  // rotated-about body.
  int nTicks;
  const unsigned int refBodyNum = m_activeBodies[0]; // The body rotated about.
  _incrUntouched.FindIncrement(_cfg, _target, &nTicks, _posRes, _oriRes);
  _incrUntouched.GetMultiBody()->Configure(_incrUntouched);// For transformation
  const mathtool::Transformation incrTransform = _incrUntouched.GetMultiBody()->
                                  GetBody(refBodyNum)->GetWorldTransformation();
  if(this->m_debug) {
    CfgType diffCfg = _target - _cfg;
    diffCfg.GetMultiBody()->Configure(diffCfg);
    const mathtool::Transformation diffTransform = diffCfg.GetMultiBody()->
                                  GetBody(refBodyNum)->GetWorldTransformation();
    std::cout << "Delta's (incr's) transform = " << incrTransform << std::endl
              << "(_target - _cfg) transform = " << diffTransform << std::endl
              << "nTicks = " << nTicks << std::endl;
  }

  //Note that m_activeBodies[0] is the body to rotate about.
  //This rotation currently gets overwritten, so it technically doesn't matter.
  _rotation = incrTransform.rotation();


  ///@TODO This should be able to use GroupUtils::OverwriteDofsFromBodies().
  //Now remove the rotational bits, as incr should only do the
  // translation and then RotateCfgAboutBody() will handle all rotations:
  _incr = CfgType(this->GetTask()->GetRobot()); // Ensure zeroed out.
  for(const unsigned int body : m_activeBodies) {
    //Copy translation of main body to translation dofs for other active bodies.
    for(unsigned int i = 0; i < posDofsPerBody; ++i)
      _incr[body*dofsPerBody + i] = _incrUntouched[refBodyNum*dofsPerBody + i];
  }
  return nTicks;
}

/*----------------------------------------------------------------------------*/

#endif
