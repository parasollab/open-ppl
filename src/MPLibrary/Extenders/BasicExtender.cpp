#include "BasicExtender.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------- Construction -------------------------------*/

BasicExtender::
BasicExtender() {
  this->SetName("BasicExtender");
}


BasicExtender::
BasicExtender(XMLNode& _node) : ExtenderMethod(_node) {
  this->SetName("BasicExtender");

  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric label");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
  m_randomOrientation = _node.Read("randomOrientation", false,
      m_randomOrientation, "Use random orientation?");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
BasicExtender::
Print(std::ostream& _os) const {
  ExtenderMethod::Print(_os);
  _os << "\tdistance metric: " << m_dmLabel
      << "\n\tvalidity checker: " << m_vcLabel
      << "\n\trandom orientation: " << m_randomOrientation
      << std::endl;
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

bool
BasicExtender::
Extend(const Cfg& _start, const Cfg& _end, Cfg& _new,
    LPOutput& _lp) {
  Environment* env = this->GetEnvironment();

  // If non-random orientation, adjust the end's non-positional DOFs to match
  // the start.
  if(!m_randomOrientation) {
    Cfg end = _end;
    for(size_t i = end.PosDOF(); i < _end.DOF(); i++)
      end[i] = _start[i];
    return Expand(_start, end, _new, this->m_maxDist, _lp,
        env->GetPositionRes(), env->GetOrientationRes());
  }
  return Expand(_start, _end, _new, this->m_maxDist, _lp,
      env->GetPositionRes(), env->GetOrientationRes());
}


bool
BasicExtender::
Extend(const Cfg& _start, const Cfg& _end, Cfg& _new,
    LPOutput& _lp, CDInfo& _cdInfo) {
  Environment* env = this->GetEnvironment();

  //Clear out all data, retaining whether all CD data was wanted:
  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);

  // If non-random orientation, adjust the end's non-positional DOFs to match
  // the start.
  if(!m_randomOrientation) {
    Cfg end = _end;
    for(size_t i = end.PosDOF(); i < _end.DOF(); i++)
      end[i] = _start[i];
    return Expand(_start, end, _new, this->m_maxDist, _lp, _cdInfo,
        env->GetPositionRes(), env->GetOrientationRes());
  }

  return Expand(_start, _end, _new, this->m_maxDist, _lp, _cdInfo,
      env->GetPositionRes(), env->GetOrientationRes());
}


bool
BasicExtender::
Extend(const GroupCfgType& _start, const GroupCfgType& _end, GroupCfgType& _new,
       GroupLPOutput& _lp, const Formation& _robotIndexes) {
  Environment* env = this->GetEnvironment();

  _lp.SetLPLabel(this->GetLabel());
  _lp.SetFormation(_robotIndexes);

  return Expand(_start, _end, _new, this->m_maxDist, _lp,
      env->GetPositionRes(), env->GetOrientationRes(), _robotIndexes);
}


bool
BasicExtender::
Extend(const GroupCfgType& _start, const GroupCfgType& _end, GroupCfgType& _new,
    GroupLPOutput& _lp, CDInfo& _cdInfo,
    const Formation& _robotIndexes) {
  Environment* env = this->GetEnvironment();

  _lp.SetLPLabel(this->GetLabel());
  _lp.SetFormation(_robotIndexes);

  _cdInfo.ResetVars(_cdInfo.m_retAllInfo);

  return Expand(_start, _end, _new, this->m_maxDist, _lp, _cdInfo,
      env->GetPositionRes(), env->GetOrientationRes(), _robotIndexes);
}

/*-------------------------------- Helpers? ----------------------------------*/

bool
BasicExtender::
Expand(const Cfg& _start, const Cfg& _end, Cfg& _newCfg,
    double _delta, LPOutput& _lp, double _posRes, double _oriRes) {
  CDInfo cdInfo;
  return Expand(_start, _end, _newCfg, _delta, _lp, cdInfo, _posRes, _oriRes);
}


bool
BasicExtender::
Expand(const Cfg& _start, const Cfg& _end, Cfg& _newCfg,
    double _delta, LPOutput& _lp, CDInfo& _cdInfo,
    double _posRes, double _oriRes) {
  _lp.Clear();
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

  Cfg incr(this->GetTask()->GetRobot()),
          tick = _start,
          previous = _start;
  bool collision = false;
  int nTicks, ticker = 0;

  incr.FindIncrement(tick, _end, &nTicks, _posRes, _oriRes);

  if(this->m_debug)
    std::cout << "Trying extension:"
              << "\n\tFrom: " << _start.PrettyPrint()
              << "\n\tTo:   " << _end.PrettyPrint()
              << "\n\tIncr: " << incr.PrettyPrint()
              << "\n\tNum ticks: " << nTicks
              << std::endl;

  // Move out from start towards dir, bounded by number of ticks allowed at a
  // given resolution and the distance _delta: the maximum distance to grow
  while(!collision && dm->Distance(_start, tick) <= _delta &&
        ticker <= nTicks) {
    previous = tick;
    tick += incr;
    if(!vc->IsValid(tick, _cdInfo, "BasicExtender::Expand"))
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
  if(ticker == nTicks + 1)
    // Full expansion. We have to adjust _newCfg to be equal to _end because
    // of accumulated floating-point error from the division in FindIncrement and
    // adding incr to tick.
    /// @todo No, we don't *have* to set _newCfg to _end, and it would be better
    ///       if we didn't use this assumption. We are doing this so that we
    ///       can use extenders as local planners in multi-tree RRT methods. To
    ///       fix and remove the assumption, we need to homogenize our extenders
    ///       and local planners into a single class with methods for both uses
    ///       (i.e. 'Extend' and 'LocalPlan').
    // We do not specifically collision check _end because previous is within a
    // resolution of _end (they are very, very close) and it was already checked.
    _newCfg = _end;
  else
    // Collision reached. Use previous as it is the last collision-free tick
    _newCfg = previous;

  // Set edge weight according to distance metric.
  const double distance = dm->Distance(_start, _newCfg);
  _lp.m_edge.first.SetWeight(distance);
  _lp.m_edge.second.SetWeight(distance);

  if(this->m_debug)
    std::cout << "Extended: " << std::setprecision(4) << distance << " units."
              << "\n\tMin:Max distance: "
              << this->GetMinDistance() << " : " << this->GetMaxDistance()
              << "\n\tend: " << _newCfg.PrettyPrint()
              << std::endl;

  return distance >= this->m_minDist;
}


bool
BasicExtender::
Expand(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _newCfg, double _delta, GroupLPOutput& _lp,
    double _posRes, double _oriRes, const Formation& _robotIndexes) {
  CDInfo cdInfo;
  return Expand(_start, _end, _newCfg, _delta, _lp, cdInfo, _posRes, _oriRes,
      _robotIndexes);
}


bool
BasicExtender::
Expand(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _newCfg, double _delta, GroupLPOutput& _lp,
    CDInfo& _cdInfo, double _posRes, double _oriRes,
    const Formation& _robotIndexes) {
  if(_robotIndexes.empty())
    throw RunTimeException(WHERE) << "TODO: Need to fix group Cfg extenders to "
                                  << "work for the general case and not just "
                                  << "disassembly. Code needs to be adjusted so "
                                  << "that an empty formation means all robots "
                                  << "move without using a formation.";


  Environment* const env = this->GetEnvironment();
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

  GroupCfgType tick = _start;
  GroupCfgType previous = _start;


  /// Set these to true to have single parts treated (less efficiently) as
  /// multiple parts, which should now be identical.
  const bool multipleParts = _robotIndexes.size() > 1;
  const bool isRotational = _start.GetRobot(0)->GetMultiBody()->OrientationDOF() > 0;
  const bool subassemblyRotation = multipleParts && isRotational;

  GroupCfgType oneStep = _start; // The placeholder for computing steps of angles.

  int nTicks;
  const unsigned int leaderRobotIndex = _robotIndexes[0]; // The body rotated about.

  GroupCfgType incr(_start.GetGroupRoadmap());

  // Will find all the straight-line increments for each robot independently.
  // (Though the nTicks calculation is coupled with all moving robots).
  incr.FindIncrement(_start, _end, &nTicks, _posRes, _oriRes);
  const GroupCfgType incrUntouched = incr;

  if(subassemblyRotation) {
    // Remove the rotational bits, as incr should only do the translation
    // and then RotateFormationAboutLeader() will handle all rotations.
    incr = GroupCfgType(_start.GetGroupRoadmap()); // Ensure zeroed out.
    incr.OverwriteDofsForRobots(
            incrUntouched.GetRobotCfg(leaderRobotIndex).GetLinearPosition(),
            _robotIndexes);
  }

  // Move out from start towards dir, bounded by number of ticks allowed at a
  // given resolution and the distance _delta: the maximum distance to grow
  mathtool::Orientation rotation;
  bool collision = false;
  int ticker = 0;
  while(!collision && dm->Distance(_start, tick) <= _delta &&
        ticker <= nTicks) {
    previous = tick;
    tick += incr;

    if(this->m_debug)
      std::cout << "Extending group on tick " << ticker << std::endl;

    if(subassemblyRotation) {
      // Handle subassembly rotation. We must update the delta transformation
      // due to Euler Angles not conforming to linear angle changes between cfgs

      /// TODO: this can likely be optimized. For one, only one Configure call
      /// should be necessary here. Also a lot of the group Cfgs here could be
      /// made individual if using the leader, then using Configure on that.
      oneStep += incrUntouched;

      // Note we get the 0 body from the robot, as right now it's assumed that
      // all robots in a group have multibodies with a single body.
      previous.ConfigureRobot();
      mathtool::Transformation initialTransform =
                            previous.GetRobot(leaderRobotIndex)->GetMultiBody()->
                            GetBody(0)->GetWorldTransformation();

      oneStep.ConfigureRobot();
      mathtool::Transformation finalTransform =
                            oneStep.GetRobot(leaderRobotIndex)->GetMultiBody()->
                            GetBody(0)->GetWorldTransformation();

      mathtool::Transformation delta = -initialTransform * finalTransform;
      rotation = delta.rotation();

      if(this->m_debug)
        std::cout << "tick before rotation = " << tick.PrettyPrint()
                  << std::endl;

      tick.RotateFormationAboutLeader(_robotIndexes, rotation, this->m_debug);

      if(this->m_debug)
        std::cout << "tick after rotation = " << tick.PrettyPrint()
                  << std::endl << std::endl;
    }

    if(tick.InBounds(env->GetBoundary())) {
      //if(!vc->IsValid(tick, _cdInfo, "GroupExtender::Expand", _robotIndexes)) {
      if(!vc->IsValid(tick, _cdInfo, "GroupExtender::Expand")) {
        collision = true; //return previous tick, as it is collision-free
        if(this->m_debug)
          std::cout << "Collision found for extension tick!" << std::endl;
      }
    }
    else {
      collision = true;
      if(this->m_debug)
        std::cout << "Out of bounds extension tick!" << std::endl;
    }

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
      const GroupCfgType finalDiff = _end - tick;
      std::cout << "Successful expansion, setting _newCfg = _end."
                << "\nFinal difference (_end - tick) = "
                << finalDiff.PrettyPrint(4)
                << std::endl;

      // Confirm tick is within resolution of _end:
      if(!tick.WithinResolution(_end, _posRes, _oriRes))
        throw RunTimeException(WHERE) << "The final cfg of a full extension "
                                      << "was more than a resolution off "
                                      << "(greater than "
                                      << _posRes << " or " << _oriRes
                                      << ") from the desired end cfg.";
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

  // Add individual edges to the GroupLocalPlan:
  _lp.SetIndividualEdges(_robotIndexes);

  return distance >= this->m_minDist;
}

/*----------------------------------------------------------------------------*/
