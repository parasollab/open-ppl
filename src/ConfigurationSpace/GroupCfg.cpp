#include "GroupCfg.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Formation.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "nonstd.h"


/*------------------------------- Construction -------------------------------*/

template <typename GraphType>
GroupCfg<GraphType>::
GroupCfg(GroupRoadmapType* const& _groupMap, const bool _init) 
     : CompositeState<GraphType>((GroupGraphType*)_groupMap, GroupCfg<GraphType>::IndividualRoadmap::GetVertex) {

  // If no group map was given, this is a placeholder object. We can't do
  // anything with it since every meaningful operation requires a group map.
  if(!this->m_groupGraph)
    return;

  InitializeFormations();

  // Initialize local configurations if requested.
  if(_init)
    InitializeLocalCfgs();
}


// template <typename GraphType>
// GroupCfg<GraphType>::
// GroupCfg(GroupGraphType* const& _groupMap, const bool _init) 
//      : CompositeState<GraphType>(_groupMap, GroupCfg<GraphType>::IndividualRoadmap::GetVertex) {

//   // If no group map was given, this is a placeholder object. We can't do
//   // anything with it since every meaningful operation requires a group map.
//   if(!this->m_groupGraph)
//     return;

//   InitializeFormations();

//   // Initialize local configurations if requested.
//   if(_init)
//     InitializeLocalCfgs();
// }


/*--------------------------------- Equality ---------------------------------*/

template <typename GraphType>
bool
GroupCfg<GraphType>::
operator==(const GroupCfg& _other) const noexcept {
  // If _other is from another map, these are not the same.
  if(this->m_groupGraph != _other.m_groupGraph)
    return false;

  // Else, compare VIDs if both are valid, or by-value other wise.
  for(size_t i = 0; i < this->m_vids.size(); ++i) {
    const VID thisVID  = this->m_vids[i],
              otherVID = _other.m_vids[i];

    if(thisVID != INVALID_VID and otherVID != INVALID_VID) {
      if(thisVID != otherVID)
        return false;
    }
    else if(GetRobotCfg(i) != _other.GetRobotCfg(i))
      return false;
  }

  // Else compare formations
  if(m_formations.size() != _other.m_formations.size())
    return false; 

  for(auto formation : _other.m_formations) {
    if(!m_formations.count(formation))
      return false;
  }

  return true;
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
operator!=(const GroupCfg& _other) const noexcept {
  return !(*this == _other);
}

template <typename GraphType>
bool
GroupCfg<GraphType>::
operator<(const GroupCfg& _other) const noexcept {

  const auto& robots = this->GetRobots();

  for(size_t i = 0; i < robots.size(); i++) {
    const auto& cfg1 = GetRobotCfg(i);
    const auto& cfg2 = _other.GetRobotCfg(i);
    if(cfg1 < cfg2)
      return true;
    else if(cfg2 < cfg1)
      return false;
  }

  return false;
}
/*-------------------------------- Arithmetic --------------------------------*/

template <typename GraphType>
GroupCfg<GraphType>
GroupCfg<GraphType>::
operator+(const GroupCfg& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg += _other);
}


template <typename GraphType>
GroupCfg<GraphType>
GroupCfg<GraphType>::
operator-(const GroupCfg& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg -= _other);
}


template <typename GraphType>
GroupCfg<GraphType>
GroupCfg<GraphType>::
operator*(const double& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg *= _other);
}


template <typename GraphType>
GroupCfg<GraphType>&
GroupCfg<GraphType>::
operator+=(const GroupCfg& _other) {
  // We must require the exact same group, which indicates everything
  // lines up between the two cfgs (namely the exact robots/order of the group).
  if(this->m_groupGraph->GetGroup() != _other.m_groupGraph->GetGroup())
    throw RunTimeException(WHERE, "Cannot add GroupCfgs with different group "
                                  "roadmaps!");

  // Also ensure that the same formations exists.
  if(m_formations.size() != _other.m_formations.size())
    throw RunTimeException(WHERE) << "Cannot add GroupCfgs with different formations.";

  for(auto f1 : m_formations) {
    bool match = false;
    for(auto f2 : _other.m_formations) {
      if(*f1 == *f2) {
        match = true;
        break;
      }
    }
    if(!match)
      throw RunTimeException(WHERE) << "Cannot add GroupCfgs with different formations.";
  }

  // First add the formation dofs.
  std::set<size_t> checked;

  for(auto formation : m_formations) {
    std::vector<Cfg> cfgs;
    std::vector<Cfg> otherCfgs;

    for(auto robot : formation->GetRobots()) {
      cfgs.push_back(this->GetRobotCfg(robot));
      otherCfgs.push_back(_other.GetRobotCfg(robot));
    }

    auto dofs = formation->ConvertToFormationDOF(cfgs);
    auto otherDofs = formation->ConvertToFormationDOF(otherCfgs);


    for(size_t i = 0; i < dofs.size(); i++) {
      dofs[i] = dofs[i] + otherDofs[i];
    }

    cfgs = formation->ConvertToIndividualCfgs(dofs);

    for(auto cfg : cfgs) {
      const size_t index = this->m_groupGraph->GetGroup()->GetGroupIndex(cfg.GetRobot());
      auto copy = cfg;
      SetRobotCfg(index,std::move(copy));
      checked.insert(index);
    }
  }

  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    if(checked.count(i))
      continue;
    SetRobotCfg(i, GetRobotCfg(i) + _other.GetRobotCfg(i));
  }

  return *this;
}


template <typename GraphType>
GroupCfg<GraphType>&
GroupCfg<GraphType>::
operator-=(const GroupCfg& _other) {
  // We must require the exact same group roadmap, which indicates everything
  // lines up between the two cfgs (namely the exact robots/order of the group).
  if(this->m_groupGraph != _other.m_groupGraph)
    throw RunTimeException(WHERE, "Cannot add GroupCfgs with different group "
                                  "roadmaps!");

  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    SetRobotCfg(i, GetRobotCfg(i) - _other.GetRobotCfg(i));

  return *this;
}


template <typename GraphType>
GroupCfg<GraphType>&
GroupCfg<GraphType>::
operator*=(const double& _val) {
  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    SetRobotCfg(i, GetRobotCfg(i) * _val);

  return *this;
}

/*---------------------------- Roadmap Accessors -----------------------------*/

template <typename GraphType>
typename GroupCfg<GraphType>::GroupRoadmapType*
GroupCfg<GraphType>::
GetGroupRoadmap() const noexcept {
  return (GroupRoadmapType*)this->m_groupGraph;
}


template <typename GraphType>
GroupCfg<GraphType>
GroupCfg<GraphType>::
SetGroupRoadmap(GroupRoadmapType* const _newRoadmap) const {
  // Check that groups are compatible.
  if(this->m_groupGraph->GetGroup() != _newRoadmap->GetGroup())
    throw RunTimeException(WHERE) << "Trying to change roadmaps on incompatible "
                                  << "groups!";

  // Create new cfg using _roadmap and initializing all entries locally to 0.
  GroupCfg<GraphType> newCfg(_newRoadmap);

  // Put all individual cfgs into group cfg so that all are local:
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    newCfg.SetRobotCfg(i, IndividualCfg(GetRobotCfg(i)));

  return newCfg;
}
    
template <typename GraphType>
void
GroupCfg<GraphType>::
AddFormation(Formation* _formation) {
  m_formations.insert(_formation);
}
    
template <typename GraphType>
const std::unordered_set<Formation*>&
GroupCfg<GraphType>::
GetFormations() const {
  return m_formations;
}

/*------------------------ Individual Configurations -------------------------*/

template <typename GraphType>
void
GroupCfg<GraphType>::
SetRobotCfg(Robot* const _robot, IndividualCfg&& _cfg) {
  const size_t index = this->m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  SetRobotCfg(index, std::move(_cfg));
}


template <typename GraphType>
void
GroupCfg<GraphType>::
SetRobotCfg(const size_t _index, IndividualCfg&& _cfg) {
  this->VerifyIndex(_index);

  // Allocate space for local cfgs if not already done.
  InitializeLocalCfgs();

  //TODO::Make sure this obeys formations constraints.

  m_localCfgs[_index] = std::move(_cfg);
  this->m_vids[_index] = INVALID_VID;
}


template <typename GraphType>
void
GroupCfg<GraphType>::
SetRobotCfg(Robot* const _robot, const VID _vid) {
  const size_t index = this->m_groupMap->GetGroup()->GetGroupIndex(_robot);
  SetRobotCfg(index, _vid);
}


template <typename GraphType>
void
GroupCfg<GraphType>::
SetRobotCfg(const size_t _index, const VID _vid) {
  this->VerifyIndex(_index);

  this->m_vids[_index] = _vid;
}


template <typename GraphType>
void
GroupCfg<GraphType>::
ClearLocalCfgs() {
  m_localCfgs.clear();
}


template <typename GraphType>
typename GroupCfg<GraphType>::IndividualCfg&
GroupCfg<GraphType>::
GetRobotCfg(Robot* const _robot) {
  const size_t index = this->m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


template <typename GraphType>
typename GroupCfg<GraphType>::IndividualCfg&
GroupCfg<GraphType>::
GetRobotCfg(const size_t _index) {
  this->VerifyIndex(_index);

  const VID vid = this->GetVID(_index);
  if(vid != INVALID_VID)
    return this->m_groupGraph->GetRoadmap(_index)->GetVertex(vid);
  else {
    InitializeLocalCfgs();
    return m_localCfgs[_index];
  }
}


template <typename GraphType>
const typename GroupCfg<GraphType>::IndividualCfg&
GroupCfg<GraphType>::
GetRobotCfg(Robot* const _robot) const {
  const size_t index = this->m_groupGraph->GetGroup()->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


template <typename GraphType>
const typename GroupCfg<GraphType>::IndividualCfg&
GroupCfg<GraphType>::
GetRobotCfg(const size_t _index) const {
  this->VerifyIndex(_index);

  // If we have a valid VID for this robot, fetch its configuration from its
  // individual roadmap.
  const VID vid = this->GetVID(_index);
  if(vid != INVALID_VID)
    return this->m_groupGraph->GetRoadmap(_index)->GetVertex(vid);

  try {
    return m_localCfgs.at(_index);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Requested configuration for robot "
                                  << _index
                                  << ", but no roadmap or local cfg exists.";
  }
}

/*------------------------------ DOF Accessors -------------------------------*/

template <typename GraphType>
size_t
GroupCfg<GraphType>::
PosDOF(const size_t _index) const {
  return this->GetRobot(_index)->GetMultiBody()->PosDOF();
}


template <typename GraphType>
size_t
GroupCfg<GraphType>::
OriDOF(const size_t _index) const {
  return this->GetRobot(_index)->GetMultiBody()->OrientationDOF();
}


template <typename GraphType>
size_t
GroupCfg<GraphType>::
DOF(const size_t _index) const {
  return this->GetRobot(_index)->GetMultiBody()->DOF();
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
IsNonholonomic() const noexcept {
  for(auto robot : this->GetRobots())
    if(robot->IsNonholonomic())
      return true;
  return false;
}


template <typename GraphType>
size_t
GroupCfg<GraphType>::
CompositeDOF() const {
  size_t dofSum = 0;
  for(auto robot : this->GetRobots())
    dofSum += robot->GetMultiBody()->DOF();
  return dofSum;
}


template <typename GraphType>
double
GroupCfg<GraphType>::
Magnitude() const {
  double result = 0;
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const double m = GetRobotCfg(i).Magnitude();
    result += m * m;
  }
  return std::sqrt(result);
}


template <typename GraphType>
double
GroupCfg<GraphType>::
PositionMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const double m = GetRobotCfg(i).PositionMagnitude();
    result += m * m;
  }
  return std::sqrt(result);
}


template <typename GraphType>
double
GroupCfg<GraphType>::
OrientationMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const double m = GetRobotCfg(i).OrientationMagnitude();
    result += m * m;
  }
  return std::sqrt(result);
}

/*------------------------- Configuration Helpers ----------------------------*/

template <typename GraphType>
void
GroupCfg<GraphType>::
ConfigureRobot() const {
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    GetRobotCfg(i).ConfigureRobot();
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
WithinResolution(const GroupCfg& _cfg, const double _posRes,
    const double _oriRes) const {
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    if(!GetRobotCfg(i).WithinResolution(_cfg.GetRobotCfg(i), _posRes, _oriRes))
      return false;

  return true;
}

/*------------------------------DOF Modifiers---------------------------------*/

/*void
GroupCfg::
RotateFormationAboutLeader(const Formation& _robotList,
    const mathtool::Orientation& _rotation, const bool _debug) {
  /// Note: Currently assumes all robots just have ONE body. The case of multi-
  /// bodied robots would need to be specially handled (right now it should just
  /// be split into multiple robots if a group is needed).

  /// @todo We can probably compute this without having to configure the models
  ///       (which cost a lot of transformations).
  ConfigureRobot(); // Configure all individual cfgs.

  const size_t leaderIndex = _robotList[0];

  // Get transformation of leader before rotation:
  const IndividualCfg& leaderCfg = GetRobotCfg(leaderIndex);

  // TODO update this to handle multiple bodies per robot.
  // Use the multibody's body 0, since we assume each MB just has a single body.
  mathtool::Transformation initialLeaderTransform = leaderCfg.GetMultiBody()->
                                           GetBody(0)->GetWorldTransformation();

  const mathtool::Transformation rotation(mathtool::Vector3d(0,0,0), _rotation);

  if(_debug)
    std::cout << "Rotating bodies " << _robotList << " with rotation = "
              << rotation << std::endl;

  // The transform to be applied to all parts (including the first one). We
  // move the part to its relative world position with A at the world origin,
  // then the rotation is applied, and we return the part to its relative
  // position from A.
  const mathtool::Transformation transform = initialLeaderTransform * rotation;

  ApplyTransformationForRobots(_robotList, transform, initialLeaderTransform);
}*/

/*
void
GroupCfg::
ApplyTransformationForRobots(const Formation& _robotList,
    const mathtool::Transformation& _transform,
    const mathtool::Transformation& _relativeTransform) {
  //Compute each robot's needed transformation and set dofs in cfg.
  for (const size_t robotIndex : _robotList) {
    const IndividualCfg& robotCfg = GetRobotCfg(robotIndex);

    /// @todo Generalize this to handle robots with more than one body.
    if(robotCfg.GetMultiBody()->GetNumBodies() > 1)
      throw RunTimeException(WHERE) << "Multiple bodies not supported!";

    // Retrieve current position and rotation of robot:
    const mathtool::Transformation& initialRobotTransform =
                  robotCfg.GetMultiBody()->GetBody(0)->GetWorldTransformation();

    // From right to left: apply the inverse relative transform to the initial
    // robot transform (puts the robot into the desired frame). Then apply
    // the transform given.
    const mathtool::Transformation newTransformation =   _transform *
                                                       (-_relativeTransform) *
                                                         initialRobotTransform;

    // Extract the transformation. Note: This is assuming 6 DOFs!
    const std::vector<double>& transformed = newTransformation.GetCfg();

//    OverwriteDofsForRobots(transformed, _robotList);
    OverwriteDofsForRobots(transformed, {robotIndex});
  }
}*/


template <typename GraphType>
void
GroupCfg<GraphType>::
AddDofsForRobots(const std::vector<double>& _dofs, const std::vector<size_t>& _robots) {
  for(const size_t robotIndex : _robots) {
    if(IsLocalCfg(robotIndex)) {
      // We can simply modify the local values, since it's not a roadmap cfg yet
      IndividualCfg& cfg = GetRobotCfg(robotIndex);

      // Ensure this robot has the correct number of DOF.
      if(_dofs.size() != cfg.DOF())
        throw RunTimeException(WHERE) << "Tried to add " << _dofs.size()
                                      << "dofs to robot " << robotIndex
                                      << ", which has " << cfg.DOF() << " DOFs.";

      // Update the robot's cfg.
      for(unsigned int i = 0; i < _dofs.size(); ++i)
        cfg[i] += _dofs[i];
    }
    else {
      // Must copy the cfg since it is not local.
      IndividualCfg cfg = GetRobotCfg(robotIndex);

      // Ensure this robot has the correct number of DOF.
      if(_dofs.size() != cfg.DOF())
        throw RunTimeException(WHERE) << "Tried to add " << _dofs.size()
                                      << "dofs to robot " << robotIndex
                                      << ", which has " << cfg.DOF() << " DOFs.";

      // Update the robot's cfg.
      for(unsigned int i = 0; i < _dofs.size(); ++i)
        cfg[i] += _dofs[i];
      SetRobotCfg(robotIndex, std::move(cfg));
    }
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
AddDofsForRobots(const mathtool::Vector3d& _dofs, const std::vector<size_t>& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg robotCfg = GetRobotCfg(robotIndex);
    for(size_t i = 0; i < robotCfg.PosDOF(); ++i)
      robotCfg[i] += _dofs[i];
    SetRobotCfg(robotIndex, std::move(robotCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
OverwriteDofsForRobots(const std::vector<double>& _dofs,
    const std::vector<size_t>& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg newIndividualCfg(this->GetRobot(robotIndex));
    newIndividualCfg.SetData(_dofs);
    SetRobotCfg(robotIndex, std::move(newIndividualCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
OverwriteDofsForRobots(const mathtool::Vector3d& _dofs,
    const std::vector<size_t>& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg newIndividualCfg(this->GetRobot(robotIndex));
    newIndividualCfg.SetLinearPosition(_dofs);
    SetRobotCfg(robotIndex, std::move(newIndividualCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
OverwriteDofsForRobots(const GroupCfg& _fromCfg, const std::vector<size_t>& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg robotCfg = _fromCfg.GetRobotCfg(robotIndex);
    SetRobotCfg(robotIndex, std::move(robotCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
OverwriteDofsForRobots(const GroupCfg& _fromCfg,
    const std::vector<Robot*>& _robots) {
  auto fromGroup = _fromCfg.GetGroupRoadmap()->GetGroup(),
       toGroup   = this->m_groupGraph->GetGroup();

  for(Robot* const robot : _robots) {
    const size_t fromIndex = fromGroup->GetGroupIndex(robot),
                 toIndex   = toGroup->GetGroupIndex(robot);
    IndividualCfg robotCfg = _fromCfg.GetRobotCfg(fromIndex);
    SetRobotCfg(toIndex, std::move(robotCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
SetData(const std::vector<double>& _dofs) {
  if(_dofs.size() != CompositeDOF())
    throw RunTimeException(WHERE) << "Tried to set " << _dofs.size()
                                  << " DOFs on a robot group with "
                                  << CompositeDOF() << " DOFs.";

  size_t compositeIndex = 0;
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const size_t robotDof = DOF(i);
    IndividualCfg& robotCfg = GetRobotCfg(i);

    for(size_t i = 0; i < robotDof; ++i, ++compositeIndex)
      robotCfg[i] = _dofs[compositeIndex];
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
FindIncrement(const GroupCfg& _start, const GroupCfg& _goal, const int _nTicks) {
  // Need positive number of ticks.
  if(_nTicks <= 0)
    throw RunTimeException(WHERE) << "Divide by 0";
  if(_start.m_groupGraph != _goal.m_groupGraph)
    throw RunTimeException(WHERE) << "Cannot use two different groups (or group "
                                  << "roadmaps) with this operation currently!";

  // Find increment for all robots in formations.
  std::set<Robot*> found;
  auto group = this->m_groupGraph->GetGroup();

  for(auto formation : m_formations) {

    std::vector<Cfg> start;
    std::vector<Cfg> goal;

    for(auto robot : formation->GetRobots()) {
      start.push_back(_start.GetRobotCfg(robot));
      goal.push_back(_goal.GetRobotCfg(robot));
    }

    auto cfgs = formation->FindIncrement(start,goal,_nTicks);
    for(auto cfg : cfgs) {
      
      // Grab robot from cfg.
      auto robot = cfg.GetRobot();

      // Save cfg to local cfgs.
      auto index = group->GetGroupIndex(robot);
      SetRobotCfg(index,std::move(cfg));

      // Mark this robot as complete.
      found.insert(robot);
    }
  }


  // For each robot in the group, find the increment for the individual cfg
  // given the number of ticks found.
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {

    // Check that robots have not already been incremented by formation increments.
    auto robot = this->GetRobot(i);
    if(found.count(robot))
      continue;

    IndividualCfg incr(robot);
    incr.FindIncrement(_start.GetRobotCfg(i), _goal.GetRobotCfg(i), _nTicks);
    SetRobotCfg(i, std::move(incr));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
FindIncrement(const GroupCfg& _start, const GroupCfg& _goal, int* const _nTicks,
    const double _positionRes, const double _orientationRes) {
  const GroupCfg<GraphType> diff = _goal - _start;

  *_nTicks = std::max(1., std::ceil(std::max(
                      diff.PositionMagnitude() / _positionRes,
                      diff.OrientationMagnitude() / _orientationRes)));

  FindIncrement(_start, _goal, *_nTicks);
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
InBounds(const Boundary* const _b) const noexcept {
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    if(!GetRobotCfg(i).InBounds(_b))
      return false;

  return true;
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
InBounds(const Environment* const _env) const noexcept {
  return InBounds(_env->GetBoundary());
}

template <typename GraphType>
void
GroupCfg<GraphType>::
GetRandomGroupCfg(const Boundary* const _b) {
  std::set<Robot*> found;
  auto group = this->m_groupGraph->GetGroup();

  for(size_t i = 0; i < this->GetNumRobots(); i++) {
    this->m_vids[i] = INVALID_VID;
  }
  InitializeLocalCfgs();

  for(auto formation : m_formations) {
    auto cfgs = formation->GetRandomFormationCfg(_b);
    for(auto cfg : cfgs) {

      // Grab robot from cfg.
      auto robot = cfg.GetRobot();

      // Save cfg to local cfgs.
      auto index = group->GetGroupIndex(robot);
      m_localCfgs[index] = cfg;

      // Mark this robot as complete.
      found.insert(robot);
    }
  }

  auto robots = group->GetRobots();
  for(size_t i = 0; i < robots.size(); i++) {
    auto robot = robots[i];

    // Check if robot cfg was set by formation sampling.
    if(found.count(robot))
      continue;

    // If not, get a random configuration for it.
    Cfg cfg(robot);
    cfg.GetRandomCfg(_b);

    // Save cfg to local cfgs.
    m_localCfgs[i] = cfg;
  }
}

template <typename GraphType>
void
GroupCfg<GraphType>::
GetRandomGroupCfg(Environment* _env) {
  GetRandomGroupCfg(_env->GetBoundary());
}

template <typename GraphType>
void
GroupCfg<GraphType>::
NormalizeOrientation(const std::vector<size_t>& _robots) noexcept {
  if(_robots.empty()) // Do all robots in this case.
    for(size_t i = 0; i < this->GetNumRobots(); ++i)
      GetRobotCfg(i).NormalizeOrientation();
  else
    for(size_t i : _robots)
      GetRobotCfg(i).NormalizeOrientation();
}

/*------------------------------ Output Helpers ------------------------------*/

template <typename GraphType>
std::string
GroupCfg<GraphType>::
PrettyPrint(const size_t _precision) const {
  std::ostringstream oss;
  oss.precision(_precision);
  oss << "{ ";
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const IndividualCfg& robotCfg = GetRobotCfg(i);
    if(IsLocalCfg(i))
      oss << "Local: ";
    oss << robotCfg.PrettyPrint(_precision) << ", ";
  }
  oss << " }";

  return oss.str();
}

/*----------------------------------------------------------------------------*/

template <typename GraphType>
bool
GroupCfg<GraphType>::
IsLocalCfg(const size_t _robotIndex) const noexcept {
  // Only true if there is local data (meaning INVALID_VID is present)
  return this->m_vids[_robotIndex] == INVALID_VID;
}

template <typename GraphType>
void
GroupCfg<GraphType>::
InitializeFormations() noexcept {
  m_formations = this->m_groupGraph->GetActiveFormations();
}

template <typename GraphType>
void
GroupCfg<GraphType>::
InitializeLocalCfgs() noexcept {
  // We will assume the local cfgs are initialized if the container size is
  // correct.
  const size_t numRobots = this->GetNumRobots();
  if(m_localCfgs.size() == numRobots)
    return;

  m_localCfgs.clear();
  m_localCfgs.resize(numRobots);

  for(size_t i = 0; i < numRobots; ++i)
    m_localCfgs[i] = IndividualCfg(this->GetRobot(i));
}

/*----------------------------------------------------------------------------*/

template <typename GraphType>
std::ostream&
operator<<(std::ostream& _os, const GroupCfg<GraphType>& _groupCfg) {
  // Might not need to be hidden behind GROUP_MAP, but doing for consistency
#ifdef GROUP_MAP
  _os << "0 ";
#endif

  // Loop through all robots in the group and print each one's cfg in order.
  for(size_t i = 0; i < _groupCfg.GetNumRobots(); ++i)
    _os << _groupCfg.GetRobotCfg(i);

  return _os;
}

/*----------------------------------------------------------------------------*/
