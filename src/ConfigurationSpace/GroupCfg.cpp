#include "GroupCfg.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "nonstd.h"


/*------------------------------- Construction -------------------------------*/

GroupCfg::
GroupCfg(GraphType* const _groupMap, const bool _init) : m_groupMap(_groupMap) {

  if(m_groupMap)
    m_vids.resize(m_groupMap->GetGroup()->Size(), INVALID_VID);

  if(_init) {
    if(!m_groupMap)
      throw RunTimeException(WHERE, "_init was true but m_groupMap was null in "
                                    "constructor!");

    // Create local cfgs of default values (0's) for all robots in group.
    for(size_t i = 0; i < this->GetNumRobots(); ++i)
      this->SetCfg(i, IndividualCfg(this->GetRobot(i)));
  }
}


/*--------------------------------- Equality ---------------------------------*/

bool
GroupCfg::
operator==(const GroupCfg& _other) const noexcept {
  // If _other is from another map, these are not the same.
  if(m_groupMap != _other.m_groupMap)
    return false;

  // Else, compare VIDs if both are valid, or by-value other wise.
  for(size_t i = 0; i < m_vids.size(); ++i) {
    const VID thisVID  = m_vids[i],
              otherVID = _other.m_vids[i];

    if(thisVID != INVALID_VID and otherVID != INVALID_VID) {
      if(thisVID != otherVID)
        return false;
    }
    else if(GetRobotCfg(i) != _other.GetRobotCfg(i))
      return false;
  }

  return true;
}


bool
GroupCfg::
operator!=(const GroupCfg& _other) const noexcept {
  return !(*this == _other);
}


GroupCfg
GroupCfg::
operator+(const GroupCfg& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg += _other);
}


GroupCfg
GroupCfg::
operator-(const GroupCfg& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg -= _other);
}


GroupCfg
GroupCfg::
operator*(const double& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg *= _other);
}


GroupCfg&
GroupCfg::
operator+=(const GroupCfg& _other) {
  // We must require the exact same group roadmap, which indicates everything
  // lines up between the two cfgs (namely the exact robots/order of the group).
  if(this->m_groupMap != _other.m_groupMap)
    throw RunTimeException(WHERE, "Cannot add GroupCfgs with different group "
                                  "roadmaps!");

  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->SetCfg(i, this->GetRobotCfg(i) + _other.GetRobotCfg(i));

  return *this;
}


GroupCfg&
GroupCfg::
operator-=(const GroupCfg& _other) {
  // We must require the exact same group roadmap, which indicates everything
  // lines up between the two cfgs (namely the exact robots/order of the group).
  if(this->m_groupMap != _other.m_groupMap)
    throw RunTimeException(WHERE, "Cannot add GroupCfgs with different group "
                                  "roadmaps!");

  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->SetCfg(i, this->GetRobotCfg(i) - _other.GetRobotCfg(i));

  return *this;
}


GroupCfg&
GroupCfg::
operator*=(const double& _val) {
  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->SetCfg(i, this->GetRobotCfg(i) * _val);

  return *this;
}


/*-------------------------------- Accessors ---------------------------------*/

void
GroupCfg::
SetCfg(Robot* const _robot, Cfg&& _cfg) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  this->SetCfg(index, std::move(_cfg));
}


void
GroupCfg::
SetCfg(const size_t _index, Cfg&& _cfg) {
  // Allocate space for local cfgs if not already done.
  m_localCfgs.resize(m_groupMap->GetGroup()->Size());

  m_localCfgs[_index] = std::move(_cfg);
  m_vids[_index] = INVALID_VID;
}


void
GroupCfg::
SetCfg(Robot* const _robot, const VID _vid) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  this->SetCfg(index, _vid);
}


void
GroupCfg::
SetCfg(const size_t _index, const VID _vid) {
  m_vids[_index] = _vid;
}


void
GroupCfg::
ClearLocalData() {
  m_localCfgs.clear();
}


Cfg&
GroupCfg::
GetRobotCfg(const size_t _index) {
  const Cfg& cfg = const_cast<const GroupCfg*>(this)->GetRobotCfg(_index);
  return const_cast<Cfg&>(cfg);
}


const Cfg&
GroupCfg::
GetRobotCfg(const size_t _index) const {
  const VID vid = GetVID(_index);

  // If we have a valid VID for this robot, fetch its configuration from its
  // individual roadmap.
  if(vid != INVALID_VID) {
    auto roadmap = m_groupMap->GetRoadmap(_index);
    return roadmap->GetVertex(vid);
  }

  try {
    return m_localCfgs.at(_index);
  }
  catch(const std::out_of_range&) {
    std::ostringstream oss;
    oss << "Requested configuration for robot " << _index
        << ", but no roadmap or local cfg exists.";
    throw RunTimeException(WHERE, oss.str());
  }
}


GroupCfg::VID
GroupCfg::
GetVID(const size_t _index) const noexcept {
  return m_vids[_index];
}


Robot*
GroupCfg::
GetRobot(const size_t _index) const {
  Robot* const robot = m_groupMap->GetGroup()->GetRobot(_index);

  //TODO: remove this after we are very sure things are working for the
  //      whole assembly planning strategy.
  if(!robot)
    throw RunTimeException(WHERE, "Error! Robot pointer was null.");

  return robot;
}


size_t
GroupCfg::
PosDOF(const size_t _index) const {
  return this->GetRobot(_index)->GetMultiBody()->PosDOF();
}


size_t
GroupCfg::
OriDOF(const size_t _index) const {
  return this->GetRobot(_index)->GetMultiBody()->OrientationDOF();
}


size_t
GroupCfg::
DOF(const size_t _index) const {
  return this->GetRobot(_index)->GetMultiBody()->DOF();
}

bool
GroupCfg::
IsNonholonomic() const noexcept {
  // TODO: This should ideally check if ANY robot is nonholonomic in the group.
  return this->GetRobot(0)->IsNonholonomic();
}


double
GroupCfg::
Magnitude() const {
  double result = 0;
  for(size_t robot = 0; robot < GetNumRobots(); ++robot) {
    const IndividualCfg robotCfg = GetRobotCfg(robot);
    for(size_t i = 0; i < robotCfg.DOF(); ++i)
      result += robotCfg[i] * robotCfg[i];
  }
  return std::sqrt(result);
}


double
GroupCfg::
PositionMagnitude() const {
  double result = 0;
  for(size_t robot = 0; robot < GetNumRobots(); ++robot) {
    const IndividualCfg robotCfg = GetRobotCfg(robot);
    for(size_t i = 0; i < robotCfg.DOF(); ++i)
      if(robotCfg.GetMultiBody()->GetDOFType(i) == DofType::Positional)
        result += robotCfg[i] * robotCfg[i];
  }
  return std::sqrt(result);
}


double
GroupCfg::
OrientationMagnitude() const {
  double result = 0;
  for(size_t robot = 0; robot < GetNumRobots(); ++robot) {
    const IndividualCfg robotCfg = GetRobotCfg(robot);
    for(size_t i = 0; i < robotCfg.DOF(); ++i)
      if(robotCfg.GetMultiBody()->GetDOFType(i) != DofType::Positional)
        result += robotCfg[i] * robotCfg[i];
  }
  return std::sqrt(result);
}


/*--------------------------Configuration Helpers-----------------------------*/

void
GroupCfg::
ConfigureRobot() const {
  for(size_t i = 0; i < GetNumRobots(); ++i)
    GetRobotCfg(i).ConfigureRobot();
}


bool
GroupCfg::
WithinResolution(const GroupCfg& _cfg, const double _posRes,
                 const double _oriRes) const {
  // Simply return false if any of them individually are not within a resolution
  for(size_t i = 0; i < GetNumRobots(); ++i)
    if(!GetRobotCfg(i).WithinResolution(_cfg.GetRobotCfg(i), _posRes, _oriRes))
      return false;

  return true;
}


/*------------------------------DOF Modifiers---------------------------------*/

void
GroupCfg::
RotateFormationAboutLeader(const Formation& _robotList,
                           const mathtool::Orientation& _rotation,
                           const bool _debug) {
  /// Note: Currently assumes all robots just have ONE body. The case of multi-
  /// bodied robots would need to be specially handled (right now it should just
  /// be split into multiple robots if a group is needed).

  const size_t leaderIndex = _robotList[0];

  // Get transformation of leader before rotation:
  const IndividualCfg& leaderCfg = GetRobotCfg(leaderIndex);
  leaderCfg.ConfigureRobot();

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
}


void
GroupCfg::
ApplyTransformationForRobots(const Formation& _robotList,
                             const mathtool::Transformation& _transform,
                             const mathtool::Transformation& _relativeTransform) {
  //Compute each robot's needed transformation and set dofs in cfg.
  for (const size_t robotIndex : _robotList) {
    const IndividualCfg& robotCfg = GetRobotCfg(robotIndex);

    // TODO: generalize this to handle robots with >1 body!
    if(robotCfg.GetMultiBody()->GetNumBodies() > 1)
      throw RunTimeException(WHERE, "Multiple bodies not supported!");

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
}


void
GroupCfg::
AddDofsForRobots(const std::vector<double>& _dofs,
                 const Formation& _robots) {
  // This function adds all dofs (assumes #dofs per body is correct) to each
  // body given in _bodies to _cfg.
  for(const size_t robotIndex : _robots) {
    IndividualCfg cfg = this->GetRobotCfg(robotIndex);

    // TODO: Might want to remove this check.
    if(_dofs.size() != cfg.DOF())
      throw RunTimeException(WHERE, "Error! Trying to add in wrong number of "
                                    "dofs " + std::to_string(_dofs.size()) +
                                    " to body " + std::to_string(robotIndex));

    for(unsigned int i = 0; i < _dofs.size(); ++i)
      cfg[i] += _dofs[i]; // Add in each dof value for this robot's cfg.
    SetCfg(robotIndex, std::move(cfg));
  }
}


void
GroupCfg::
AddDofsForRobots(const mathtool::Vector3d& _dofs,
                 const Formation& _robots) {
  // This function adds all positional dofs in _dofs. It will handle 1-3 dofs
  // based on each body.
  for(const size_t robotIndex : _robots) {
    IndividualCfg robotCfg = this->GetRobotCfg(robotIndex);
    for(size_t i = 0; i < robotCfg.PosDOF(); ++i)
      robotCfg[i] += _dofs[i]; // Add in each dof value for this robot's cfg.
    SetCfg(robotIndex, std::move(robotCfg));
  }
}


void
GroupCfg::
OverwriteDofsForRobots(const std::vector<double>& _dofs,
                       const Formation& _robots) {
  // For each robot given, set the individual cfg of this group cfg.
  for(const size_t robotIndex : _robots) {
    IndividualCfg newIndividualCfg(GetRobot(robotIndex));
    newIndividualCfg.SetData(_dofs); // SetData checks dofs to be correct size.
    SetCfg(robotIndex, std::move(newIndividualCfg));
  }
}


void
GroupCfg::
OverwriteDofsForRobots(const mathtool::Vector3d& _dofs,
                       const Formation& _robots) {
  // For each robot given, set the individual cfg of this group cfg.
  for(const size_t robotIndex : _robots) {
    IndividualCfg newIndividualCfg(GetRobot(robotIndex));
    newIndividualCfg.SetLinearPosition(_dofs); // SetData checks dofs to be correct size.
    SetCfg(robotIndex, std::move(newIndividualCfg));
  }
}


void
GroupCfg::
OverwriteDofsForRobots(const GroupCfg& _fromCfg,
                       const Formation& _robots) {
  // For each robot given, set the individual cfg of this group cfg.
  for(const size_t robotIndex : _robots) {
    Cfg robotCfg = _fromCfg.GetRobotCfg(robotIndex);
    SetCfg(robotIndex, std::move(robotCfg));
  }
}


void
GroupCfg::
SetData(const std::vector<double>& _dofs) {
  const size_t indRobotNumDof = DOF();
  if(_dofs.size() != (indRobotNumDof * GetNumRobots()))
    throw RunTimeException(WHERE, "Mismatched DOF sizes!");

  for(size_t robotNum = 0; robotNum < GetNumRobots(); ++robotNum) {
    const size_t robotOffset = robotNum * indRobotNumDof;
    for(size_t dofNum = 0; dofNum < indRobotNumDof; ++dofNum)
      GetRobotCfg(robotNum)[dofNum] = _dofs[robotOffset + dofNum];
  }
}


void
GroupCfg::
FindIncrement(const GroupCfg& _start, const GroupCfg& _goal, const int _nTicks){
  // Need positive number of ticks.
  if(_nTicks <= 0)
    throw RunTimeException(WHERE, "Divide by 0");
  if(_start.m_groupMap != _goal.m_groupMap)
    throw RunTimeException(WHERE, "Cannot use two different groups (or group "
                                  "roadmaps) with this operation currently!");

  // For each robot in the group, find the increment for the individual cfg
  // given the number of ticks found.
  for(size_t robotIndex = 0; robotIndex < GetNumRobots(); ++robotIndex) {
    IndividualCfg incr(GetRobot(robotIndex));
    incr.FindIncrement(_start.GetRobotCfg(robotIndex),
                       _goal.GetRobotCfg(robotIndex), _nTicks);
    SetCfg(robotIndex, std::move(incr));
  }
}


void
GroupCfg::
FindIncrement(const GroupCfg& _start, const GroupCfg& _goal,
              int* const _nTicks,
              const double _positionRes, const double _orientationRes) {
  const GroupCfg diff = _goal - _start;

  *_nTicks = std::max(1., std::ceil(std::max(
                      diff.PositionMagnitude() / _positionRes,
                      diff.OrientationMagnitude() / _orientationRes)));

  this->FindIncrement(_start, _goal, *_nTicks);
}


bool
GroupCfg::
InBounds(const Boundary* const _b) const noexcept {
  for(size_t i = 0; i < GetNumRobots(); ++i)
    if(!GetRobotCfg(i).InBounds(_b))
      return false;

  return true;
}


bool
GroupCfg::
InBounds(const Environment* const _env) const noexcept {
  return InBounds(_env->GetBoundary());
}


void
GroupCfg::
NormalizeOrientation(const Formation& _robots) noexcept {
  if(_robots.empty()) // Do all robots in this case.
    for(size_t i = 0; i < GetNumRobots(); ++i)
      GetRobotCfg(i).NormalizeOrientation();
  else
    for(size_t i : _robots)
      GetRobotCfg(i).NormalizeOrientation();
}


GroupCfg
GroupCfg::
ChangeRoadmap(GraphType* const _newRoadmap) const {
  // Check that groups are compatible.
  if(m_groupMap->GetGroup()->Size() != _newRoadmap->GetGroup()->Size())
    throw RunTimeException(WHERE, "Trying to change roadmaps on incompatible "
                                  "groups!");

  // Create new cfg using _roadmap and initializing all entries locally to 0.
  GroupCfg newCfg(_newRoadmap);

  // Put all individual cfgs into group cfg so that all are local:
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    newCfg.SetCfg(i, std::move(Cfg(this->GetRobotCfg(i))));

  return newCfg;
}


bool
GroupCfg::
IsLocalCfg(const size_t _robotIndex) const noexcept {
  // Only true if there is local data (meaning INVALID_VID is present)
  return m_vids[_robotIndex] == INVALID_VID;
}



/*-------------------------------Output Helpers-------------------------------*/

std::string
GroupCfg::
PrettyPrint(const size_t _precision) const {
  std::ostringstream oss;
  oss.precision(_precision);
  oss << "{ ";
  for(size_t i = 0; i < GetNumRobots(); ++i) {
    const IndividualCfg& robotCfg = GetRobotCfg(i);
    if(IsLocalCfg(i))
      oss << "Local: ";
    oss << robotCfg.PrettyPrint(_precision) << ", ";
  }
  oss << " }";

  return oss.str();
}



std::ostream& operator<<(std::ostream& _os, const GroupCfg& _groupCfg) {

// Might not need to be hidden behind GROUP_MAP, but doing for consistency
#ifdef GROUP_MAP
  _os << "0 ";
#endif

  // Loop through all robots in the group and print each one's cfg in order.
  for(size_t i = 0; i < _groupCfg.GetNumRobots(); ++i)
    _os << _groupCfg.GetRobotCfg(i);

  return _os;
}

