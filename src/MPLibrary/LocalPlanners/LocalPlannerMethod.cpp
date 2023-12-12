#include "LocalPlannerMethod.h"

#include "MPProblem/Environment/Environment.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

/*------------------------------- Construction -------------------------------*/

LocalPlannerMethod::
LocalPlannerMethod(const bool _saveIntermediates)
  : m_saveIntermediates(_saveIntermediates)
{ }


LocalPlannerMethod::
LocalPlannerMethod(XMLNode& _node) : MPBaseObject(_node) {
  m_saveIntermediates = _node.Read("saveIntermediates", false,
      m_saveIntermediates, "Save intermediate nodes");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

void
LocalPlannerMethod::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tSave intermediates: " << m_saveIntermediates
      << std::endl;
}

/*------------------------ LocalPlanner Interface ----------------------------*/

inline
bool
LocalPlannerMethod::
IsConnected(const Cfg& _start, const Cfg& _end,
    LPOutput* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath) {
  Cfg col(_start.GetRobot());
  return IsConnected(_start, _end, col, _lpOutput, _posRes,
      _oriRes, _checkCollision, _savePath);
}


bool
LocalPlannerMethod::
IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _col,
    GroupLPOutput* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, const Formation& _formation) {
  throw NotImplementedException(WHERE) << "No default implementation provided.";
}


inline
bool
LocalPlannerMethod::
IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupLPOutput* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, const Formation& _formation) {
  GroupCfgType col(_start.GetGroupRoadmap());
  return IsConnected(_start, _end, col, _lpOutput, _posRes,
                     _oriRes, _checkCollision, _savePath, _formation);
}


std::vector<Cfg>
LocalPlannerMethod::
BlindPath(const std::vector<Cfg>& _waypoints,
    const double _posRes, const double _oriRes) {
  // Blind local-plan between each intermediate,
  bool first = true;
  std::vector<Cfg> out;
  LPOutput lpOutput;
  for(auto iter = _waypoints.begin(); iter + 1 != _waypoints.end(); ++iter) {
    // If this isn't the first configuration, then its an intermediate and must
    // be added to the path.
    if(!first)
      out.push_back(*iter);
    else
      first = false;

    // Reconstruct the resolution-level edge and append it to the output.
    lpOutput.Clear();
    IsConnected(*iter, *(iter + 1), &lpOutput, _posRes, _oriRes, false, true);
    out.insert(out.end(), lpOutput.m_path.begin(), lpOutput.m_path.end());
  }
  return out;
}


std::vector<Cfg>
LocalPlannerMethod::
BlindPath(const std::vector<Cfg>& _waypoints) {
  auto env = this->GetEnvironment();
  return BlindPath(_waypoints, env->GetPositionRes(), env->GetOrientationRes());
}


std::vector<typename LocalPlannerMethod::GroupCfgType>
LocalPlannerMethod::
BlindPath(const std::vector<GroupCfgType>& _waypoints, const double _posRes,
    const double _oriRes, const Formation& _formation) {
  // Blind local-plan between each intermediate,
  bool first = true;
  std::vector<GroupCfgType> out;
  GroupLPOutput lpOutput(_waypoints.at(0).GetGroupRoadmap());
  for(auto iter = _waypoints.begin(); iter + 1 != _waypoints.end(); ++iter) {
    // If this isn't the first configuration, then its an intermediate and must
    // be added to the path.
    if(!first)
      out.push_back(*iter);
    else
      first = false;

    // Reconstruct the resolution-level edge and append it to the output.
    lpOutput.Clear();
    IsConnected(*iter, *(iter + 1), &lpOutput, _posRes, _oriRes, false, true);
    out.insert(out.end(), lpOutput.m_path.begin(), lpOutput.m_path.end());
  }
  return out;
}


std::vector<typename LocalPlannerMethod::GroupCfgType>
LocalPlannerMethod::
BlindPath(const std::vector<GroupCfgType>& _waypoints,
    const Formation& _formation) {
  auto env = this->GetEnvironment();
  return BlindPath(_waypoints, env->GetPositionRes(), env->GetOrientationRes(),
      _formation);
}

/*----------------------------------------------------------------------------*/
