#include "IntermediatesEdgeValidityChecker.h"

#include "MPLibrary/MPLibrary.h"


IntermediatesEdgeValidityChecker::
IntermediatesEdgeValidityChecker() :
EdgeValidityCheckerMethod() {
  this->SetName("IntermediatesEdgeValidityChecker");
}


IntermediatesEdgeValidityChecker::
IntermediatesEdgeValidityChecker(XMLNode& _node) :
EdgeValidityCheckerMethod(_node) {
  this->SetName("IntermediatesEdgeValidityChecker");

  m_lpLabel = _node.Read("lpLabel", false, "", "The local planner label.");

  m_vcLabel = _node.Read("vcLabel", false, "", "The validity label.");

  m_overrideLp = _node.Read("overrideLp", false, m_overrideLp, "Overide local planner stored in edge");

}


void
IntermediatesEdgeValidityChecker::
Initialize() {
  // StraightLine(m_vcLabel, m_binaryEvaluation, m_saveIntermediates);
}


bool
IntermediatesEdgeValidityChecker::
ValidateEdge(VID _source, VID _target, std::vector<size_t>& _collisions) {

  RoadmapType* r = this->GetRoadmap();

  if (m_overrideLp){
    Cfg c1 = r->GetVertex(_source);
    Cfg c2 = r->GetVertex(_target);

    return ValidateEdge(c1, c2, _collisions);
  }

  else{
    std::vector<Cfg> intermediates = this->GetMPLibrary()->ReconstructEdge(r, _source, _target);

    if (intermediates.size() == 0 )
      throw RunTimeException(WHERE, "Edge local planner did not provide intermediates.\
                                            Consider overriding edge local planners.");

    bool res = (HandleIntermediates(intermediates, _collisions) > 0) ? true : false;

    return res;
  }
}


bool
IntermediatesEdgeValidityChecker::
ValidateEdge(Cfg& _c1, Cfg& _c2, std::vector<size_t>& _collisions) {

  LocalPlannerMethod* lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);

  Environment* env = this->GetEnvironment();

  std::vector<Cfg> intermediates = lp->BlindPath({_c1, _c2}, env->GetPositionRes(), env->GetOrientationRes());
  
  bool res = (HandleIntermediates(intermediates, _collisions) > 0) ? true : false;

  return res;

}


double
IntermediatesEdgeValidityChecker::
EdgeWeightedClearance(VID _u, VID _v) {

  std::vector<Cfg> intermediates;

  RoadmapType* r = this->GetRoadmap();

  if (m_overrideLp){
    
    Cfg c1 = r->GetVertex(_u);
    Cfg c2 = r->GetVertex(_v);

    return EdgeWeightedClearance(c1, c2);
    
  }

  else{
    intermediates = this->GetMPLibrary()->ReconstructEdge(r, _u, _v);

    if (intermediates.size() == 0 )
      throw RunTimeException(WHERE, "Edge local planner did not provide intermediates.\
                                            Consider overriding edge local planners.");
  }

  std::vector<size_t> collisions = {};

  double clearance = HandleIntermediates(intermediates, collisions, true);

  return clearance;
}


double
IntermediatesEdgeValidityChecker::
EdgeWeightedClearance(Cfg& _c1, Cfg& _c2) {

  LocalPlannerMethod* lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);

  Environment* env = this->GetEnvironment();

  std::vector<Cfg> intermediates = lp->BlindPath({_c1, _c2}, env->GetPositionRes(), env->GetOrientationRes());

  std::vector<size_t> collisions = {};
  
  double clearance = HandleIntermediates(intermediates, collisions, true);

  return clearance;

}


double
IntermediatesEdgeValidityChecker::
HandleIntermediates(std::vector<Cfg>& _intermediates, std::vector<size_t>& _collisions, bool _reportClearance) {

  double res = _reportClearance ? MAX_DBL : 1.0;

  _collisions.clear();

  /// @todo pqp_solid is currently the only implemented cllision checking validity
  /// checker supporting clearance queries 
  ValidityCheckerMethod* vc = (_reportClearance)? this->GetMPLibrary()->GetValidityChecker("pqp_solid") :
                                                  this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

  /// This is the reason why a collision detection validity checker is required
  auto cd = dynamic_cast<CollisionDetectionValidityMethod*>(vc);

  auto env = this->GetEnvironment();

  size_t numObst = env->NumObstacles();

  /// Set of flags for collision reporting
  std::vector<bool> collisionIndicators = std::vector<bool>(numObst, false);

  /// Preparing arguments for a CD call
  const string callee = this->GetNameAndLabel() + "::IntermediatesEdgeValidityChecker";

  CDInfo obstInfo;

  bool inCollision;

  for (auto &cfg: _intermediates) {

    auto robot = cfg.GetRobot();
    cfg.ConfigureRobot();

    const MultiBody* robotMultiBody = robot->GetMultiBody();

    for (size_t i = 0; i < numObst; i++) {
      obstInfo.ResetVars(true);

      MultiBody* obstMultiBody = env->GetObstacle(i);

      inCollision = cd->IsMultiBodyCollision(obstInfo, robotMultiBody, obstMultiBody, callee);

      /// registering collisions and clearance according to flags.
      if (this->m_reportCollisions && inCollision) {
        collisionIndicators[i] = true;
      }
      if (_reportClearance){
        double weightedNewClearance = std::max(obstInfo.m_minDist - obstMultiBody->GetWeight(), 0.0);

        res = std::min(weightedNewClearance, res); 
      }
      else {
        if (inCollision){
          res = -1.0;
        }
      }
    }  
  }

    
  if (this->m_reportCollisions) { 
    for (size_t i = 0; i < numObst; i++){
      if (collisionIndicators[i])
        _collisions.push_back(i);
    }
  }

  return res;
}
