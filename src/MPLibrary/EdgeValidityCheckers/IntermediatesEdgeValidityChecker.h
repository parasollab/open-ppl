#ifndef INTERMEDIATE_EDGE_VALIDITY_CHECKER_H_
#define INTERMEDIATE_EDGE_VALIDITY_CHECKER_H_

#include "EdgeValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup EdgeValidityCheckers
/// @brief Validates an edge by calling a local planner and collision checking
/// all intermediates created by that local planner separately. 
/// 
/// @note Assumes that the validity checker is a collision detection validity
/// checker.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class IntermediatesEdgeValidityChecker : public EdgeValidityCheckerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType               CfgType;
    typedef typename MPTraits::RoadmapType           RoadmapType;
    typedef typename RoadmapType::VID                VID;
    typedef typename RoadmapType::EID                EID;
    typedef typename RoadmapType::adj_edge_iterator  EI;

    IntermediatesEdgeValidityChecker();
    IntermediatesEdgeValidityChecker(XMLNode& _node);

    void Initialize() override;


    /// Checks the validity of an edge using a set of intermediate cfgs
    /// each validated independently.
    /// @param _u The source of the edge to validate
    /// @param _v The target of the edge to validate
    /// @param _collisions An empty container that will be populated with
    ///   a list of indices of obstacles with which the edge collides.
    /// @return True if all of the intermediate cfgs are in free space.
    bool ValidateEdge(VID _u, VID _v, vector<size_t>& _collisions) override;
    bool ValidateEdge(CfgType& _c1, CfgType& _c2, vector<size_t>& _collisions) override;

    /// Determines the clearance of the edge given by two VIDs.
    /// @param _u The source of the edge
    /// @param _v The target of the edge
    /// @return The clearance of the edge
    /// @note Takes into account weighted obstacles
    double EdgeWeightedClearance(VID _u, VID _v);
    double EdgeWeightedClearance(CfgType& _c1, CfgType& _c2);


  private:

    ///@name helper functions
    ///@{


    /// Calls a collision detector for a set of intermediate cfgs , and can 
    ///   populate the collisions vector with the indices of obstacles in collision
    ///   with these cfgs or report the clearance.
    /// @param _intermediates A vector of intermediate cfgs.
    /// @param _collisions A vector to be populated with the indices of obstacles in collision
    ///   with the cfgs.
    /// @return -/+ 1.0 to report invalid/valid if used for collision reporting, and
    ///   otherwise returns the weighted clearance of the edge.
    /// @note Clears the vector _collisions before populating it
    double HandleIntermediates(vector<CfgType>& _intermediates, vector<size_t>& _collisions, bool _reportClearance = false);

    /// @todo Maybe can get a nicer implementation using a clearance map of CDInfo
    /// Iterates over a set of intermediates in order to find the minimum weighted clearance
    /// @param _intermediates A vector of intermediate cfgs.
    /// @return The minimum weighted clearance
    double intermediateClearance(vector<CfgType>& _intermediates);

    ///@}
    ///@name member variables
    ///@{

    ///@}

    string m_lpLabel;                 ///< the local planner label

    string m_vcLabel;                 ///< the validity checker label. Must be a label of a
                                      ///<  collision detection validity checker.
    bool m_overideLp{false};          ///< Flag for using a different local planner than the one
                                      ///<  stored in the roadmap edges
};

template <typename MPTraits>
IntermediatesEdgeValidityChecker<MPTraits>::
IntermediatesEdgeValidityChecker() :
EdgeValidityCheckerMethod<MPTraits>() {
  this->SetName("IntermediatesEdgeValidityChecker");
}

template <typename MPTraits>
IntermediatesEdgeValidityChecker<MPTraits>::
IntermediatesEdgeValidityChecker(XMLNode& _node) :
EdgeValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("IntermediatesEdgeValidityChecker");

  m_lpLabel = _node.Read("lpLabel", false, "", "The local planner label.");

  m_vcLabel = _node.Read("vcLabel", false, "", "The validity label.");

  m_overideLp = _node.Read("overrideLp", false, m_overrideLp, "Overide local planner stored in edge");

}

template <typename MPTraits>
void
IntermediatesEdgeValidityChecker<MPTraits>::
Initialize() {
  // StraightLine(m_vcLabel, m_binaryEvaluation, m_saveIntermediates);
}



template <typename MPTraits>
bool
IntermediatesEdgeValidityChecker<MPTraits>::
ValidateEdge(VID _source, VID _target, vector<size_t>& _collisions) {

  RoadmapType* r = this->GetRoadmap();

  if (m_overideLp){
    CfgType c1 = r->GetVertex(_source);
    CfgType c2 = r->GetVertex(_target);

    return ValidateEdge(c1, c2, _collisions);
  }

  else{
    vector<CfgType> intermediates = this->GetMPLibrary()->ReconstructEdge(r, _source, _target);

    if (intermediates.size() == 0 )
      throw RunTimeException(WHERE, "Edge local planner did not provide intermediates.\
                                            Consider overriding edge local planners.");

    bool res = (HandleIntermediates(intermediates, _collisions) > 0) ? true : false;

    return res;
  }

}


template <typename MPTraits>
bool
IntermediatesEdgeValidityChecker<MPTraits>::
ValidateEdge(CfgType& _c1, CfgType& _c2, vector<size_t>& _collisions) {

  LocalPlannerMethod<MPTraits>* lp = this->GetLocalPlanner(m_lpLabel);

  Environment* env = this->GetEnvironment();

  vector<CfgType> intermediates = lp->BlindPath({_c1, _c2}, env->GetPositionRes(), env->GetOrientationRes());
  
  bool res = (HandleIntermediates(intermediates, _collisions) > 0) ? true : false;

  return res;

}


template <typename MPTraits>
double
IntermediatesEdgeValidityChecker<MPTraits>::
EdgeWeightedClearance(VID _u, VID _v) {

  vector<CfgType> intermediates;

  RoadmapType* r = this->GetRoadmap();

  if (m_overideLp){
    
    CfgType c1 = r->GetVertex(_u);
    CfgType c2 = r->GetVertex(_v);

    return EdgeWeightedClearance(c1, c2);
    
  }

  else{
    intermediates = this->GetMPLibrary()->ReconstructEdge(r, _u, _v);

    if (intermediates.size() == 0 )
      throw RunTimeException(WHERE, "Edge local planner did not provide intermediates.\
                                            Consider overriding edge local planners.");
  }

  vector<size_t> collisions = {};

  double clearance = HandleIntermediates(intermediates, collisions, true);

  return clearance;
}


template <typename MPTraits>
double
IntermediatesEdgeValidityChecker<MPTraits>::
EdgeWeightedClearance(CfgType& _c1, CfgType& _c2) {

  LocalPlannerMethod<MPTraits>* lp = this->GetLocalPlanner(m_lpLabel);

  Environment* env = this->GetEnvironment();

  vector<CfgType> intermediates = lp->BlindPath({_c1, _c2}, env->GetPositionRes(), env->GetOrientationRes());

  vector<size_t> collisions = {};
  
  double clearance = HandleIntermediates(intermediates, collisions, true);

  return clearance;

}


template <typename MPTraits>
double
IntermediatesEdgeValidityChecker<MPTraits>::
HandleIntermediates(vector<CfgType>& _intermediates, vector<size_t>& _collisions, bool _reportClearance) {

  double res = _reportClearance ? MAX_DBL : 1.0;

  _collisions.clear();

  /// @todo pqp_solid is currently the only implemented cllision checking validity
  /// checker supporting clearance queries 
  ValidityCheckerMethod<MPTraits>* vc = (_reportClearance)? this->GetValidityChecker("pqp_solid") :
                                                            this->GetValidityChecker(m_vcLabel);

  /// This is the reason why a collision detection validity checker is required
  auto cd = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(vc);

  auto env = this->GetEnvironment();

  size_t numObst = env->NumObstacles();

  /// Set of flags for collision reporting
  vector<bool> collisionIndicators = vector<bool>(numObst, false);

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
#endif
