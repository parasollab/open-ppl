#ifndef INTERMEDIATE_EDGE_VALIDITY_CHECKER_H_
#define INTERMEDIATE_EDGE_VALIDITY_CHECKER_H_

#include "EdgeValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup EdgeValidityCheckers
/// @brief Validates an edge by calling a local planner and collision checking
/// all intermediates created by that local planner separately. 
/// 
/// @note Assumes that the validity chacker is a collision detection validity
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

    bool ValidateEdge(VID _u, VID _v, vector<size_t>& _collisions) override;
    bool ValidateEdge(CfgType& _c1, CfgType& _c2, vector<size_t>& _collisions) override;

  private:

    ///@name helper functions
    ///@{


    /// @TODO
    /// @note Clears the vector _collisions 
    bool ReportIntermediateCollisions(vector<CfgType>& _intermediates, vector<size_t>& _collisions);

    ///@}
    ///@name member variables
    ///@{

    ///@}

    string m_lpLabel;               ///< the local planner label

    string m_vcLabel;               ///< the validity checker label. Must be a label of a
                                    ///<  collision detection validity checker.
    bool m_overideLp{false};        ///< Flag for using a different local planner than the one
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

  m_overideLp = _node.Read("overrideLp", false, false, "Overide local planner stored in edge");

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

    return ReportIntermediateCollisions(intermediates, _collisions);
  }

}


template <typename MPTraits>
bool
IntermediatesEdgeValidityChecker<MPTraits>::
ValidateEdge(CfgType& _c1, CfgType& _c2, vector<size_t>& _collisions) {

  LocalPlannerMethod<MPTraits>* lp = this->GetLocalPlanner(m_lpLabel);

  Environment* env = this->GetEnvironment();

  vector<CfgType> intermediates = lp->BlindPath({_c1, _c2}, env->GetPositionRes(), env->GetOrientationRes());
  
  return ReportIntermediateCollisions(intermediates, _collisions);

}


template <typename MPTraits>
bool
IntermediatesEdgeValidityChecker<MPTraits>::
ReportIntermediateCollisions(vector<CfgType>& _intermediates, vector<size_t>& _collisions) {

  bool res = true;

  _collisions.clear();

  ValidityCheckerMethod<MPTraits>* vc = this->GetValidityChecker(m_vcLabel);

  // This is the reason why a collision detection validity checker is required
  auto cd = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(vc);

  auto env = this->GetEnvironment();

  const string callee = this->GetNameAndLabel() + "::IntermediatesEdgeValidityChecker";

  size_t numObst = env->NumObstacles();

  vector<bool> collisionIndicators = vector<bool>(numObst, false);

  CDInfo obstInfo;

  bool inCollision;

  for (auto &cfg: _intermediates) {

    auto robot = cfg.GetRobot();
    cfg.ConfigureRobot();

    const MultiBody* robotMultiBody = robot->GetMultiBody();

    for (size_t i = 0; i < numObst; i++) {

      obstInfo.ResetVars(true);

      //@todo can maybe report penetration depth here in order to provide 
      //  additional information
      inCollision = cd->IsMultiBodyCollision(obstInfo,
        robotMultiBody, env->GetObstacle(i), callee);

      if (inCollision){
        collisionIndicators[i] = true;
        res = false;
      }
    }
    
  }

  for (size_t i = 0; i < numObst; i++){
    if (collisionIndicators[i])
      _collisions.push_back(i);
  }
  return res;

}
#endif
