#ifndef PMPL_MASKED_PROXIMITY_SAMPLER_GROUP_H_
#define PMPL_MASKED_PROXIMITY_SAMPLER_GROUP_H_

#include "MaskedSamplerMethodGroup.h"
#include "MPProblem/RobotGroup/GroupUtils.h"

#include <iostream>


////////////////////////////////////////////////////////////////////////////////
/// Specialized masked sampler for composite C-Spaces (for diassembly)
///
/// This sampler extends a random distance from m_startCfg (which must be set,
/// along with the active robots). The general idea of the sampler to have an
/// EST-like sample returned, hence choosing a node and then extending a
/// certain distance from it.
///
/// @todo This needs to be reimplemented to use the Filter method such that
///       m_startCfg is passed into it instead of being a class member.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MaskedProximitySamplerGroup : public MaskedSamplerMethodGroup<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename GroupCfgType::IndividualCfg IndividualCfg;

    ///@}
    ///@name Local Types
    ///@{

    using typename SamplerMethod<MPTraits>::GroupOutputIterator;

    typedef typename std::vector<size_t> RobotFormation;

    ///@}
    ///@name Construction
    ///@{

    MaskedProximitySamplerGroup();

    MaskedProximitySamplerGroup(XMLNode& _node);

    virtual ~MaskedProximitySamplerGroup() = default;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        std::vector<GroupCfgType>& _result,
        std::vector<GroupCfgType>& _collision) override;

    ///@}
    ///@name Internal State
    ///@{

    double m_maxDist{1.0}; ///< Maximum distance to move away from a cfg.

    std::string m_vcLabel; ///< The validity checker to use.
    std::string m_dmLabel; ///< The distance metric to use.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MaskedProximitySamplerGroup<MPTraits>::
MaskedProximitySamplerGroup(XMLNode& _node)
    : MaskedSamplerMethodGroup<MPTraits>(_node) {
  this->SetName("MaskedProximitySamplerGroup");

  m_maxDist = _node.Read("maxDist", false, m_maxDist, 0.,
      numeric_limits<double>::max(),
      "The maximum distance to move from a cfg.");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "The validity checker to use for sampling");
  m_dmLabel = _node.Read("dmLabel", true, "",
      "The distance metric to use for sampling");
}


template <typename MPTraits>
MaskedProximitySamplerGroup<MPTraits>::
MaskedProximitySamplerGroup() : MaskedSamplerMethodGroup<MPTraits>() {
  this->SetName("MaskedProximitySamplerGroup");
}

/*---------------------------- Sampler Interface -----------------------------*/

template <typename MPTraits>
bool
MaskedProximitySamplerGroup<MPTraits>::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision) {
  if(!this->m_startCfg.GetGroupRoadmap())
    throw RunTimeException(WHERE, "Invalid start cfg!");

  //The input cfg is a cfg from the roadmap. Extend up to m_maxDist away
  // from _cfg and return (extendedCfg - roadmap[0]).
  auto vc = this->GetValidityChecker(m_vcLabel);

  RobotFormation robotList = this->m_activeRobots;  // Copy for potential reordering.

  ///@TODO: For now I'm assuming that the bodies all have the same dofsPerBody,
  ///       this should be updated to be arbitrary in the future.
  const IndividualCfg& robotCfg = _cfg.GetRobotCfg(robotList[0]);
  const size_t posDofsPerBody = robotCfg.PosDOF();
  const size_t oriDofsPerBody = robotCfg.OriDOF();
  const size_t dofsPerBody = posDofsPerBody + oriDofsPerBody;
  const bool isRotational = oriDofsPerBody > 0;

  GroupCfgType extendedCfg = _cfg; // The cfg to be in proximity of.

  //Get random direction, scale it down using the range [0,1), scale it up to
  // m_maxDist then add the new direction to the starting cfg.
  const double x = DRand()-.5;
  const double y = DRand()-.5;
  double z;
  if(posDofsPerBody == 2)
    z = 0.;
  else
    z = DRand()-.5;

  mathtool::Vector3d dir(x, y, z);
  dir.selfNormalize();
  dir *= m_maxDist;

  extendedCfg.AddDofsForRobots(dir, robotList);

  /// Now the translation has been applied to all the bodies. Next, handle
  /// rotations appropriately:
  const double rotScale = 2.0;
  if (robotList.size() == 1) { // Single body case
    for(size_t i = posDofsPerBody; i < dofsPerBody; ++i)
      extendedCfg.GetRobotCfg(robotList[0])[i] += (DRand() - .5) * rotScale;
  }
  else if(isRotational) {
    // Rotate about the body number in bodyList[0].
    const size_t randInd = LRand() % robotList.size();
    if(randInd > 0)
      std::swap(robotList[0], robotList[randInd]);

    //Generate the random rotation, taking into account rotation bounds:
    const double piScaled = PI * rotScale; // Needs PI for Euler Angle creation.
    const double gamma = piScaled * (DRand() - .5);
    double alpha, beta;
    if(oriDofsPerBody == 1) {
      alpha = 0;
      beta = 0;
    }
    else {
      alpha = piScaled * (DRand() - .5);
      beta = piScaled * (DRand() - .5);
    }
    const mathtool::EulerAngle deltaRot(gamma, beta, alpha); // ZYX order?
    const mathtool::Orientation rotMat(deltaRot);

    if(this->m_debug)
      std::cout << "robotList = " << robotList << std::endl
                << "Cfg before rotation: " << extendedCfg.PrettyPrint()
                << std::endl;


    //Preserves any existing translations, as it will undo, rotate, and redo
    // any translation (and additional components due to not rotating about itself)
    //TODO::Update this to new Formation representation.
    throw RunTimeException(WHERE) << "Not currently supported.";
    //extendedCfg.RotateRobotFormationAboutLeader(robotList, rotMat, this->m_debug);

    if(this->m_debug)
      std::cout << "Cfg after rotation: " << extendedCfg.PrettyPrint()
                << std::endl;
  }

  extendedCfg.NormalizeOrientation(robotList);

  if(this->m_debug)
    std::cout << "Cfg after orientation normalization: "
              << extendedCfg.PrettyPrint() << std::endl << std::endl;

  //Check that the sample is in bounds and valid:
  const std::string callee = this->GetNameAndLabel() + "::Sampler";
  if(!extendedCfg.InBounds(_boundary) ||
     //!vc->IsValid(extendedCfg, callee, this->m_activeRobots)) {
     !vc->IsValid(extendedCfg, callee)) {
    _collision.push_back(extendedCfg);
    return false;
  }

  //The sample given back must account for startCfg offset, which is built into
  // the cfg we started from, so no adjustment is needed here.
  _result.push_back(extendedCfg);
  return true;
}

/*----------------------------------------------------------------------------*/

#endif
