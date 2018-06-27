#ifndef MASKED_PROXIMITY_SAMPLER_GROUP_H_
#define MASKED_PROXIMITY_SAMPLER_GROUP_H_

#include <iostream>

#include "MaskedSamplerMethodGroup.h"
#include "MPProblem/RobotGroup/GroupUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Specialized masked sampler for composite C-Spaces (for diassembly)
///
/// This Sampler takes a random cfg from the roadmap, extends a random distance
/// away from that cfg, and then returns the cfg to add to the root (VID 0) cfg
/// of the roadmap in order to get to the sampled cfg. That's to say it's the
/// CSpace vector from the root to the sample that gets returned.
///
/// The general idea of the sampler to have an EST-like sample returned, hence
/// choosing a node and then extending a certain distance from it.
///
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MaskedProximitySamplerGroup : public MaskedSamplerMethodGroup<MPTraits> {

  public:
    typedef typename MPTraits::MPLibrary::DistanceMetricPointer  DistanceMetricPointer;
    typedef typename MPTraits::MPLibrary::ValidityCheckerPointer ValidityCheckerPointer;

    ///@name Motion Planning Types
    ///@{

    typedef typename MaskedSamplerMethodGroup<MPTraits>::GroupCfgType GroupCfgType;
    typedef typename GroupCfgType::IndividualCfg IndividualCfg;
    typedef typename std::vector<GroupCfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<GroupCfgType>> OutputIterator;
    typedef typename std::vector<size_t> Formation;

    ///@}
    ///@name Construction
    ///@{

    MaskedProximitySamplerGroup();
    MaskedProximitySamplerGroup(XMLNode& _node);
    virtual ~MaskedProximitySamplerGroup() = default;

    ///@}

    /// Try to sample a set number of new configurations from a given boundary.
    /// @param[in] _numNodes The number of samples desired.
    /// @param[in] _maxAttempts The maximum number of attempts for each sample.
    /// @param[in] _boundary The boundary to sample from.
    /// @param[out] _result An iterator to storage for the new configurations.
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
            const Boundary* const _boundary, OutputIterator _result) override;

  protected:
    ///@name Sampler Rule
    ///@{

    /// Takes a single input configuration and applies the sampler rule to
    /// generate one or more output configurations.
    /// @param[in] _cfg The input configuration.
    /// @param[in] _boundary The sampling boundary.
    /// @param[out] _result The resulting output configurations.
    /// @param[out] _collision The (optional) return for failed attempts.
    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision) override;

    ///@}

    ///@name Parameters
    ///@{

    double m_maxDist{1.0}; /// Maximum distance to move away from a cfg.

    string m_vcLabel;
    string m_dmLabel;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MaskedProximitySamplerGroup<MPTraits>::
MaskedProximitySamplerGroup(XMLNode& _node) : MaskedSamplerMethodGroup<MPTraits>(_node) {
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
void
MaskedProximitySamplerGroup<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
       OutputIterator _result) {
  for(size_t i = 0; i < _numNodes; ++i) {
    //Get a random vertex from the graph and hand it to Sampler()

    vector<GroupCfgType> result;
    vector<GroupCfgType> collision;
    //Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      this->GetStatClass()->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(this->m_startCfg, _boundary, result, collision))
        break;
    }

    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(),
        result.size());
    _result = copy(result.begin(), result.end(), _result);
    if(this->m_debug)
      for(const GroupCfgType& cfg : result) {
        std::cout << "MaskedProximitySamplerGroup::Sample result = "
                  << cfg.PrettyPrint() << std::endl;
      }
  }
  if(this->m_debug)
    std::cout << std::endl; // To show where an entire Sample call ends.
}


template <typename MPTraits>
bool
MaskedProximitySamplerGroup<MPTraits>::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision) {
  if(!this->m_startCfg.GetGroupMap())
    throw RunTimeException(WHERE, "Invalid start cfg!");

  //The input cfg is a cfg from the roadmap. Extend up to m_maxDist away
  // from _cfg and return (extendedCfg - roadmap[0]).
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
//  auto map = this->GetGroupRoadmap();

  const string callee = this->GetNameAndLabel() + "::Sampler()";

  Formation robotList = this->m_activeRobots;  // Copy for potential reordering.

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

//  for(size_t bodyNum : bodyList)
//    for(size_t i = 0; i < posDofsPerBody; i++)
//      extendedCfg[bodyNum*dofsPerBody + i] += dir[i];
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
//    random_shuffle(robotList.begin(), robotList.end());
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
      std::cout << "m_activeRobots = " << this->m_activeRobots << std::endl;

    //Preserves any existing translations, as it will undo, rotate, and redo
    // any translation (and additional components due to not rotating about itself)
    extendedCfg.RotateFormationAboutLeader(robotList, rotMat, this->m_debug);
  }

  extendedCfg.NormalizeOrientation(robotList);

  //Check that the sample is in bounds and valid:
  if(!extendedCfg.InBounds(_boundary) ||
     !vc->IsValid(extendedCfg, callee, this->m_activeRobots)) {
//    _collision.push_back(extendedCfg - map->GetVertex(0));
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
