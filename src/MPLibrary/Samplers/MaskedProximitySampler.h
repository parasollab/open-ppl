#ifndef MASKED_PROXIMITY_SAMPLER_H_
#define MASKED_PROXIMITY_SAMPLER_H_

#include <iostream>

#include "MaskedSamplerMethod.h"
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
class MaskedProximitySampler : public MaskedSamplerMethod<MPTraits> {

  public:
    typedef typename MPTraits::MPLibrary::DistanceMetricPointer  DistanceMetricPointer;
    typedef typename MPTraits::MPLibrary::ValidityCheckerPointer ValidityCheckerPointer;

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename std::vector<CfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<CfgType>> OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    MaskedProximitySampler();
    MaskedProximitySampler(XMLNode& _node);
    virtual ~MaskedProximitySampler() = default;

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
    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    ///@}

    ///@name Parameters
    ///@{

    double m_maxDist{1.0}; /// Maximum distance to move away from a cfg.
    double m_rotationFactor{1.0};

    string m_vcLabel;
    string m_dmLabel;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MaskedProximitySampler<MPTraits>::
MaskedProximitySampler(XMLNode& _node) : MaskedSamplerMethod<MPTraits>(_node) {
  this->SetName("MaskedProximitySampler");
  m_maxDist = _node.Read("maxDist", false, m_maxDist, 0.,
                          numeric_limits<double>::max(),
                         "The maximum distance to move from a cfg.");

  m_rotationFactor = _node.Read("rotationFactor", false, m_rotationFactor,
                         0., 1.0, "Factor to determine max sampled rotation");

  m_vcLabel = _node.Read("vcLabel", true, "",
                         "The validity checker to use for sampling");
  m_dmLabel = _node.Read("dmLabel", true, "",
                         "The distance metric to use for sampling");
}

template <typename MPTraits>
MaskedProximitySampler<MPTraits>::
MaskedProximitySampler() : MaskedSamplerMethod<MPTraits>() {
  this->SetName("MaskedProximitySampler");
}


/*---------------------------- Sampler Interface -----------------------------*/

template <typename MPTraits>
void
MaskedProximitySampler<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
       OutputIterator _result) {
  auto graph = this->GetRoadmap()->GetGraph();

  for(size_t i = 0; i < _numNodes; ++i) {
    //Get a random vertex from the graph and hand it to Sampler()
    size_t cfgVid = LRand() % graph->get_num_vertices();
    CfgType cfg = graph->GetVertex(cfgVid);
    vector<CfgType> result;
    vector<CfgType> collision;
    //Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      this->GetStatClass()->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(cfg, _boundary, result, collision))
        break;
    }

    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(),
        result.size());
    _result = copy(result.begin(), result.end(), _result);
    if(this->m_debug)
      for(const CfgType& cfg : result)
        std::cout << "MaskedProximitySampler::Sample result = "
                  << cfg.PrettyPrint() << std::endl;
  }
  if(this->m_debug)
    std::cout << std::endl; // To show where an entire Sample call ends.
}


template <typename MPTraits>
bool
MaskedProximitySampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) {
  //The input cfg is a cfg from the roadmap. Extend up to m_maxDist away
  // from _cfg and return (extendedCfg - roadmap[0]).
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
  auto graph = this->GetRoadmap()->GetGraph();

  const string callee = this->GetNameAndLabel() + "::Sampler()";
  const unsigned int dofsPerBody = 6;

  CfgType extendedCfg = _cfg;//The cfg to 'extend' from.

  //Get random direction of m_maxDist length, scale it down using the range [0,1),
  // then add the new direction to the starting cfg.
  mathtool::Vector3d dir(DRand()-.5, DRand()-.5, DRand()-.5);
  dir.selfNormalize();
  dir *= m_maxDist;

  for(unsigned int bodyNum : this->m_bodyList)
    for(unsigned int i = 0; i < 3; i++)
      extendedCfg[bodyNum*dofsPerBody + i] += dir[i];

  //A constant used throughout the function.
  // m_rotationFactor might go away or change since this just limits what the
  // angle can get set to, not how far it will rotate, which is bad.
  const double rotScale = 2. * m_rotationFactor;

  if(!this->m_onlyPositionalDOFs) {
    if (this->m_bodyList.size() == 1) {
      for(unsigned int i = 3; i < 6; i++) { // compute first rotation
        extendedCfg[this->m_bodyList[0]*dofsPerBody + i] = (DRand() - .5) * rotScale;
      }
    }
    else { // multi part subassembly
      std::vector<unsigned int> bodyList = this->m_bodyList;//Copy it for reordering

      // shuffle randomly, and then rotate about the body number in bodyList[0].
      ///@TODO uncomment this once working for true random sampling.
//      random_shuffle(bodyList.begin(), bodyList.end());

      //Generate the random rotation, taking into account rotation bounds:
      const double piScaled = PI * rotScale;
      mathtool::EulerAngle deltaRot(piScaled * (DRand() - .5),
                                    piScaled * (DRand() - .5),
                                    piScaled * (DRand() - .5));

      mathtool::Orientation rotMat(deltaRot);

      if(this->m_debug) {
        std::cout << "Rotating the following bodies about body " << bodyList[0]
                  << ": {";
        for(size_t i = 1; i < bodyList.size()-1; ++i)
          std::cout << bodyList[i] << ", ";
        std::cout << bodyList[bodyList.size()-1] << "}" << std::endl;
      }
      //Preserves any existing translations, as it will undo, rotate, and redo
      // any translation (and additional components due to not rotating about itself)
      extendedCfg = RotateCfgAboutBody<MPTraits>(bodyList, extendedCfg, rotMat);
      this->m_lastRotAboutBody = bodyList[0];
    }
  }

  extendedCfg.NormalizeOrientation();

  //Check that the sample is in bounds and valid:
  if(!extendedCfg.InBounds(_boundary)
      || !vc->IsValid(extendedCfg, callee)) {
    _collision.push_back(extendedCfg - graph->GetVertex(0));
    return false;
  }

  //The sample given back must account for startCfg offset, which is built into
  // the cfg we started from, so no adjustment is needed here.
  _result.push_back(extendedCfg);
  return true;
}

/*----------------------------------------------------------------------------*/

#endif
