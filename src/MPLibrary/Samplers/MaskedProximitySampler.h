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
    typedef typename std::vector<unsigned int> Subassembly;

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

    // m_rotationFactor determines the magnitude of rotation that is allowed to be
    // applied to the sample, wrt the starting cfg.
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
//  auto graph = this->GetRoadmap()->GetGraph();

  for(size_t i = 0; i < _numNodes; ++i) {
    vector<CfgType> result;
    vector<CfgType> collision;
    //Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      this->GetStatClass()->IncNodesAttempted(this->GetNameAndLabel());

      // Just use the start cfg as the cfg to get a proximity sample from:
      if(this->Sampler(this->m_startCfg, _boundary, result, collision))
        break;
    }

    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(),
        result.size());
    _result = copy(result.begin(), result.end(), _result);
    if(this->m_debug)
      for(const CfgType& cfg : result) {
        std::cout << "MaskedProximitySampler::Sample result = "
                  << cfg.PrettyPrint() << std::endl;
        this->VerifyCfg(cfg);
      }
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

  ///@TODO: For now I'm assuming that the bodies all have the same dofsPerBody,
  ///       this should be updated to be arbitrary in the future.
  const unsigned int numBodies = _cfg.GetMultiBody()->GetNumBodies();
  const unsigned int posDofsPerBody = _cfg.PosDOF();
  const unsigned int oriDofsPerBody = _cfg.OriDOF();
  const unsigned int dofsPerBody = posDofsPerBody + oriDofsPerBody;
  const bool isRotational = oriDofsPerBody > 0;

  // A little bit of sanity checking:
  if((dofsPerBody * numBodies) != _cfg.DOF() ||
     _cfg.DOF() % (posDofsPerBody + oriDofsPerBody) != 0)
    throw RunTimeException(WHERE, "DOFs don't match up for multibody!");

  CfgType extendedCfg = _cfg; // The cfg to be in proximity of.

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

  Subassembly bodyList = this->m_bodyList;  // Copy for potential reordering.
  for(unsigned int bodyNum : bodyList)
    for(unsigned int i = 0; i < posDofsPerBody; i++)
      extendedCfg[bodyNum*dofsPerBody + i] += dir[i];

  /// Now the translation has been applied to all the bodies. Next, handle
  /// rotations appropriately:
  const double rotScale = 2. * m_rotationFactor; // Scale amount of rotation
  if (bodyList.size() == 1) { // Single body case
    for(unsigned int i = posDofsPerBody; i < dofsPerBody; i++)
      extendedCfg[bodyList[0]*dofsPerBody + i] += (DRand() - .5) * rotScale;
  }
  else if(isRotational) {
    // shuffle randomly, and then rotate about the body number in bodyList[0].
    random_shuffle(bodyList.begin(), bodyList.end());

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
      std::cout << "m_bodyList = " << this->m_bodyList << std::endl;

    //Preserves any existing translations, as it will undo, rotate, and redo
    // any translation (and additional components due to not rotating about itself)
    extendedCfg = RotateCfgAboutBody<MPTraits>(bodyList, extendedCfg, rotMat,
                                               dofsPerBody, this->m_debug);
  }

  this->m_lastSamplesLeaderBody = bodyList[0];
  extendedCfg.NormalizeOrientation();

  //Check that the sample is in bounds and valid:
  if(!extendedCfg.InBounds(_boundary) ||
     !vc->IsValid(extendedCfg, callee)) {
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
