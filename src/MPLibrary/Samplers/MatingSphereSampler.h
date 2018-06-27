#ifndef MATING_SPHERE_SAMPLER_H_
#define MATING_SPHERE_SAMPLER_H_

#include "MaskedSamplerMethodGroup.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Most basic version of the Disassembly Samplers. A Mating sampler
///        simply translates along a straight line. This one allows for bodies
///        to be set, in order to handle composite C-Spaces. This version of the
///        mating sampler randomly generates points on a unit sphere as dirs.
///
/// TODO: This class is not done (and also unused) but is unique and should be
///       updated to group behaviors if needed!
////////////////////////////////////////////////////////////////////////////////

template <class MPTraits>
class MatingSphereSampler : public MaskedSamplerMethodGroup<MPTraits> {

    //TODO: This class still needs updating to Groups!

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename std::vector<CfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<CfgType>> OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    MatingSphereSampler(const double dist = 10);
    MatingSphereSampler(XMLNode& _node);
    virtual ~MatingSphereSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    /// Try to sample a set number of new configurations from a given boundary.
      /// @param[in] _numNodes The number of samples desired.
      /// @param[in] _maxAttempts The maximum number of attempts for each sample.
      /// @param[in] _boundary The boundary to sample from.
      /// @param[out] _result An iterator to storage for the new configurations.
      /// @param[out] _collision An (optional) iterator to storage for failed
      ///                        attempts.
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
            const Boundary* const _boundary, OutputIterator _result,
            OutputIterator _collision) override;

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        std::vector<CfgType>& _result, std::vector<CfgType>& _collision) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<CfgType> m_samples; ///< list with all computed samples
    size_t m_numNodes = 0;     ///< number of samples of the last computation
    const double m_pi = M_PI;
    const double m_twoPi = M_PI * 2;
    double m_dist;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MatingSphereSampler<MPTraits>::
MatingSphereSampler(const double dist) : MaskedSamplerMethod<MPTraits>(), m_dist(dist) {
  this->SetName("MatingSphereSampler");
}


template <typename MPTraits>
MatingSphereSampler<MPTraits>::
MatingSphereSampler(XMLNode& _node) : MaskedSamplerMethod<MPTraits>(_node) {
  this->SetName("MatingSphereSampler");
  m_dist = _node.Read("dist", false, 1., 0.,
           numeric_limits<double>::max(), "Distance of the samples");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MatingSphereSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
}

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
void
MatingSphereSampler<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
       OutputIterator _result, OutputIterator _collision) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Sample()" << endl;

  ///@TODO: For now I'm assuming that the bodies all have the same dofsPerBody,
  ///       this should be updated to be arbitrary in the future.
  MultiBody* const mb = this->GetTask()->GetRobot()->GetMultiBody();
  const unsigned int numBodies = mb->GetNumBodies();
  const unsigned int dofsPerBody = mb->DOF() / numBodies;
  const unsigned int posDofsPerBody = mb->PosDOF();
  const unsigned int oriDofsPerBody = mb->OrientationDOF();

  // A little bit of sanity checking:
  if((dofsPerBody * numBodies) != mb->DOF() ||
     (posDofsPerBody + oriDofsPerBody) != dofsPerBody)
    throw RunTimeException(WHERE, "DOFs don't match up for multibody!");

  // compute the samples only once, if the number of nodes keeps the same
  if (_numNodes != m_numNodes || m_samples.empty()) {
    m_numNodes = _numNodes;
    // compute the distance for one sample to separate from the robot
    auto robot = this->GetMPProblem()->GetRobot(0);

    // compute increment size from _numNodes
    double incr = sqrt((m_pi * m_twoPi) / _numNodes);

    std::vector<CfgType> result;
    std::vector<CfgType> collision;
    // iterate through phi and theta to create the samples
    for (double theta = 0; theta < m_twoPi; theta += incr) {
      for (double phi = 0; phi < m_pi; phi += incr) {
        // compute sphere surface normal
        Vector3d vec;
        vec[0] = cos(theta) * sin(phi);
        vec[1] = sin(theta) * sin(phi);
        if(posDofsPerBody == 3)
          vec[2] = cos(phi); // Only need this if Volumetric
        // normalize and multiply with distance r
        vec.selfNormalize();
        vec *= m_dist;
        CfgType cfg = this->m_startCfg; //Start it at the start cfg, then extend
        for(size_t i = 0; i < robot->GetMultiBody()->GetNumBodies(); i++) {
          for(size_t j = 0; j < posDofsPerBody; j++) {
            cfg[(i*dofsPerBody) + j] += vec[j]; //Add in dof component, not set.
          }
        }
        // check sample for collision/OOB
        Sampler(cfg, _boundary, result, collision);
      }
    }
    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(), result.size());

    m_samples = result;
    this->MaskCfgs(result);
    _result = copy(result.begin(), result.end(), _result);
    _collision = copy(collision.begin(), collision.end(), _collision);
  }
  else {
    std::vector<CfgType> result = m_samples;
    this->MaskCfgs(result);
    _result = copy(result.begin(), result.end(), _result);
  }


}

template <typename MPTraits>
bool
MatingSphereSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    std::vector<CfgType>& _result, std::vector<CfgType>& _collision) {

  Environment* env = this->GetEnvironment();

  if(_cfg.InBounds(env->GetBoundary())) {
    _result.push_back(_cfg);
    return true;
  }
  else {
    _collision.push_back(_cfg);
    return false;
  }
}

/*----------------------------------------------------------------------------*/

#endif
