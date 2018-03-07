#ifndef MATING_NORMAL_SAMPLER_H_
#define MATING_NORMAL_SAMPLER_H_

#include "MaskedSamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Most basic version of the Disassembly Samplers. A Mating sampler
///        simply translates along a straight line. This one allows for bodies
///        to be set, in order to handle composite C-Spaces. This version of the
///        mating sampler uses all face normals present in the environment as
///        candidates for mating directions. Then they are filtered out if their
///        x/y/z components are too similar to existing chosen directions.
////////////////////////////////////////////////////////////////////////////////

template <class MPTraits>
class MatingNormalSampler : public MaskedSamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename std::vector<CfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<CfgType> >
                                                                 OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    MatingNormalSampler(const double dist = 10,
                        const double _duplicateThreshold = 0.001);
    MatingNormalSampler(XMLNode& _node);
    virtual ~MatingNormalSampler() = default;

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
        vector<CfgType>& _result, vector<CfgType>& _collision) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<CfgType> m_samples; ///< list with all computed samples
    CfgType m_startCfgForSamples;

    double m_duplicateThreshold; ///< threshold to remove duplicates
    double m_dist;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MatingNormalSampler<MPTraits>::
MatingNormalSampler(const double dist, const double _duplicateThreshold) :
    MaskedSamplerMethod<MPTraits>(),
    m_duplicateThreshold(_duplicateThreshold), m_dist(dist) {
  this->SetName("MatingNormalSampler");
}


template <typename MPTraits>
MatingNormalSampler<MPTraits>::
MatingNormalSampler(XMLNode& _node) : MaskedSamplerMethod<MPTraits>(_node) {
  this->SetName("MatingNormalSampler");
  m_duplicateThreshold = _node.Read("duplicateThreshold", false, 0.01, 0.0, 10.0,
        "Threshold to remove duplictates from the normal list");
  m_dist = _node.Read("dist", false, 1., 0.,
        numeric_limits<double>::max(), "Distance of the samples");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MatingNormalSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
}

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
void
MatingNormalSampler<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
       OutputIterator _result, OutputIterator _collision) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Sample()" << std::endl;

  if(this->m_startCfg != m_startCfgForSamples) {
    if(this->m_debug)
      std::cout << "New startCfg detected, recomputing samples." << std::endl;
    m_samples = std::vector<CfgType>();
  }

  ///@TODO move this sanity checking part to DisassemblyMethod::Initialize()
  MultiBody* const mb = this->GetTask()->GetRobot()->GetMultiBody();
  const unsigned int numBodies = mb->GetNumBodies();
  const unsigned int posDofsPerBody = mb->PosDOF();
  const unsigned int oriDofsPerBody = mb->OrientationDOF();
  const unsigned int dofsPerBody = posDofsPerBody + oriDofsPerBody;

  // A little bit of sanity checking:
  if((dofsPerBody * numBodies) != mb->DOF() ||
     (posDofsPerBody + oriDofsPerBody) != dofsPerBody)
    throw RunTimeException(WHERE, "DOFs don't match up for multibody!");

  // compute the samples only once, checks if samples are empty
  if (m_samples.empty()) {
    // compute the distance for one sample to separate from the robot
    auto robot = this->GetTask()->GetRobot();
    auto multiBody = robot->GetMultiBody();

    std::vector<CfgType> result;
    std::vector<CfgType> collision;

    std::vector<Vector3d> normals;
    normals.reserve(60000);

    //Push back the standard x/y/z dirs to ensure they are included (and not
    // something that's just similar but skewed).
    const Vector3d xDir(1,0,0);
    const Vector3d yDir(0,1,0);

    normals.push_back(xDir);
    normals.push_back(-xDir);
    normals.push_back(yDir);
    normals.push_back(-yDir);

    if(posDofsPerBody == 3) {
      const Vector3d zDir(0,0,1);
      normals.push_back(zDir);
      normals.push_back(-zDir);
    }


    ///@TODO: This had (numBodies - 1) which seemed wrong (but has been
    ///       working fine...). Check this!
    for(size_t i = 0; i < numBodies; ++i) {
      const std::vector<GMSPolygon>& polygons =
                           multiBody->GetBody(i)->GetPolyhedron().m_polygonList;
      for (const GMSPolygon& polygon : polygons) {
        Vector3d normal = polygon.GetNormal();
        if(posDofsPerBody == 2)
          normal[2] = 0.0; // If 2D, zero out the z-component

        normals.push_back(normal);
      }
    }

    for (Vector3d& normal : normals)
      normal.selfNormalize(); // normalize normals

    if(this->m_debug)
      std::cout << "Normalized the normals" << std::endl;
    // remove duplicates from normal list, start with sorting to make it faster
    struct {
      bool operator()(Vector3d a, Vector3d b) const
      {
        return a[0] < b[0];
      }
    } customLess;
    sort(begin(normals), end(normals), customLess);
    if(this->m_debug)
      std::cout << "Sorted the normals" << std::endl;

    auto i = begin(normals);
    while (i != end(normals)-1) {
      auto j = i + 1;
      while (j != end(normals) - 1) {
        if (fabs((*i)[0] - (*j)[0]) > m_duplicateThreshold)
          break;

        if (fabs((*i)[1] - (*j)[1]) < m_duplicateThreshold &&
            fabs((*i)[2] - (*j)[2]) < m_duplicateThreshold)
          j = normals.erase(j);
        else
          ++j;
      }
      ++i;
    } // TODO: This doesn't compare the last two normals and should be fixed.

    // multiply the normals to get the right magnitude
    for (auto &normal : normals)
      normal *= m_dist;

    if(this->m_debug)
      std::cout << "Final scaled normals = " << normals << std::endl;

    // check sample for collision
    for (auto &normal : normals) {
      CfgType cfg = this->m_startCfg; //Start it at the start cfg, then extend.
      for(size_t i = 0; i < robot->GetMultiBody()->GetNumBodies(); i++)
        for(size_t j = 0; j < posDofsPerBody; j++)
          cfg[(i*dofsPerBody) + j] += normal[j]; //Add in dof component, don't set it.

      Sampler(cfg, _boundary, result, collision);
    }

    if(this->m_debug)
      std::cout << this->GetNameAndLabel() << "There were " << result.size()
                << " normal mating vectors found." << std::endl;
    this->GetStatClass()->IncNodesGenerated(
                                        this->GetNameAndLabel(), result.size());
    //The startCfg can change (like if we use subassemblies) so we need to keep
    // track of which cfg the cached samples are valid for.
    m_startCfgForSamples = this->m_startCfg;
    m_samples = result;
    this->MaskCfgs(result);
    _result = copy(result.begin(), result.end(), _result);
    _collision = copy(collision.begin(), collision.end(), _collision);
  }
  else {
    vector<CfgType> result = m_samples;
    this->MaskCfgs(result);
    _result = copy(result.begin(), result.end(), _result);
  }
}

template <typename MPTraits>
bool
MatingNormalSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

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
