#ifndef MATING_NORMAL_SAMPLER_H_
#define MATING_NORMAL_SAMPLER_H_

#include "MaskedSamplerMethodGroup.h"

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
class MatingNormalSamplerGroup : public MaskedSamplerMethodGroup<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType CfgType;
    typedef typename std::vector<CfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<CfgType> >
                                                                 OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    MatingNormalSamplerGroup(const double dist = 10,
                        const double _duplicateThreshold = 0.001);
    MatingNormalSamplerGroup(XMLNode& _node);
    virtual ~MatingNormalSamplerGroup() = default;

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

    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
            const Boundary* const _boundary, OutputIterator _result) override;

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<mathtool::Vector3d> m_normals; ///< List with computed samples.
    CfgType m_startCfgForSamples; ///< To determine whether m_normals is current.

    double m_duplicateThreshold; ///< Dot product threshold to remove duplicates.
    double m_dist; ///< Length of each vector.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MatingNormalSamplerGroup<MPTraits>::
MatingNormalSamplerGroup(const double dist, const double _duplicateThreshold) :
    MaskedSamplerMethodGroup<MPTraits>(),
    m_duplicateThreshold(_duplicateThreshold), m_dist(dist) {
  this->SetName("MatingNormalSamplerGroup");
}


template <typename MPTraits>
MatingNormalSamplerGroup<MPTraits>::
MatingNormalSamplerGroup(XMLNode& _node) : MaskedSamplerMethodGroup<MPTraits>(_node) {
  this->SetName("MatingNormalSamplerGroup");
  m_duplicateThreshold = _node.Read("duplicateThreshold", false, 0.01, 0.0, 10.0,
        "Threshold to remove duplictates from the normal list");
  m_dist = _node.Read("dist", false, 1., 0.,
        numeric_limits<double>::max(), "Distance of the samples");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MatingNormalSamplerGroup<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
}

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
void
MatingNormalSamplerGroup<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
       OutputIterator _result) {
  std::vector<CfgType> collision;
  Sample(_numNodes, _maxAttempts, _boundary, _result, back_inserter(collision));
}

template <typename MPTraits>
void
MatingNormalSamplerGroup<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
       OutputIterator _result, OutputIterator _collision) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Sample()" << std::endl;

  if(this->m_startCfg != m_startCfgForSamples) {
    if(this->m_debug)
      std::cout << "New startCfg detected, recomputing samples." << std::endl;
    m_normals = std::vector<mathtool::Vector3d>();
  }

  // compute the samples only once, checks if samples are empty
  if (m_normals.empty()) {
    const size_t numRobots = this->m_startCfg.GetNumRobots();
    const size_t posDofsPerBody = this->m_startCfg.PosDOF();

    std::vector<Vector3d> normals;
    normals.reserve(60000); // Arbitrary number.

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

    for(size_t i = 0; i < numRobots; ++i) {
      MultiBody* const multiBody = this->m_startCfg.GetRobot(i)->GetMultiBody();
      if(multiBody->GetNumBodies() > 1)
        throw RunTimeException(WHERE, "More than one body is in individual "
                      "robot's multibody! For now this is not supported here!");
      const size_t body = 0; /// TODO add loop here to support multiple bodies.
      const std::vector<GMSPolygon>& polygons =
                        multiBody->GetBody(body)->GetPolyhedron().m_polygonList;
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
    for (Vector3d& normal : normals)
      normal *= m_dist;

    if(this->m_debug)
      std::cout << this->GetNameAndLabel() << "There were " << normals.size()
                << " normal mating vectors found." << std::endl;
    this->GetStatClass()->IncNodesGenerated(
                                       this->GetNameAndLabel(), normals.size());
    //The startCfg can change (like if we use subassemblies) so we need to keep
    // track of which cfg the cached samples are valid for.
    m_startCfgForSamples = this->m_startCfg;

    m_normals = normals;
  }

  // Add in the dofs for each mating vector
  for (size_t i = 0; i < m_normals.size(); ++i) {
    CfgType cfg = m_startCfgForSamples;
    cfg.AddDofsForRobots(m_normals[i], this->m_activeRobots);
    _result = cfg;
  }
}

//template <typename MPTraits>
//std::vector<typename MPTraits::GroupCfg>
//MatingNormalSamplerGroup<MPTraits>::
//MakeMatingVectors(const GroupCfg& _startCfg,
//          const std::vector<Vector3d>& _normals, const std::vector<size_t>& ) {
//
//  // Create a vector of group cfgs set to the _startCfg.
//  std::vector<GroupCfg> cfgs(_normals.size(), _startCfg);
//  for (size_t i = 0; i < _normals.size(); ++i)
//    cfgs[i].AddDofsForBodies(_normals[i], this->m_activeRobots);
//}

template <typename MPTraits>
bool
MatingNormalSamplerGroup<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) {

  Environment* env = this->GetEnvironment();

  // Note: this does not do a validity check.
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
