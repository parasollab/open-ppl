#ifndef PMPL_MATING_NORMAL_SAMPLER_H_
#define PMPL_MATING_NORMAL_SAMPLER_H_

#include "MaskedSamplerMethodGroup.h"


////////////////////////////////////////////////////////////////////////////////
/// Most basic version of the Disassembly Samplers. A Mating sampler
/// simply translates along a straight line. This one allows for bodies
/// to be set, in order to handle composite C-Spaces. This version of the
/// mating sampler uses all face normals present in the environment as
/// candidates for mating directions. Then they are filtered out if their
/// x/y/z components are too similar to existing chosen directions.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class MatingNormalSamplerGroup : public MaskedSamplerMethodGroup<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename SamplerMethod<MPTraits>::GroupOutputIterator;

    ///@}
    ///@name Construction
    ///@{

    MatingNormalSamplerGroup();

    MatingNormalSamplerGroup(XMLNode& _node);

    virtual ~MatingNormalSamplerGroup() = default;

    ///@}
    ///@name SamplerMethod Overrides
    ///@{

    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, GroupOutputIterator _result,
        GroupOutputIterator _collision) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<mathtool::Vector3d> m_normals; ///< List with computed samples.
    GroupCfgType m_startCfgForSamples; ///< To determine whether m_normals is current.

    double m_duplicateThreshold; ///< Dot product threshold to remove duplicates.
    double m_dist; ///< Length of each vector.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MatingNormalSamplerGroup<MPTraits>::
MatingNormalSamplerGroup() {
  this->SetName("MatingNormalSamplerGroup");
}


template <typename MPTraits>
MatingNormalSamplerGroup<MPTraits>::
MatingNormalSamplerGroup(XMLNode& _node)
    : MaskedSamplerMethodGroup<MPTraits>(_node) {
  this->SetName("MatingNormalSamplerGroup");

  m_duplicateThreshold = _node.Read("duplicateThreshold", false, 0.01, 0.0, 10.0,
      "Threshold to remove duplictates from the normal list");
  m_dist = _node.Read("dist", false, 1., 0.,
      std::numeric_limits<double>::max(), "Distance of the samples");
}

/*-------------------------- SamplerMethod Overrides -------------------------*/

template <typename MPTraits>
void
MatingNormalSamplerGroup<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
       GroupOutputIterator _result, GroupOutputIterator _collision) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Sample()" << std::endl;

  if(this->m_startCfg != m_startCfgForSamples) {
    if(this->m_debug)
      std::cout << "New startCfg detected, recomputing samples." << std::endl;
    m_normals = std::vector<mathtool::Vector3d>();
  }

  // compute the samples only once, checks if samples are empty
  if(m_normals.empty()) {
    const size_t numRobots = this->m_startCfg.GetNumRobots();
    const size_t posDofsPerBody = this->m_startCfg.PosDOF();

    std::vector<Vector3d> normals;

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
      for(const GMSPolygon& polygon : polygons) {
        Vector3d normal = polygon.GetNormal();
        if(posDofsPerBody == 2)
          normal[2] = 0.0; // If 2D, zero out the z-component

        normals.push_back(normal);
      }
    }

    for(Vector3d& normal : normals)
      normal.selfNormalize(); // normalize normals
    if(this->m_debug)
      std::cout << "Normalized the normals" << std::endl;

    // remove duplicates from normal list, start with sorting to make it faster
    std::sort(begin(normals), end(normals));
    if(this->m_debug)
      std::cout << "Sorted the normals" << std::endl;

    auto i = begin(normals);
    while(i != end(normals)-1) {
      auto j = i + 1;
      while(j != end(normals) - 1) {
        if(fabs((*i)[0] - (*j)[0]) > m_duplicateThreshold)
          break;

        if(fabs((*i)[1] - (*j)[1]) < m_duplicateThreshold &&
            fabs((*i)[2] - (*j)[2]) < m_duplicateThreshold)
          j = normals.erase(j);
        else
          ++j;
      }
      ++i;
    } // TODO: This doesn't compare the last two normals and should be fixed.

    // multiply the normals to get the right magnitude
    for(Vector3d& normal : normals)
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
  for(size_t i = 0; i < m_normals.size(); ++i) {
    GroupCfgType cfg = m_startCfgForSamples;
    cfg.AddDofsForRobots(m_normals[i], this->m_activeRobots);
    _result = cfg;
  }
}

/*----------------------------------------------------------------------------*/

#endif
