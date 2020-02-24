#ifndef WORKSPACE_IMPORTANCE_SAMPLER_H_
#define WORKSPACE_IMPORTANCE_SAMPLER_H_

#include "SamplerMethod.h"

#include "MPLibrary/MPTools/TetGenDecomposition.h"
#include "Workspace/WorkspaceDecomposition.h"


////////////////////////////////////////////////////////////////////////////////
/// Workspace Importance Sampling uses a tetrahedralization to bias sampling
/// towards narrow passages of the workspace.
///
/// Workspace Importance Sampling defines an importance value over each
/// tetrahedron of a tetrahedralization. From the importance value, a number of
/// attempts is defined for a tetrahedron. At sampling time, a random
/// tetrahedron is selected (uniformly) and number of attempts samples are tried
/// to yield a successful configuration in the configuration space.
///
/// From:
///   Hanna Kurniawati and David Hsu, "Workspace Importance Sampling for
///   Probabilistic Roadmap Planners," In Proc. of the Int. Conf. on Intelligent
///   Robots and Systems (IROS), Sendai, Japan, pp. 1618-1623, Sept 2004.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class WorkspaceImportanceSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    WorkspaceImportanceSampler();
    WorkspaceImportanceSampler(XMLNode& _node);
    virtual ~WorkspaceImportanceSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    virtual void Initialize() override;

    ///@}

  protected:

    ///@name SamplerMethod Overrides
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        std::vector<CfgType>& _result, std::vector<CfgType>& _collision)
        override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Make sure the decomposition is available and compute tetrahedron
    /// importances.
    void InitDecomposition();

    /// Compute the height of a tetraherdon relative to a specified facet.
    double ComputeTetrahedronHeight(const WorkspaceRegion& _tetra,
        const WorkspaceRegion::Facet& _f) const;

    /// Compute the importance of a given tetrahedron.
    double ComputeTetrahedronImportance(const size_t _i) const;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel;              ///< Validity checker label.
    std::string m_decompositionLabel;   ///< The workspace decomposition label.

    std::vector<size_t> m_numAttempts;  ///< Number of attempts per tetrahedron

    /// The "eagerness in obtaining one milestone for each tetrahedron".
    double m_alpha{.2};

    bool m_initialized{false};    ///< Is the decomposition initialized?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
WorkspaceImportanceSampler<MPTraits>::
WorkspaceImportanceSampler() {
  this->SetName("WorkspaceImportanceSampler");
}


template <typename MPTraits>
WorkspaceImportanceSampler<MPTraits>::
WorkspaceImportanceSampler(XMLNode& _node) :
    SamplerMethod<MPTraits>(_node) {
  this->SetName("WorkspaceImportanceSampler");

  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");

  m_decompositionLabel = _node.Read("decompositionLabel", true, "",
      "The workspace decomposition to use.");

  m_alpha = _node.Read("alpha", false,
      m_alpha, numeric_limits<double>::epsilon(), 1.,
      "Eagerness in obtaining one milestone for each tetrahedron");
}

/*---------------------------- MPBaseObject Overrides ------------------------*/

template <typename MPTraits>
void
WorkspaceImportanceSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
}


template <typename MPTraits>
void
WorkspaceImportanceSampler<MPTraits>::
Initialize() {
  m_initialized = false; // Tell object to initialize on first use.
}

/*--------------------------- SamplerMethod Overrides ------------------------*/

template <typename MPTraits>
bool
WorkspaceImportanceSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    std::vector<CfgType>& _result, std::vector<CfgType>& _collision) {
  if(!m_initialized)
    InitDecomposition();

  const std::string callee(this->GetNameAndLabel() + "::SampleImpl()");
  Environment* const env = this->GetEnvironment();
  auto wd = this->GetMPTools()->GetDecomposition(m_decompositionLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);

  // Pick a tetrahedron with uniform random probability.
  const size_t t = LRand() % wd->GetNumRegions();
  const Boundary* const tetra = wd->GetRegion(t).GetBoundary();

  if(this->m_debug)
    std::cout << "Selected tetra " << t
              << "\n\tBoundary: " << *tetra
              << "\n\tVolume:   " << tetra->GetVolume()
              << "\n\tAttempts: " << m_numAttempts[t]
              << std::endl;

  for(size_t i = 0; i < m_numAttempts[t]; ++i) {
    // Generate a sample inside the tetrahedron.
    CfgType cfg(this->GetTask()->GetRobot());
    cfg.GetRandomCfg(env);

    // Set the positional DOFs to a random point in the tetra.
    const std::vector<double> p = tetra->GetRandomPoint();
    for(size_t i = 0; i < cfg.PosDOF() and i < p.size(); ++i)
      cfg[i] = p[i];

    // Check if the new configuration is valid.
    if(cfg.InBounds(env) and vc->IsValid(cfg, callee)) {
      _result.push_back(cfg);
      return true;
    }
    else
      _collision.push_back(cfg);
  }
  return false;
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
WorkspaceImportanceSampler<MPTraits>::
InitDecomposition() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::InitDecomposition");

  m_initialized = true;

  // Get the tetrahedralization.
  auto wd = this->GetMPTools()->GetDecomposition(m_decompositionLabel);

  // Compute tetrahedron importance values.
  const size_t numTetras = wd->GetNumRegions();
  double totalImportance = 0;
  std::vector<double> importances(numTetras);
  for(size_t i = 0; i < numTetras; ++i)
    totalImportance += importances[i] = ComputeTetrahedronImportance(i);

  // Compute tetrahedron attempts based on importance values.
  m_numAttempts.clear();
  m_numAttempts.reserve(numTetras);
  for(size_t i = 0; i < numTetras; ++i) {
    const size_t nt = log(1 - m_alpha) / log(1 - importances[i] / totalImportance);
    m_numAttempts.push_back(std::max<size_t>(nt, 1));
  }

  if(this->m_debug)
    std::cout << this->GetName() << "::InitDecomposition"
              << "\n\tNumber of tetrahedra: " << numTetras
              << "\n\tTotal importance:     " << totalImportance
              << "\n\tMax importance:       "
              << *std::max_element(importances.begin(), importances.end())
              << "\n\tMin importance:       "
              << *std::min_element(importances.begin(), importances.end())
              << "\n\tMax attempts:         "
              << *std::max_element(m_numAttempts.begin(), m_numAttempts.end())
              << "\n\tMin attempts:         "
              << *std::min_element(m_numAttempts.begin(), m_numAttempts.end())
              << std::endl;
  // This is handy for figuring out how to set alpha.
  //std::cin.ignore();
}


template <typename MPTraits>
double
WorkspaceImportanceSampler<MPTraits>::
ComputeTetrahedronHeight(const WorkspaceRegion& _tetra,
    const WorkspaceRegion::Facet& _f) const {
  double height = -1;

  // Loop over all points in the tetrahedron.
  for(size_t i = 0; i < 4; ++i) {
    const auto& p = _tetra.GetPoint(i);

    // Look for this point in the facet.
    bool pointOnFacet = false;
    for(size_t index = 0; index < 3; ++index) {
      if(p == _f.GetPoint(index)) {
        pointOnFacet = true;
        break;
      }
    }

    // If we didn't find p on this facet, then use p to find the height.
    if(!pointOnFacet) {
      height = (_f.GetPoint(0) - p) * _f.GetNormal();
      break;
    }
  }

  // Ensure this height makes sense.
  if(height <= 0)
    throw RunTimeException(WHERE) << "Can't have a tetrahedron with "
                                  << "non-positive height " << height << ".";

  return height;
}


template <typename MPTraits>
double
WorkspaceImportanceSampler<MPTraits>::
ComputeTetrahedronImportance(const size_t _i) const {
  // The importance of the tetrahedron is the average importance of its relevant
  // facets.
  // Any facets touching an obstacle are dubbed boundary facets. If there are
  // any boundary facets, then only they are relevant. Otherwise, all facets are
  // relevant.
  // The importance of each facet is given by the height of the tetrahedron
  // using that facet as the base.

  // Find number of neighbors.
  auto wd = this->GetMPTools()->GetDecomposition(m_decompositionLabel);
  auto iter = wd->find_vertex(_i);
  size_t numNeighbors = iter->size();
  if(numNeighbors > 4)
    throw RunTimeException(WHERE) << "Can't have more than four neighbors in a "
                                  << "tetrahedral decomposition graph.";

  // Initialize all facets as relevant to the importance calculation.
  const auto& tetra = iter->property();
  std::vector<const WorkspaceRegion::Facet*> importanceFacets;
  for(const auto& f : tetra.GetFacets())
    importanceFacets.push_back(&f);

  // If a tetrahedron has fewer than four adjacent neighbors, then it is on the
  // boundary. Remove any shared facets from the importance calculation.
  if(numNeighbors < 4) {
    for(auto edge = iter->begin(); edge != iter->end(); ++edge) {
      // Get the portal and its facets.
      const auto& portal = edge->property();
      auto portalFacets = portal.FindFacets();

      // Ensure we only have one facet in this portal as expected.
      if(portalFacets.size() != 1)
        throw RunTimeException(WHERE) << "Each portal must have exactly one "
                                      << "shared facet, but detected "
                                      << portalFacets.size()
                                      << " shared facets between adjacent "
                                      << "regions.";

      // Find the shared facet in the boundary facet list.
      auto fIter = find(importanceFacets.begin(), importanceFacets.end(),
          portalFacets.front());

      // Ensure we found a real facet.
      if(fIter == importanceFacets.end())
        throw RunTimeException(WHERE) << "The neighboring facets of a "
                                      << "tetrahedron must be present in its "
                                      << "facet list.";

      // Remove this facet from the set of boundary facets.
      importanceFacets.erase(fIter);
    }
  }

  // Compute importance from relevant facets.
  double importance = 0;
  for(const auto& f : importanceFacets)
    importance += ComputeTetrahedronHeight(tetra, *f);
  return importance /= importanceFacets.size();
}

/*----------------------------------------------------------------------------*/

#endif
