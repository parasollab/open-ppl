#ifndef WORKSPACE_IMPORTANCE_SAMPLER_H_
#define WORKSPACE_IMPORTANCE_SAMPLER_H_

#include "SamplerMethod.h"

#include "Utilities/TetGenDecomposition.h"
#include "Workspace/WorkspaceDecomposition.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Workspace Importance Sampling uses a tetrahedralization to bias
///        sampling towards narrow passages of the workspace.
/// @tparam MPTraits Motion planning universe
///
/// Workspace Importance Sampling defines an importance value over each
/// tetrahedron of a tetrahedralization. From the importance value, a number of
/// attempts is defined for a tetrahedron. At sampling time, a random
/// tetrahedron is selected (uniformly) and number of attempts samples are tried
/// to yield a successful configuration in the configuration space.
///
/// From:
/// Hanna Kurniawati and David Hsu, "Workspace Importance Sampling for
/// Probabilistic Roadmap Planners," In Proc. of the Int. Conf. on Intelligent
/// Robots and Systems (IROS), Sendai, Japan, pp. 1618-1623, Sept 2004.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class WorkspaceImportanceSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///@name Construction
    ///@{

    WorkspaceImportanceSampler();
    WorkspaceImportanceSampler(MPProblemType* _problem, XMLNode& _node);

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}

  protected:

    ///@name SamplerMethod Overrides
    ///@{

    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute importance and num attempts for each tetrahedron.
    void Initialize();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute the height of a tetraherdon relative to a specified facet.
    double ComputeTetrahedronHeight(const WorkspaceRegion& _tetra,
        const WorkspaceRegion::Facet& _f) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute the importance of a given tetrahedron.
    double ComputeTetrahedronImportance(const size_t _i) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get a random point within the tetrahedron defined by four points.
    /// @param[in] _base The base of the tetrahedron in absolute coordinates.
    /// @param[in] _p1 Another point, relative to _base.
    /// @param[in] _p2 Another point, relative to _base.
    /// @param[in] _p3 Another point, relative to _base.
    /// @return A point inside the tetrahedron defined by {_base, _base + _p1,
    ///         _base + _p2, _base + _p3}.
    const Point3d RandomPointInTetrahedron(const Point3d& _base,
        const Point3d& _p1, const Point3d& _p2, const Point3d& _p3) const;

    ///@}
    ///@name Internal State
    ///@{

    string m_vcLabel{"pqp_solid"}; ///< Validity checker label.

    vector<size_t> m_numAttempts; ///< Number of attempts per tetrahedron

    double m_alpha{.2}; ///< The "eagerness in obtaining one milestone for
                        ///< each tetrahedron".

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template<class MPTraits>
WorkspaceImportanceSampler<MPTraits>::
WorkspaceImportanceSampler() {
  this->SetName("WorkspaceImportanceSampler");
}


template<class MPTraits>
WorkspaceImportanceSampler<MPTraits>::
WorkspaceImportanceSampler(MPProblemType* _problem, XMLNode& _node) :
    SamplerMethod<MPTraits>(_problem, _node) {
  this->SetName("WorkspaceImportanceSampler");
  m_vcLabel = _node.Read("vcLabel", false, m_vcLabel, "Validity Test Method");
  m_alpha = _node.Read("alpha", false,
      m_alpha, numeric_limits<double>::epsilon(), 1.,
      "Eagerness in obtaining one milestone for each tetrahedron");
}

/*---------------------------- MPBaseObject Overrides ------------------------*/

template<class MPTraits>
void
WorkspaceImportanceSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
}

/*--------------------------- SamplerMethod Overrides ------------------------*/

template<class MPTraits>
bool
WorkspaceImportanceSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  // Ensure auxiliary structures are initialized.
  static bool init = false;
  if(!init) {
    Initialize();
    init = true;
  }

  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  Environment* env = this->GetEnvironment();
  auto tetrahedralization = env->GetDecomposition();
  auto vc = this->GetValidityChecker(m_vcLabel);

  // Pick a tetrahedron with uniform random probability.
  size_t t = LRand() % tetrahedralization->GetNumRegions();
  const auto& tetra = tetrahedralization->GetRegion(t);

  // Get relative coordinates.
  const Vector3d& o = tetra.GetPoint(0);
  const Vector3d a = tetra.GetPoint(1) - o;
  const Vector3d b = tetra.GetPoint(2) - o;
  const Vector3d c = tetra.GetPoint(3) - o;

  // Try to generate a sample inside the tetrahedron.
  for(size_t i = 0; i < m_numAttempts[t]; ++i) {
    CfgType cfg;
    cfg.GetRandomCfg(this->GetEnvironment());

    const Point3d p = RandomPointInTetrahedron(o, a, b, c);
    for(size_t j = 0; j < cfg.PosDOF(); ++j)
      cfg[j] = p[j];

    // Check if the new configuration is valid.
    if(env->InBounds(cfg)) {
      if(vc->IsValid(cfg, callee)) {
        _result.push_back(cfg);
        return true;
      }
      else
        _collision.push_back(cfg);
    }
  }
  return false;
}

/*---------------------------------- Helpers ---------------------------------*/

template<class MPTraits>
void
WorkspaceImportanceSampler<MPTraits>::
Initialize() {
  if(this->m_debug)
    cout << "Initializing " << this->GetNameAndLabel() << "...\n";

  // Get the tetrahedralization from the environment.
  auto env = this->GetEnvironment();
  if(!env->GetDecomposition()) {
    if(this->m_debug)
      cout << "\tDecomposing environment...\n";
    env->Decompose(TetGenDecomposition(this->GetBaseFilename(), "pnQ"));
  }
  else if(this->m_debug)
    cout << "\tEnvironment is already decomposed.\n";
  auto tetrahedralization = env->GetDecomposition();

  if(this->m_debug)
    cout << "\tComputing importance values...\n";

  // Compute tetrahedron importance values.
  const size_t numTetras = tetrahedralization->GetNumRegions();
  double totalImportance = 0;
  vector<double> importances(numTetras);
  for(size_t i = 0; i < numTetras; ++i)
    totalImportance += importances[i] = ComputeTetrahedronImportance(i);

  if(this->m_debug)
    cout << "\t\tImportance range: ["
         << *min_element(importances.begin(), importances.end())
         << ":"
         << *max_element(importances.begin(), importances.end())
         << "]\n"
         << "\tComputing num attempts per tetraherdon...\n";

  // Compute tetrahedron attempts based on importance values.
  m_numAttempts.reserve(numTetras);
  for(size_t i = 0; i < numTetras; ++i) {
    size_t nt = log(1 - m_alpha) / log(1 - importances[i] / totalImportance);
    m_numAttempts.push_back(nt);
  }

  if(this->m_debug)
    cout << "\t\tNumber of attempts range: ["
         << *min_element(m_numAttempts.begin(), m_numAttempts.end())
         << ":"
         << *max_element(m_numAttempts.begin(), m_numAttempts.end())
         << "]" << endl;
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
    throw PMPLException("Tetrahedralization error", WHERE, "Can't have a "
        "tetrahedron with non-positive height " + to_string(height) + ".");

  // Limit the minimum height because very small values ruin the importance
  // calculations.
  height = max(height, 10 * this->GetEnvironment()->GetPositionRes());

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
  auto iter = this->GetEnvironment()->GetDecomposition()->find_vertex(_i);
  const size_t numNeighbors = iter->size();
  if(numNeighbors > 4)
    throw PMPLException("Tetrahedralization error", WHERE, "Can't have more than"
        " four neighbors in a tetrahedral region graph.");
  if(numNeighbors == 0)
    throw PMPLException("Tetrahedralization error", WHERE, "Can't have a tetrahe"
        "dron with no neighbors.");

  // Initialize all facets as relevant to the importance calculation.
  const auto& tetra = iter->property();
  vector<const WorkspaceRegion::Facet*> importanceFacets;
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
        throw PMPLException("Tetrahedralization error", WHERE, "Each portal "
            "must have exactly one shared facet, but detected " +
            to_string(portalFacets.size()) + " shared facets between adjacent "
            "regions.");

      // Find the shared facet in the boundary facet list.
      auto fIter = find(importanceFacets.begin(), importanceFacets.end(),
          portalFacets.front());

      // Ensure we found a real facet.
      if(fIter == importanceFacets.end())
        throw PMPLException("Tetrahedralization error", WHERE, "The neighboring "
            "facets of a tetrahedron must be present in its facet list.");

      // Remove this facet from the set of boundary facets.
      importanceFacets.erase(fIter);
    }
  }

  // Compute importance from relevant facets.
  double importance = 0;
  for(const auto& f : importanceFacets)
    importance += ComputeTetrahedronHeight(tetra, *f);
  return importance /= numNeighbors;
}


template <typename MPTraits>
const Point3d
WorkspaceImportanceSampler<MPTraits>::
RandomPointInTetrahedron(const Point3d& _base, const Point3d& _p1,
    const Point3d& _p2, const Point3d& _p3) const {
  // From:
  //   C. Rocchini and P. Cignoni, "Generating Random Points in a Tetrahedron,"
  //       Journal of Graphics Tools, 2001.

  // Pick a point in unit cube.
  double s = DRand();
  double t = DRand();
  double u = DRand();

  // Cut cube in half with plane s + t = 1.
  if(s + t > 1) {
    s = 1 - s;
    t = 1 - t;
  }

  // Cut cube with planes t + u = 1 and s + t + u = 1.
  if(s + t + u > 1) {
    if(t + u > 1) {
      double ttmp = 1 - u;
      double utmp = 1 - s - t;
      swap(t, ttmp);
      swap(u, utmp);
    }
    else {
      double stmp = 1 - t - u;
      double utmp = s + t + u - 1;
      swap(s, stmp);
      swap(u, utmp);
    }
  }

  // Determine random point in tetrahedron.
  return _base + _p1 * s + _p2 * t + _p3 * u;
}

/*----------------------------------------------------------------------------*/

#endif
