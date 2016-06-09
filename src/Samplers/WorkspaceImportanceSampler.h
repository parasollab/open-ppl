#ifndef WORKSPACE_IMPORTANCE_SAMPLER_H_
#define WORKSPACE_IMPORTANCE_SAMPLER_H_

#include "SamplerMethod.h"

#include "Utilities/TetGenDecomposition.h"

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
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    WorkspaceImportanceSampler(string _vcLabel = "", double _alpha = 0.2);
    WorkspaceImportanceSampler(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;

  protected:
    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

  private:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Tetrahedralize environment, compute importance, and num attempts
    ///        for each tetrahedron.
    void InitTetrahedralization();

    string m_vcLabel; ///< Validity checker label

    TetGenDecomposition* m_tetrahedralization; ///< TetGen tetrahedralization
    vector<tuple<Point3d, Point3d, Point3d, Point3d>>
      m_tetras; ///< Tetrahedrons
    vector<size_t> m_numAttempts; ///< Number of attempts per tetrahedron

    double m_alpha; ///< alpha is the "eagerness in obtaining one milestone for
                    ///< each tetrahedron".
};

template<class MPTraits>
WorkspaceImportanceSampler<MPTraits>::
WorkspaceImportanceSampler(string _vcLabel, double _alpha) :
  m_vcLabel(_vcLabel),
  m_tetrahedralization(new TetGenDecomposition("pn")),
  m_alpha(_alpha) {
    this->SetName("WorkspaceImportanceSampler");
  }

template<class MPTraits>
WorkspaceImportanceSampler<MPTraits>::
WorkspaceImportanceSampler(MPProblemType* _problem, XMLNode& _node) :
  SamplerMethod<MPTraits>(_problem, _node),
  m_tetrahedralization(new TetGenDecomposition(_node)) {
    this->SetName("WorkspaceImportanceSampler");
    m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
    m_alpha = _node.Read("alpha", true,
        0.2, numeric_limits<double>::epsilon(), 1.,
        "Eagerness in obtaining one milestone for each tetrahedron");
  }

template<class MPTraits>
void
WorkspaceImportanceSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
}

template<class MPTraits>
bool
WorkspaceImportanceSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  static bool tetrahedralizationInit = false;
  if(!tetrahedralizationInit) {
    InitTetrahedralization();
    tetrahedralizationInit = true;
  }

  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  Environment* env = this->GetEnvironment();
  ValidityCheckerPointer vcm = this->GetValidityChecker(m_vcLabel);

  //pick tetrahedron uniformly
  size_t t = LRand() % m_tetras.size();

  size_t numAttempts = m_numAttempts[t];
  auto& tetra = m_tetras[t];

  Vector3d o = get<0>(tetra);
  Vector3d a = (get<1>(tetra) - o);
  Vector3d b = (get<2>(tetra) - o);
  Vector3d c = (get<3>(tetra) - o);

  //for each attempt
  //  pick biased sample
  //  validity check on it
  for(size_t i = 0; i < numAttempts; ++i) {
    CfgType cfg;
    cfg.GetRandomCfg(this->GetEnvironment());

    //bias cfg to random point in tetra
    //From:
    //
    //C. Rocchini and P. Cignoni, "Generating Random Points in a Tetrahedron,"
    //Journal of Graphics Tools, 2001.

    //pick point in unit cube
    double s = DRand();
    double t = DRand();
    double u = DRand();
    //cut cube in half with plane s + t = 1
    if(s + t > 1) {
      s = 1 - s;
      t = 1 - t;
    }
    //cut cube with planes t + u = 1 and s + t + u = 1
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
    //determine random point in tetrahedron
    Point3d p = o + a*s + b*t + c*u;

    for(size_t j = 0; j < cfg.PosDOF(); ++j)
      cfg[j] = p[j];

    //Validate by checking if Cfg is in bounds and "valid", e.g., collision-free
    if(env->InBounds(cfg)) {
      if(vcm->IsValid(cfg, callee)) {
        _result.push_back(cfg);
        return true;
      }
      else
        _collision.push_back(cfg);
    }
  }
  return false;
}

template<class MPTraits>
void
WorkspaceImportanceSampler<MPTraits>::
InitTetrahedralization() {
  m_tetrahedralization->Decompose(this->GetEnvironment(),
      this->GetBaseFilename());

  //determine tetrahedron importance values
  size_t numTetras = m_tetrahedralization->GetNumTetras();
  size_t numCorners = m_tetrahedralization->GetNumCorners();
  const double* const points = m_tetrahedralization->GetPoints();
  const int* const tetras = m_tetrahedralization->GetTetras();
  const int* const neighbors = m_tetrahedralization->GetNeighbors();

  m_tetras.reserve(numTetras);
  m_numAttempts.reserve(numTetras);

  double totalImportance = 0;
  vector<double> importances;
  importances.reserve(numTetras);

  for(size_t i = 0; i < numCorners*numTetras; i += numCorners) {
    //build tetrahedron
    Point3d a(&points[3*tetras[i + 0]]);
    Point3d b(&points[3*tetras[i + 1]]);
    Point3d c(&points[3*tetras[i + 2]]);
    Point3d d(&points[3*tetras[i + 3]]);
    m_tetras.emplace_back(a, b, c, d);

    //Determine importance:
    //h(t) = \Sigma_1^4{B_ih_i}/\Sigma_1^4{B_i} if any facet is on boundary
    //h(t) = \Sigma_1^4{h_i}/4 otherwise
    double importance =
      TriangleHeight(a, b, c) +
      TriangleHeight(a, b, d) +
      TriangleHeight(a, c, d) +
      TriangleHeight(b, c, d);
    size_t numNeighbors = 0;
    for(size_t j = 0; j < 4; ++j)
      numNeighbors += neighbors[4*i/numCorners + j] == -1 ? 0 : 1;
    //if a tetrahedron only has less than 4 adjacent neighbors at least one
    //facet is on the boundary
    if(numNeighbors < 4) {
      for(size_t j = 0; j < 4; ++j) {
        int n = neighbors[4*i/numCorners + j];
        if(n != -1) {
          Point3d o[4] = {
            &points[3*tetras[n*numCorners + 0]],
            &points[3*tetras[n*numCorners + 1]],
            &points[3*tetras[n*numCorners + 2]],
            &points[3*tetras[n*numCorners + 3]]
          }; //neighbor tetrahedron

          //find coinciding triangle
          vector<Point3d> tri;
          if(find(o, o+4, a) != o+4)
            tri.push_back(a);
          if(find(o, o+4, b) != o+4)
            tri.push_back(b);
          if(find(o, o+4, c) != o+4)
            tri.push_back(c);
          if(find(o, o+4, d) != o+4)
            tri.push_back(d);

          if(tri.size() != 3)
            throw RunTimeException(WHERE, "Error in finding neighbor triangle.");

          //as this triangle is not a boundary triangle subtract its height from
          //importance
          importance -= TriangleHeight(tri[0], tri[1], tri[2]);
        }
      }
      importance /= (4 - numNeighbors);
    }
    else
      importance /= 4;
    importances.push_back(importance);
    totalImportance += importance;
  }

  //determine tetrahedron attempts based on importance values
  //n_t = ln(1-alpha)/ln(1-h_t/h_total)
  for(size_t i = 0; i < numTetras; ++i) {
    size_t nt = log(1-m_alpha)/log(1-importances[i]/totalImportance);
    m_numAttempts.push_back(nt);
  }
}

#endif

