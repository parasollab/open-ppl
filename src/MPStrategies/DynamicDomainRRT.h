#ifndef DYNAMIC_DOMAIN_RRT_H_
#define DYNAMIC_DOMAIN_RRT_H_

#include "BasicRRTStrategy.h"

#include "Environment/BoundingBox.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Dynamic Domain RRT adapts the sampling space of RRTs to better
///        approximate boundary information
/// @tparam MPTraits Motion planning universe
///
/// Dynamic Domain RRT considers a bounding sphere around each configuration.
/// During RRT sampling only nodes within some \f$q_{near}\f$'s radius. If
/// extension is unsuccessful then \f$q_{near}\f$'s radius is set to \f$R\f$. If
/// successful then \f$q_{new}\f$'s radius is set to \f$\inf\f$.
///
/// Yershova, Anna, Sim, Thierry, and Lavalle, Steven M., "Dynamic-Domain RRTs:
/// Efficient Exploration by Controlling the Sampling Domain," Proc. of the Int.
/// Conf. on Robotics and Automation (ICRA), pp. 3856-3861, Barcelona, Spain,
/// Apr. 2005.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class DynamicDomainRRT : public BasicRRTStrategy<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;

    //Non-XML constructor sets all private variables
    DynamicDomainRRT(string _dm="euclidean",
        string _nf="bfnf", string _vc="cd1", string _nc="kClosest",
        string _gt="UNDIRECTED_TREE", string _extenderLabel="BERO",
        vector<string> _evaluators=vector<string>(),
        double _minDist=0.01, double _growthFocus=0.05,
        bool _evaluateGoal=true, const CfgType& _start=CfgType(),
        const CfgType& _goal=CfgType(), size_t _numRoots=1,
        size_t _numDirections=1, size_t _maxTrial = 3, bool _growGoals=false,
        double _r = 10);

    DynamicDomainRRT(MPProblemType* _problem, XMLNode& _node);

    virtual ~DynamicDomainRRT();

    virtual void Initialize();
    virtual void Print(ostream& _os) const;

  protected:
    ////////////////////////////////////////////////////////////////////////////
    /// @return String for accessing Cfg::GetStat for radius value
    static constexpr string RLabel() {return "DDRRT::R";}

    virtual CfgType SelectDirection();
    virtual VID ExpandTree(CfgType& _dir);

    double m_r; ///< R value to set radius on failures to. R is a factor of the
                ///< environment resolution.
    bool m_boundaryNodes; ///< True if boundary configurations have been found
    shared_ptr<BoundingBox> m_bbx; ///< Sampling domain
};

template<class MPTraits>
DynamicDomainRRT<MPTraits>::
DynamicDomainRRT(string _dm, string _nf, string _vc, string _nc,
    string _gt, string _extenderLabel, vector<string> _evaluators,
    double _minDist, double _growthFocus, bool _evaluateGoal,
    const CfgType& _start, const CfgType& _goal, size_t _numRoots,
    size_t _numDirections, size_t _maxTrial, bool _growGoals, double _r) :
    BasicRRTStrategy<MPTraits>(_dm, _nf, _vc, _nc, _gt, _extenderLabel,
        _evaluators, _minDist, _growthFocus, _evaluateGoal, _start, _goal,
        _numRoots, _numDirections, _maxTrial, _growGoals),
    m_r(_r) {
  this->m_meLabels = _evaluators;
  this->SetName("DynamicDomainRRT");
}

template<class MPTraits>
DynamicDomainRRT<MPTraits>::
DynamicDomainRRT(MPProblemType* _problem, XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_problem, _node) {
  this->SetName("DynamicDomainRRT");
  m_r = _node.Read("r", true, 10., 0., MAX_DBL,
      "R value of approach. Should be a multiple of environment resolution.");
}

template<class MPTraits>
DynamicDomainRRT<MPTraits>::
~DynamicDomainRRT() {
}

template<class MPTraits>
void
DynamicDomainRRT<MPTraits>::
Print(ostream& _os) const {
  BasicRRTStrategy<MPTraits>::Print(_os);
  _os << "\tr:: " << m_r << endl;
}

template<class MPTraits>
void
DynamicDomainRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();
  for(auto v  : *this->GetRoadmap()->GetGraph())
    v.property().SetStat(RLabel(), MAX_DBL);

  ///TODO: Allow construction from bounding sphere as well
  m_boundaryNodes = false;
  m_bbx = shared_ptr<BoundingBox>(
      new BoundingBox(
        *dynamic_pointer_cast<BoundingBox>(
          this->GetEnvironment()->GetBoundary())));
}

template<class MPTraits>
typename MPTraits::CfgType
DynamicDomainRRT<MPTraits>::
SelectDirection(){
  CfgType dir;
  dir.GetRandomCfg(this->GetEnvironment(), m_bbx);
  return dir;
}

template<class MPTraits>
typename DynamicDomainRRT<MPTraits>::VID
DynamicDomainRRT<MPTraits>::
ExpandTree(CfgType& _dir) {
  // Setup MP Variables
  StatClass* stats = this->GetStatClass();
  DistanceMetricPointer dm = this->GetDistanceMetric(this->m_dm);
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(this->m_nf);
  ExtenderPointer e = this->GetExtender(this->m_extenderLabel);
  RoadmapType* rdmp = this->GetRoadmap();
  GraphType* g = rdmp->GetGraph();

  VID recentVID = INVALID_VID;

  // Find closest Cfg in map
  vector<pair<VID, double> > neighbors;
  vector<CfgType> cfgs;

  size_t numRoadmapVertex  = g->get_num_vertices();

  size_t treeSize = 0;
  for(auto& tree : this->m_trees)
    treeSize += tree.size();

  bool fixTree = false;
  if(treeSize > numRoadmapVertex)
    fixTree = true;
  else {
    vector<pair<size_t, VID>> ccs;
    stapl::sequential::vector_property_map<GraphType, size_t> cmap;
    get_cc_stats(*g, cmap, ccs);
    if(ccs.size() != this->m_trees.size())
      fixTree = true;
  }

  //node deleted by dynamic environment, fix all trees
  if(fixTree) {
    this->m_trees.clear();
    vector<pair<size_t, VID>> ccs;
    stapl::sequential::vector_property_map<GraphType, size_t> cmap;
    get_cc_stats(*g, cmap, ccs);
    vector<VID> ccVIDs;
    for(auto& cc : ccs) {
      cmap.reset();
      ccVIDs.clear();
      get_cc(*g, cmap, cc.second, ccVIDs);
      this->m_trees.push_back(ccVIDs);
    }
    this->m_currentTree = this->m_trees.begin();
  }

  stats->StartClock("NeighborhoodFinding");

  nf->FindNeighbors(this->GetRoadmap(), this->m_currentTree->begin(),
      this->m_currentTree->end(), _dir, back_inserter(neighbors));

  stats->StopClock("NeighborhoodFinding");

  VID nearVID = neighbors[0].first;
  CfgRef nearest = g->GetVertex(neighbors[0].first);

  //Early quit if q_near is not within q_near.radius of q_rand
  if(dm->Distance(nearest, _dir) >= nearest.GetStat(RLabel()))
    return INVALID_VID;

  CfgType newCfg;
  vector<CfgType> intermediates, rintermediates;

  stats->StartClock("Extend");
  LPOutput<MPTraits> lpOutput;
  bool extendSucc = e->Extend(nearest, _dir, newCfg, lpOutput);
  intermediates = lpOutput.m_intermediates;
  stats->StopClock("Extend");

  double dist;
  if(intermediates.empty())
    dist = dm->Distance(nearest, newCfg);
  else {
    dist = dm->Distance(nearest, intermediates.front());
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit1 = intermediates.begin(),
        cit2 = cit1 + 1; cit2 != intermediates.end(); ++cit1, ++cit2)
      dist += dm->Distance(*cit1, *cit2);
    dist += dm->Distance(intermediates.back(), newCfg);
    copy(intermediates.rbegin(), intermediates.rend(),
        back_inserter(rintermediates));
  }

  //failed expansion, set q_near.radius to R and return
  if(!extendSucc) {
    if(this->m_debug)
      cout << "RRT could not expand!" << endl;
    nearest.SetStat(RLabel(), m_r);
    if(m_boundaryNodes) {
      m_bbx->GetRange(0) = make_pair(
          min(nearest[0]-m_r, m_bbx->GetRange(0).first),
          max(nearest[0]+m_r, m_bbx->GetRange(0).second)
          );
      m_bbx->GetRange(1) = make_pair(
          min(nearest[1]-m_r, m_bbx->GetRange(1).first),
          max(nearest[1]+m_r, m_bbx->GetRange(1).second)
          );
      m_bbx->GetRange(2) =
        m_bbx->GetRange(2).second == numeric_limits<double>::max() ?
        make_pair(-numeric_limits<double>::max(), numeric_limits<double>::max()) :
        make_pair(
            min(nearest[2]-m_r, m_bbx->GetRange(2).first),
            max(nearest[2]+m_r, m_bbx->GetRange(2).second)
            );
    }
    else {
      m_bbx->GetRange(0) = make_pair(nearest[0]-m_r, nearest[0]+m_r);
      m_bbx->GetRange(1) = make_pair(nearest[1]-m_r, nearest[1]+m_r);
      m_bbx->GetRange(2) =
        m_bbx->GetRange(2).second == numeric_limits<double>::max() ?
        make_pair(-numeric_limits<double>::max(), numeric_limits<double>::max()) :
        make_pair(nearest[2]-m_r, nearest[2]+m_r);
      m_boundaryNodes = true;
    }
    return recentVID;
  }

  static size_t expansions = 0;

  if(this->m_debug)
    cout << "Expansion::" << ++expansions
         << "\tfrom " << nearVID << " to " << newCfg << endl;

  // If good to go, add to roadmap
  if(dist >= this->m_minDist) {
    recentVID = g->AddVertex(newCfg);
    g->GetVertex(recentVID).SetStat("Parent", neighbors[0].first);
    g->GetVertex(recentVID).SetStat(RLabel(), MAX_DBL);
    this->m_currentTree->push_back(recentVID);
    if(std::string::npos != this->m_gt.find("UNDIRECTED"))
      g->AddEdge(nearVID, recentVID, lpOutput.m_edge);
    else
      g->AddEdge(nearVID, recentVID, lpOutput.m_edge.first);

    //If Graph type is GRAPH not TREE, then perform connection phase, i.e., this
    //is what RRG goes.
    if(std::string::npos != this->m_gt.find("GRAPH")) {
      if(this->m_debug) {
        cout << "tree roots:\n";
        for(auto& tree : this->m_trees)
          cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
        cout << "connecting neighbors...\n";
      }
      this->ConnectNeighbors(recentVID);
      if(this->m_debug) {
        cout << "tree roots:\n";
        for(auto& tree : this->m_trees)
          cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
      }
    }

    for(size_t i=2; i <= this->m_numDirections; i++) {
      //expansion to other m-1 directions
      CfgType randdir = this->SelectDispersedDirection(nearVID);
      stats->StartClock("Extend");
      vector<CfgType> intermediates, rintermediates;
      bool extendSucc = e->Extend(nearest, randdir, newCfg, lpOutput);
      intermediates = lpOutput.m_intermediates;
      if(intermediates.empty())
        dist = dm->Distance(nearest, newCfg);
      else {
        dist = dm->Distance(nearest, intermediates.front());
        typedef typename vector<CfgType>::iterator CIT;
        for(CIT cit1 = intermediates.begin(), cit2 = cit1 + 1;
            cit2 != intermediates.end(); ++cit1, ++cit2)
          dist += dm->Distance(*cit1, *cit2);
        dist += dm->Distance(intermediates.back(), newCfg);
        copy(intermediates.rbegin(), intermediates.rend(),
            back_inserter(rintermediates));
      }
      stats->StopClock("Extend");

      if(!extendSucc) {
        if(this->m_debug)
          cout << "RRT could not expand to additional directions!" << endl;
      }
      else if(dist >= this->m_minDist) {
        VID otherVID = g->AddVertex(newCfg);
        g->GetVertex(otherVID).SetStat("Parent", nearVID);
        g->GetVertex(otherVID).SetStat(RLabel(), MAX_DBL);
        this->m_currentTree->push_back(otherVID);
        if(std::string::npos != this->m_gt.find("UNDIRECTED"))
          g->AddEdge(nearVID, otherVID, lpOutput.m_edge);
        else
          g->AddEdge(nearVID, otherVID, lpOutput.m_edge.first);

        if(std::string::npos != this->m_gt.find("GRAPH")){
          if(this->m_debug) {
            cout << "tree roots:\n";
            for(auto& tree : this->m_trees)
              cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
          }
          string conClockName = "Connection time ";
          stats->StartClock(conClockName);

          this->ConnectNeighbors(otherVID);

          stats->StopClock(conClockName);
          if(this->m_debug) {
            cout << "tree roots:\n";
            for(auto& tree : this->m_trees)
              cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
          }
        }
      }
    }
  }
  else {
    if(this->m_debug)
      cout << "\t(expansion too close, not adding)\n";
  }

  return recentVID;
}

#endif
