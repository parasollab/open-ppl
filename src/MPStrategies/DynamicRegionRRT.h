#ifndef DYNAMIC_REGION_RRT_H_
#define DYNAMIC_REGION_RRT_H_

#include <unordered_map>

#include "BasicRRTStrategy.h"

#include "Environment/Boundary.h"
#include "Environment/BoundingSphere.h"
#include "Utilities/ReebGraphConstruction.h"
#include "Utilities/TetGenDecomposition.h"

extern TetGenDecomposition* t;

////////////////////////////////////////////////////////////////////////////////
/// \brief  DynamicRegionRRT
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class DynamicRegionRRT : public BasicRRTStrategy<MPTraits> {

  public:

    // Local Types
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer
        NeighborhoodFinderPointer;
    typedef shared_ptr<Boundary> RegionPtr;

    // Construction
    DynamicRegionRRT(const CfgType& _start = CfgType(),
        const CfgType& _goal = CfgType(),
        string _dm = "euclidean", string _nf = "BFNF", string _vc = "PQP_SOLID",
        string _nc = "kClosest", string _gt = "UNDIRECTED_TREE",
        string _extenderLabel = "BERO",
        vector<string> _evaluators = vector<string>(),
        double _minDist = 0.001, double _growthFocus = 0.05,
        bool _evaluateGoal = true, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3,
        bool _growGoals = false);
    DynamicRegionRRT(MPProblemType* _problem, XMLNode& _node);

    // Inherited functions
    void Initialize();
    void Run();
    void Finalize();

  private:

    ////////////////////////////////////////////////////////////////////////////
    /// \brief  Computes the growth direction for the RRT, choosing between the
    ///         entire environment and each attract region with uniform
    ///         probability to generate q_rand.
    /// \return The resulting growth direction.
    CfgType SelectDirection();

    vector<RegionPtr> m_regions; ///< All Regions
    RegionPtr m_samplingRegion;  ///< Points to the current sampling region.

    TetGenDecomposition* m_tetrahedralization; ///< TetGen decomposition
    string m_switches;             ///< Input switches to TetGen model
    bool m_writeFreeModel;         ///< Output TetGen model of freespace
    bool m_writeDecompModel;       ///< Output TetGen model tetrahedrons

    ReebGraphConstruction* m_reebGraphConstruction; ///< Embedded reeb graph
    bool m_readReeb;
    bool m_writeReeb;
    string m_reebFilename;
};


template<class MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(const CfgType& _start, const CfgType& _goal, string _dm,
    string _nf, string _vc, string _nc, string _gt, string _extenderLabel,
    vector<string> _evaluators, double _minDist,
    double _growthFocus, bool _evaluateGoal, size_t _numRoots,
    size_t _numDirections, size_t _maxTrial, bool _growGoals) :
  BasicRRTStrategy<MPTraits>(_dm, _nf, _vc, _nc, _gt, _extenderLabel,
      _evaluators, _minDist, _growthFocus, _evaluateGoal,
      _start, _goal, _numRoots, _numDirections, _maxTrial, _growGoals) {
    this->SetName("DynamicRegionRRT");
    m_switches = "pqnQ";
    m_writeFreeModel = false;
    m_writeDecompModel = false;
    m_readReeb = false;
    m_writeReeb = false;
  }

template<class MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(MPProblemType* _problem, XMLNode& _node) :
  BasicRRTStrategy<MPTraits>(_problem, _node) {
    this->SetName("DynamicRegionRRT");
    m_switches = "pqn";
    m_writeFreeModel = false;
    m_writeDecompModel = false;

    m_readReeb = _node.Read("readReeb", false, false, "Read Reeb Graph from file");
    m_reebFilename = _node.Read("reebFilename", m_readReeb, "", "Filename for Read "
        "or write ReebGraph operations.");
    m_writeReeb = _node.Read("writeReeb", false, false, "Write Reeb Graph to file");
  }

template<class MPTraits>
void
DynamicRegionRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();

  StatClass* stats = this->GetStatClass();

  //Tetrahedralize environment
  m_tetrahedralization = new TetGenDecomposition(m_switches,
      m_writeFreeModel, m_writeDecompModel);
  stats->StartClock("Tetrahedralization");
  m_tetrahedralization->Decompose(this->GetEnvironment());
  stats->StopClock("Tetrahedralization");

  //Embed ReebGraph
  stats->StartClock("ReebGraphConstruction");
  if(m_readReeb)
    m_reebGraphConstruction = new ReebGraphConstruction(MPProblemType::GetPath(m_reebFilename));
  else
    m_reebGraphConstruction = new ReebGraphConstruction(m_tetrahedralization);
  stats->StopClock("ReebGraphConstruction");

  if(m_writeReeb)
    m_reebGraphConstruction->Write(this->GetBaseFilename() + ".reeb");
}


template<class MPTraits>
void
DynamicRegionRRT<MPTraits>::
Run() {
  if(this->m_debug)
    cout << "\nBegin DynamicRegionRRT::Run" << endl;

  // Setup MP Variables
  StatClass* stats = this->GetStatClass();
  Environment* env = this->GetEnvironment();

  stats->StartClock("DynamicRegionRRT");

  CfgRef s = this->m_query->GetQuery()[0];
  Vector3d start(s[0], s[1], s[2]);

  //Get directed flow network
  typedef ReebGraphConstruction::FlowGraph FlowGraph;
  typedef FlowGraph::vertex_descriptor FVD;
  typedef FlowGraph::edge_descriptor FED;
  pair<FlowGraph, FVD> flow = m_reebGraphConstruction->
    GetFlowGraph(start, env->GetPositionRes());

  unordered_map<FVD, bool> visited;
  for(auto vit = flow.first.begin(); vit != flow.first.end(); ++vit)
    visited[vit->descriptor()] = false;

  //Spark a region for each outgoing edge of start

  //Region structure stores tuple of flow edge descriptor,
  //index along flow edge, number of failed extentions
  unordered_map<RegionPtr, tuple<FED, size_t, size_t>> regions;

  double robotRadius = env->GetRobot(0)->GetBoundingSphereRadius();
  auto sit = flow.first.find_vertex(flow.second);
  for(auto eit = sit->begin(); eit != sit->end(); ++eit) {
    auto i = regions.emplace(
        RegionPtr(new BoundingSphere(start, 3*robotRadius)),
        make_tuple(eit->descriptor(), 0, 0));
    m_regions.push_back(i.first->first);
  }
  visited[sit->descriptor()] = true;

  CfgType dir;
  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation) {
    //find my growth direction. Default is to randomly select node or bias towards a goal
    double randomRatio = DRand();
    if(randomRatio < this->m_growthFocus)
      dir = this->GoalBiasedDirection();
    else
      dir = SelectDirection();

    // Randomize Current Tree
    this->m_currentTree = this->m_trees.begin() + LRand() % this->m_trees.size();

    VID recent = this->ExpandTree(dir);
    if(recent != INVALID_VID) {

      CfgRef newest = this->GetRoadmap()->GetGraph()->GetVertex(recent);

      if(m_samplingRegion) {
        get<2>(regions[m_samplingRegion]) = 0;

        while(env->InBounds(newest, m_samplingRegion)) {
          Vector3d cur = m_samplingRegion->GetCenter();

          auto& pr = regions[m_samplingRegion];
          FlowGraph::vertex_iterator vi;
          FlowGraph::adj_edge_iterator ei;
          flow.first.find_edge(get<0>(pr), vi, ei);
          vector<Vector3d>& path = ei->property();
          size_t& i = get<1>(pr);
          size_t j = i+1;
          if(j < path.size()) {
            Vector3d& next = path[j];
            m_samplingRegion->ApplyOffset(next-cur);
            i = j;
          }
          //else need to delete region
          else {
            auto rit = find(m_regions.begin(), m_regions.end(), m_samplingRegion);
            m_regions.erase(rit);
            regions.erase(m_samplingRegion);
            break;
          }
        }
      }

      //Add new regions
      Vector3d p(newest[0], newest[1], newest[2]);

      for(auto vit = flow.first.begin(); vit != flow.first.end(); ++vit) {
        double dist = (vit->property() - p).norm();
        if(dist < 3*robotRadius && !visited[vit->descriptor()]) {
          for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
            auto i = regions.emplace(
                RegionPtr(new BoundingSphere(vit->property(), 3*robotRadius)),
                make_tuple(eit->descriptor(), 0, 0));
            m_regions.push_back(i.first->first);
          }
          visited[vit->descriptor()] = true;
        }
      }

      //connect various trees together
      this->ConnectTrees(recent);
      //see if tree is connected to goals
      if(this->m_evaluateGoal)
        this->EvaluateGoals(recent);

      //evaluate the roadmap
      bool evalMap = this->EvaluateMap();
      bool oneTree = this->m_trees.size() == 1;
      if(!this->m_growGoals) {
        bool useGoals = this->m_evaluateGoal;
        bool allGoals = useGoals && this->m_goalsNotFound.size() == 0;
        mapPassedEvaluation = evalMap && oneTree && (allGoals || !useGoals);
        if(this->m_debug && allGoals)
          cout << "RRT FOUND ALL GOALS" << endl;
        if(this->m_trees.begin()->size() >= 15000)
          mapPassedEvaluation = true;
      }
      else
        mapPassedEvaluation = evalMap && oneTree;
    }
    else {
      if(m_samplingRegion) {
        ++get<2>(regions[m_samplingRegion]);
        if(get<2>(regions[m_samplingRegion]) > 100) {
          auto rit = find(m_regions.begin(), m_regions.end(), m_samplingRegion);
          m_regions.erase(rit);
          regions.erase(m_samplingRegion);
        }
      }
      mapPassedEvaluation = false;
    }
  }

  stats->StopClock("DynamicRegionRRT");

  m_regions.clear();
  regions.clear();

  if(this->m_debug)
    cout<<"\nEnd DynamicRegionRRT::Run" << endl;
}


template<class MPTraits>
void
DynamicRegionRRT<MPTraits>::
Finalize() {
  BasicRRTStrategy<MPTraits>::Finalize();
}

template<class MPTraits>
typename DynamicRegionRRT<MPTraits>::CfgType
DynamicRegionRRT<MPTraits>::
SelectDirection() {
  RegionPtr samplingBoundary;
  Environment* env = this->GetEnvironment();

  size_t _index = rand() % (m_regions.size() + 1);

  if(_index == m_regions.size()) {
    m_samplingRegion.reset();
    samplingBoundary = this->GetEnvironment()->GetBoundary();
  }
  else {
    m_samplingRegion = m_regions[_index];
    samplingBoundary = m_samplingRegion;
  }

  try {
    CfgType mySample;
    mySample.GetRandomCfg(env,samplingBoundary);
    return mySample;
  }
  //catch Boundary too small exception
  catch(PMPLException _e) {
    CfgType mySample;
    mySample.GetRandomCfg(env);
    return mySample;
  }
  //catch all others and exit
  catch(exception _e) {
    cerr << _e.what() << endl;
    exit(1);
  }
}

#endif
