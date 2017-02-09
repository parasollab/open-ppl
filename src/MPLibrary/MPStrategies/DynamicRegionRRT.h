#ifndef DYNAMIC_REGION_RRT_H_
#define DYNAMIC_REGION_RRT_H_

#include <queue>
#include <unordered_map>

#include "BasicRRTStrategy.h"

#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"
#include "Utilities/ReebGraphConstruction.h"

#include "Utilities/MedialAxisUtilities.h"
#include "Workspace/WorkspaceSkeleton.h"

#ifdef VIZMO
#include "GUI/ModelSelectionWidget.h"
#include "Models/TempObjsModel.h"
#include "Models/ThreadSafeSphereModel.h"
#include "Models/Vizmo.h"
#endif


////////////////////////////////////////////////////////////////////////////////
/// DynamicRegionRRT uses an embedded Reeb graph to guide dynamic sampling
/// regions through the environment.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DynamicRegionRRT : public BasicRRTStrategy<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    ///@}
    ///@name Local Types
    ///@{

    typedef ReebGraphConstruction::FlowGraph FlowGraph;

    ///@}
    ///@name Construction
    ///@{

    DynamicRegionRRT(string _dm = "euclidean", string _nf = "Nearest",
        string _vc = "rapid", string _nc = "kClosest", string _ex = "BERO",
        vector<string> _evaluators = vector<string>(),
        string _gt = "UNDIRECTED_TREE",  bool _growGoals = false,
        double _growthFocus = .05, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3);

    DynamicRegionRRT(XMLNode& _node);

    virtual ~DynamicRegionRRT() = default;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Run() override;

    ///@}

  protected:

    ///@name RRT Overrides
    ///@{

    /// Computes the growth direction for the RRT, choosing between the entire
    /// environment and each attract region with uniform probability to generate
    /// q_rand.
    /// @return The resulting growth direction.
    virtual CfgType SelectDirection() override;

    ///@}

  private:

    ///@}
    ///@name Internal State
    ///@{

    bool m_prune{true};          ///< Prune the flow graph?

    double m_regionFactor{2.5};  ///< The region radius is this * robot radius.
    double m_robotFactor{1.};
        ///< The robot is touching a sampling region if inside by this amount.

    Boundary* m_samplingRegion{nullptr};         ///< The current sampling region.

    ReebGraphConstruction* m_reebGraph{nullptr}; ///< Embedded reeb graph
    WorkspaceSkeleton m_skeleton;                ///< Workspace skeleton

    ///@}
};

/*------------------------------ Construction --------------------------------*/


template <typename MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(string _dm, string _nf, string _vc, string _nc, string _ex,
    vector<string> _evaluators, string _gt, bool _growGoals,
    double _growthFocus, size_t _numRoots, size_t _numDirections,
    size_t _maxTrial) :
    BasicRRTStrategy<MPTraits>(_dm, _nf, _vc, _nc, _ex, _evaluators, _gt,
        _growGoals, _growthFocus, _numRoots, _numDirections, _maxTrial) {
  this->SetName("DynamicRegionRRT");
}


template <typename MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("DynamicRegionRRT");
  m_regionFactor = _node.Read("regionFactor", false, 2.5, 1., 4., "The region "
      "radius is this * robot radius");
  m_prune = _node.Read("pruneFlowGraph", false, true, "Enable/disable flow "
      "graph pruning");
  m_robotFactor = _node.Read("robotFactor", false, 1., 0., 1., "The robot is "
      "touch if inside by this amount");
}

/*----------------------- MPStrategyMethod Overriddes ------------------------*/

template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();

  auto env = this->GetEnvironment();

  StatClass* stats = this->GetStatClass();
  stats->StartClock("SkeletonConstruction");

  //Embed ReebGraph
  delete m_reebGraph;
  m_reebGraph = new ReebGraphConstruction();
  m_reebGraph->Construct(env, this->GetBaseFilename());

  // Create the workspace skeleton.
  m_skeleton = m_reebGraph->GetSkeleton();

  // Direct the workspace skeleton outward from the starting point.
  const CfgType& s = this->m_query->GetQuery()[0];
  Vector3d start(s[0], s[1], s[2]);
  m_skeleton = m_skeleton.Direct(start);

  // Prune the workspace skeleton relative to the goal.
  const CfgType& goalCfg = this->m_query->GetQuery()[1];
  m_skeleton.PruneFlowGraph(goalCfg);
  //if(m_prune)
  //  m_skeleton.PushToMedialAxis();

  // Spark a region for each outgoing edge of start
  const double robotRadius = s.GetMultiBody()->GetBoundingSphereRadius();
  const double regionRadius = m_regionFactor * robotRadius;
  m_skeleton.MarkAllNodesUnvisited();
  m_skeleton.InitRegions(start, regionRadius);

  stats->StopClock("SkeletonConstruction");

#ifdef VIZMO
  GetVizmo().GetEnv()->AddWorkspaceDecompositionModel(env->GetDecomposition());
  GetVizmo().GetEnv()->AddReebGraphModel(m_reebGraph);
  GetMainWindow()->GetModelSelectionWidget()->CallResetLists();

  // Make map non-selectable during execution.
  GetVizmo().GetMap()->SetSelectable(false);
#endif
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
Run() {
  if(this->m_debug)
    cout << "\nBegin DynamicRegionRRT::Run" << endl;

  StatClass* stats = this->GetStatClass();
  stats->StartClock("DynamicRegionRRT::Run");

  const CfgType& s = this->m_query->GetQuery()[0];
  Vector3d start(s[0], s[1], s[2]);
  const double robotRadius = s.GetMultiBody()->GetBoundingSphereRadius();
  const double regionRadius = m_regionFactor * robotRadius;

  CfgType target(this->GetTask()->GetRobot());
  while(!this->EvaluateMap()) {
    // Find growth direction: either sample or bias towards a goal.
    if(this->m_query && DRand() < this->m_growthFocus &&
        !this->m_query->GetGoals().empty())
      target = this->m_query->GetRandomGoal();
    else
      target = this->SelectDirection();

    // Randomize Current Tree
    this->m_currentTree = this->m_trees.begin() + LRand() % this->m_trees.size();

    // Try to extend the tree towards target.
    VID recent = this->ExpandTree(target);

    // If the attempt succeeds, update the regions.
    if(recent != INVALID_VID) {
      CfgType& newest = this->GetRoadmap()->GetGraph()->GetVertex(recent);

      // Clear failed sampling attempts on current region.
      if(m_samplingRegion)
        m_skeleton.SetFailedAttempts(m_samplingRegion, 0);

      // Move existing regions along.
      m_skeleton.AdvanceRegions(newest);

      // Create new regions near newest.
      Vector3d p(newest[0], newest[1], newest[2]);
      m_skeleton.CreateRegions(p, regionRadius);

      // Connect trees if there are more than one.
      this->ConnectTrees(recent);
    }
    // If the attempt fails and we are using a dynamic region, reset its failure
    // count.
    else if(m_samplingRegion) {
      /// @TODO
      //++get<2>(regions[m_samplingRegion]);
      //if(get<2>(regions[m_samplingRegion]) > 1000) {
      //  auto rit = find(m_regions.begin(), m_regions.end(), m_samplingRegion);
      //  m_regions.erase(rit);
      //  regions.erase(m_samplingRegion);
      //}
    }
#ifdef VIZMO
    GetVizmo().GetMap()->RefreshMap();
  }
  GetVizmo().GetMap()->SetSelectable(true);
#else
  }
#endif

  stats->StopClock("DynamicRegionRRT::Run");

  if(this->m_debug)
    std::cout << "\nEnd DynamicRegionRRT::Run" << std::endl;
}

/*----------------------------- RRT Overrides --------------------------------*/

template <typename MPTraits>
typename DynamicRegionRRT<MPTraits>::CfgType
DynamicRegionRRT<MPTraits>::
SelectDirection() {
  const Boundary* samplingBoundary;
  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();

  // Randomly select a sampling region.
  auto& regions = m_skeleton.GetRegions();
  size_t index = LRand() % (regions.size() + 1);
  if(index == regions.size()) {
    m_samplingRegion = nullptr;
    samplingBoundary = env->GetBoundary();
  }
  else {
    m_samplingRegion = regions[index];
    samplingBoundary = m_samplingRegion;
  }

  // Generate the target q_rand from within the selected region.
  try {
    CfgType mySample(robot);
    mySample.GetRandomCfg(env, samplingBoundary);
    return mySample;
  }
  // Catch Boundary too small exception.
  catch(PMPLException _e) {
    CfgType mySample(robot);
    mySample.GetRandomCfg(env);
    return mySample;
  }
  catch(std::exception _e) {
    std::cerr << _e.what() << std::endl;
    std::exit(1);
  }
}

/*----------------------------------------------------------------------------*/

#endif
