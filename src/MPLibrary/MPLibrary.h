#ifndef PMPL_MP_LIBRARY_H_
#define PMPL_MP_LIBRARY_H_

#include "MPLibrary/MPSolution.h"

#include "MPLibrary/MPBaseObject.h"

#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"
#include "Utilities/XMLNode.h"
#include "Utilities/MethodSet.h"

#include "Traits/Methods.h"

#include "MPLibrary/Connectors/ConnectorMethod.h"
#include "MPLibrary/DistanceMetrics/DistanceMetricMethod.h"
#include "MPLibrary/Extenders/ExtenderMethod.h"
#include "MPLibrary/LocalPlanners/LocalPlannerMethod.h"
#include "MPLibrary/MapEvaluators/MapEvaluatorMethod.h"
#include "MPLibrary/Metrics/MetricMethod.h"
#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "MPLibrary/MPTools/MPTools.h"
#include "MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethod.h"
#include "MPLibrary/PathModifiers/PathModifierMethod.h"
#include "MPLibrary/EdgeValidityCheckers/EdgeValidityCheckerMethod.h"
#include "MPLibrary/Samplers/SamplerMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/ValidityCheckers/ValidityCheckerMethod.h"
#include "MPLibrary/GoalTracker.h"


////////////////////////////////////////////////////////////////////////////////
/// A collection of planning algorithms that can operate on a specific
/// MPProblem and MPTask.
////////////////////////////////////////////////////////////////////////////////
#ifdef _PARALLEL
class MPLibrary : public stapl::p_object
#else
class MPLibrary
#endif
{

  public:

    ///@name Motion Planning Types
    ///@{

    typedef MPSolutionType MPSolution;
    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> RoadmapType;
    typedef GroupRoadmap<GroupCfg<RoadmapType>, GroupLocalPlan<RoadmapType>> GroupRoadmapType;
    typedef size_t VID;
    typedef MPToolsType MPTools;
    typedef GoalTrackerType GoalTracker;

    ///@}
    ///@name Local Types
    ///@{

    /// Solver represents an input set to MPLibrary. It includes an
    /// MPStrategy label, seed, base file name, and vizmo debug option.
    struct Solver {
      std::string label;         ///< The XML label for the strategy to use.
      long seed;                 ///< The seed.
      std::string baseFilename;  ///< The base name for output files.
      bool vizmoDebug;           ///< Save vizmo debug info?
    };

    ///@}
    ///@name Method Set Types
    ///@{

    typedef MethodSet<DistanceMetricMethod> DistanceMetricSet;
    typedef MethodSet<ValidityCheckerMethod>
                                                            ValidityCheckerSet;
    typedef MethodSet<NeighborhoodFinderMethod>
                                                            NeighborhoodFinderSet;
    typedef MethodSet<SamplerMethod>        SamplerSet;
    typedef MethodSet<LocalPlannerMethod>   LocalPlannerSet;
    typedef MethodSet<ExtenderMethod>       ExtenderSet;
    typedef MethodSet<PathModifierMethod>   PathModifierSet;
    typedef MethodSet<EdgeValidityCheckerMethod>
                                                                EdgeValidityCheckerSet;
    typedef MethodSet<ConnectorMethod>      ConnectorSet;
    typedef MethodSet<MetricMethod>         MetricSet;
    typedef MethodSet<MapEvaluatorMethod>   MapEvaluatorSet;
    typedef MethodSet<MPStrategyMethod>     MPStrategySet;

    ///@}
    ///@name Method Pointer Types
    ///@{

    typedef typename ValidityCheckerSet::MethodPointer ValidityCheckerPointer;
    typedef typename DistanceMetricSet::MethodPointer  DistanceMetricPointer;
    typedef typename NeighborhoodFinderSet::MethodPointer
                                                       NeighborhoodFinderPointer;
    typedef typename SamplerSet::MethodPointer         SamplerPointer;
    typedef typename LocalPlannerSet::MethodPointer    LocalPlannerPointer;
    typedef typename ExtenderSet::MethodPointer        ExtenderPointer;
    typedef typename PathModifierSet::MethodPointer    PathModifierPointer;
    typedef typename EdgeValidityCheckerSet::MethodPointer
                                                       EdgeValidityCheckerPointer;
    typedef typename ConnectorSet::MethodPointer       ConnectorPointer;
    typedef typename MetricSet::MethodPointer          MetricPointer;
    typedef typename MapEvaluatorSet::MethodPointer    MapEvaluatorPointer;
    typedef typename MPStrategySet::MethodPointer      MPStrategyPointer;

    ///@}
    ///@name Construction
    ///@{

    MPLibrary();

    MPLibrary(const std::string& _filename);

    MPLibrary(XMLNode& planningLibraryNode);

    virtual ~MPLibrary();

    ///@}
    ///@name Configuration
    ///@{

    /// Read an XML file to set the algorithms and parameters in this instance.
    /// @param _filename The XML file name.
    void ReadXMLFile(const std::string& _filename);

    void ProcessXML(XMLNode &node);

    ///@}
    ///@name Distance Metric Accessors
    ///@{

    DistanceMetricPointer GetDistanceMetric(const std::string& _l) {
      return m_distanceMetrics->GetMethod(_l);
    }
    void AddDistanceMetric(DistanceMetricPointer _dm, const std::string& _l) {
      m_distanceMetrics->AddMethod(_dm, _l);
    }

    ///@}
    ///@name Validity Checker Accessors
    ///@{

    ValidityCheckerPointer GetValidityChecker(const std::string& _l) {
      return m_validityCheckers->GetMethod(_l);
    }
    void AddValidityChecker(ValidityCheckerPointer _vc, const std::string& _l) {
      m_validityCheckers->AddMethod(_vc, _l);
    }

    /// Toggle (negate) the validity output for ALL validity checkers.
    // void ToggleValidity() {
    //   for(auto& vc : *m_validityCheckers)
    //     vc.second->ToggleValidity();
    // }

    ///@}
    ///@name Neighborhood Finder Accessors
    ///@{

    NeighborhoodFinderPointer GetNeighborhoodFinder(const std::string& _l) {
      return m_neighborhoodFinders->GetMethod(_l);
    }
    void AddNeighborhoodFinder(NeighborhoodFinderPointer _nf, const std::string& _l) {
      m_neighborhoodFinders->AddMethod(_nf, _l);
    }

    ///@}
    ///@name Sampler Accessors
    ///@{

    const SamplerSet* const GetSamplers() const {return m_samplers;}
    SamplerPointer GetSampler(const std::string& _l) {
      return m_samplers->GetMethod(_l);
    }
    void AddSampler(SamplerPointer _s, const std::string& _l) {
      m_samplers->AddMethod(_s, _l);
    }

    ///@}
    ///@name Local Planner Accessors
    ///@{

    LocalPlannerPointer GetLocalPlanner(const std::string& _l) {
      return m_localPlanners->GetMethod(_l);
    }
    void AddLocalPlanner(LocalPlannerPointer _lp, const std::string& _l) {
      m_localPlanners->AddMethod(_lp, _l);
    }

    ///@}
    ///@name Extender Accessors
    ///@{

    ExtenderPointer GetExtender(const std::string& _l) {
      return m_extenders->GetMethod(_l);
    }
    void AddExtender(ExtenderPointer _mps, const std::string& _l) {
      m_extenders->AddMethod(_mps, _l);
    }

    ///@}
    ///@name Path Modifier Accessors
    ///@{

    PathModifierPointer GetPathModifier(const std::string& _l) {
      return m_pathModifiers->GetMethod(_l);
    }
    void AddPathModifier(PathModifierPointer _ps, const std::string& _l) {
      m_pathModifiers->AddMethod(_ps, _l);
    }

    ///@}
    ///@name Edge Validity Checker Accessors
    ///@{

    EdgeValidityCheckerPointer GetEdgeValidityChecker(const std::string& _l) {
      return m_edgeValidityCheckers->GetMethod(_l);
    }
    void AddEdgeValidityChecker(EdgeValidityCheckerPointer _ps, const std::string& _l) {
      m_edgeValidityCheckers->AddMethod(_ps, _l);
    }

    ///@}
    ///@}
    ///@name Connector Accessors
    ///@{

    ConnectorPointer GetConnector(const std::string& _l) {
      return m_connectors->GetMethod(_l);
    }
    void AddConnector(ConnectorPointer _c, const std::string& _l) {
      m_connectors->AddMethod(_c, _l);
    }

    ///@}
    ///@name Metric Accessors
    ///@{

    MetricPointer GetMetric(const std::string& _l) {return m_metrics->GetMethod(_l);}
    void AddMetric(MetricPointer _m, const std::string& _l) {
      m_metrics->AddMethod(_m, _l);
    }

    ///@}
    ///@name Map Evaluator Accessors
    ///@{

    MapEvaluatorPointer GetMapEvaluator(const std::string& _l) {
      return m_mapEvaluators->GetMethod(_l);
    }
    void AddMapEvaluator(MapEvaluatorPointer _me, const std::string& _l) {
      m_mapEvaluators->AddMethod(_me, _l);
    }

    /// For cases where we need to reset all instances of TimeEvaluator.
/*    void ResetTimeEvaluators() {
      for(auto& labelPtr : *m_mapEvaluators) {
        auto t = dynamic_cast<TimeEvaluator*>(labelPtr.second.get());
        if(t)
          t->Initialize();
      }
    }
*/
    ///@}
    ///@name MPStrategy Accessors
    ///@{

    const MPStrategySet* const GetMPStrategies() const {return m_mpStrategies;}
    MPStrategyPointer GetMPStrategy(const std::string& _l) {
      return m_mpStrategies->GetMethod(_l);
    }
    void AddMPStrategy(MPStrategyPointer _mps, const std::string& _l) {
      m_mpStrategies->AddMethod(_mps, _l);
    }

    ///@}
    ///@name MPTools Accessors
    ///@{

    MPTools* GetMPTools() {return m_mpTools;}

    ///@}
    ///@name Input Accessors
    ///@{

    MPProblem* GetMPProblem() const noexcept;
    void SetMPProblem(MPProblem* const _problem) noexcept;

    MPTask* GetTask() const noexcept;
    void SetTask(MPTask* const _task) noexcept;

    GroupTask* GetGroupTask() const noexcept;
    void SetGroupTask(GroupTask* const _task) noexcept;

    const std::string& GetBaseFilename() const noexcept;
    void SetBaseFilename(const std::string& _s) noexcept;

    ///@}
    ///@name Solution Accessors
    ///@{

    MPSolution* GetMPSolution() const noexcept;
    void SetMPSolution(MPSolution* _sol) noexcept;

    RoadmapType*      GetRoadmap(Robot* const _r = nullptr) const noexcept;
    GroupRoadmapType* GetGroupRoadmap(RobotGroup* const _g = nullptr) const noexcept;
    RoadmapType*      GetBlockRoadmap(Robot* const _r = nullptr) const noexcept;
    Path*             GetPath(Robot* const _r = nullptr) const noexcept;
    GroupPath*        GetGroupPath(RobotGroup* const _g = nullptr) const noexcept;
    LocalObstacleMap* GetLocalObstacleMap(Robot* const _r = nullptr) const noexcept;

    GoalTracker*      GetGoalTracker() const noexcept;
    StatClass*        GetStatClass() const noexcept;

    ///@}
    ///@name Edge Reconstruction
    ///@{

    /// Recompute a roadmap edge path at full resolution (source and target cfgs
    /// are not included).
    /// @param _roadmap The roadmap pointer.
    /// @param _source The source VID.
    /// @param _target The target VID.
    /// @param _posRes The position resolution to use.
    /// @param _oriRes The orientation resolution to use.
    /// @return A vector of cfgs along the edge, spaced at resolution intervals.
    std::vector<typename RoadmapType::VP> ReconstructEdge(
        RoadmapType* const _roadmap,
        const VID _source, const VID _target,
        const double _posRes, const double _oriRes);

    /// @overload This version is for groups.
    std::vector<typename GroupRoadmapType::VP> ReconstructEdge(
        GroupRoadmapType* const _roadmap,
        const VID _source, const VID _target,
        const double _posRes, const double _oriRes);

    /// @overload This version uses the Environment's resolutions.
    template <typename AbstractRoadmapType>
    std::vector<typename AbstractRoadmapType::VP> ReconstructEdge(
        AbstractRoadmapType* const _roadmap,
        const VID _source, const VID _target);

    ///@}
    ///@name Execution Interface
    ///@{

    /// Halt the current strategy and move on to finalization. Useful for
    /// controlling the library from an external client object.
    void Halt();

    /// Check to see if the strategy should continue.
    bool IsRunning() const noexcept;

    /// Set the current seed to match the first solver node.
    void SetSeed() const noexcept;

    /// Set the current seed.
    /// @param _seed The seed value.
    void SetSeed(const long _seed) const noexcept;

    /// Add an input set to this MPLibrary.
    /// @param _label        The MPStrategy label to use.
    /// @param _seed         The random seed to use.
    /// @param _baseFileName The base name of the XML file to use.
    /// @param _vizmoDebug   Enable/disable vizmo debug.
    void AddSolver(const std::string& _label, long _seed,
        const std::string& _baseFileName, bool _vizmoDebug = false) {
      m_solvers.push_back(Solver{_label, _seed, _baseFileName, _vizmoDebug});
    }

    std::vector<std::string> GetSolverLabels() {
      std::vector<std::string> labels;
      for (auto& solver : m_solvers)
        labels.push_back(solver.label);
      return labels;
    }

    /// Run a single input (solver) and get back its solution.
    /// @param _problem The problem representation.
    /// @param _task The task representation.
    /// @param _solution The solution container, which may or may not
    ///                  already be populated. Existing solutions should
    ///                  be extended, not overwritten.
    void Solve(MPProblem* _problem, MPTask* _task, MPSolution* _solution);
    ///@example MPLibrary_UseCase.cpp
    /// This is an example of how to use the MPLibrary methods.

    /// Run each input (solver) in sequence.
    /// @param _problem The problem representation.
    /// @param _task The task representation.
    void Solve(MPProblem* _problem, MPTask* _task);

    /// Group overload:
    void Solve(MPProblem* _problem, GroupTask* _task);

    void Solve(MPProblem* _problem, GroupTask* _task, MPSolution* _solution);

    /// Run a specific MPStrategy from the XML file with a designated seed and
    /// base output file name. This is intended for use with the simulator where
    /// agents may need to call various strategies within a single execution.
    /// @param _problem The problem representation.
    /// @param _task The task representation.
    /// @param _solution The solution container, which may or may not
    ///                  already be populated. Existing solutions should
    ///                  be extended, not overwritten.
    /// @param _label The label of the MPStrategy to call.
    /// @param _seed The seed to use.
    /// @param _baseFilename The base name for output files.
    void Solve(MPProblem* _problem, MPTask* _task, MPSolution* _solution,
        const std::string& _label, const long _seed,
        const std::string& _baseFilename);

    ///@}
    ///@name Debugging
    ///@{

    void Print(std::ostream& _os) const; ///< Print each method set.

    ///@}

  private:

    ///@name Execution Helpers
    ///@{

    /// Execute a solver with the current problem, task, and solution.
    /// @param _s The solver to execute.
    void RunSolver(const Solver& _s);

    ///@}
    ///@name Construction Helpers
    ///@{

    /// Initialize all algorithms in each method set.
    void Initialize();

    /// Clear temporary states and variables.
    void Uninitialize();

    /// Helper for parsing XML nodes.
    /// @param _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Inputs
    ///@{

    MPProblem*          m_problem{nullptr};  ///< The current MPProblem.
    MPTask*             m_task{nullptr};     ///< The current task.
    GroupTask*          m_groupTask{nullptr};///< The current group task.
    MPSolution*         m_solution{nullptr}; ///< The current solution.
    std::vector<Solver> m_solvers;           ///< The set of inputs to execute.

    ///@}
    ///@name Other Library Objects
    ///@{
    /// These do not use method sets because the sub-objects are not expected to
    /// share a common interface.

    std::unique_ptr<GoalTracker> m_goalTracker;
    MPTools* m_mpTools{nullptr};

    ///@}
    ///@name Internal State
    ///@{

    std::atomic<bool> m_running{true};  ///< Keep running the strategy?

    ///@}

  protected:

    ///@name Method Sets
    ///@{
    /// Method sets hold and offer access to the motion planning objects of the
    /// corresponding type.

    DistanceMetricSet*      m_distanceMetrics{nullptr};
    ValidityCheckerSet*     m_validityCheckers{nullptr};
    NeighborhoodFinderSet*  m_neighborhoodFinders{nullptr};
    SamplerSet*             m_samplers{nullptr};
    LocalPlannerSet*        m_localPlanners{nullptr};
    ExtenderSet*            m_extenders{nullptr};
    PathModifierSet*        m_pathModifiers{nullptr};
    EdgeValidityCheckerSet* m_edgeValidityCheckers{nullptr};
    ConnectorSet*           m_connectors{nullptr};
    MetricSet*              m_metrics{nullptr};
    MapEvaluatorSet*        m_mapEvaluators{nullptr};
    MPStrategySet*          m_mpStrategies{nullptr};

    ///@}
};


template <typename AbstractRoadmapType>
std::vector<typename AbstractRoadmapType::VP>
MPLibrary::
ReconstructEdge(AbstractRoadmapType* const _roadmap, const VID _source,
    const VID _target) {
  auto env = this->GetMPProblem()->GetEnvironment();
  return this->ReconstructEdge(_roadmap, _source, _target,
                               env->GetPositionRes(), env->GetOrientationRes());
}

#endif
