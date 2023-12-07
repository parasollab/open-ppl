#ifndef PMPL_MP_BASE_OBJECT_H_
#define PMPL_MP_BASE_OBJECT_H_
///@TODO Remove this everywhere. We should never have 'using namespace'
///      directives in header files.
using namespace std;

#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/GroupPath.h"

#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "Utilities/IOUtils.h"
#include "Utilities/MethodSet.h"
#include "Utilities/XMLNode.h"

#include "MPLibrary/MPSolution.h"

#include <iostream>
#include <string>

template <typename Vertex, typename Edge> class GenericStateGraph;
class Environment;
class MPTask;
class StatClass;
class MPLibrary;

////////////////////////////////////////////////////////////////////////////////
/// Abstract base class for all algorithm abstractions in PMPL.
///
/// The MPBaseObject carries a class name @c m_name and unique label @c m_label
/// for each algorithm. The name refers to the class from which the object was
/// instantiated, while the label refers to a specific instantiation of that
/// class.
///
/// All algorithms are owned by an MPLibrary, which is referenced here as
/// @c m_library. When initially created, these objects will have no knowledge
/// of the MPProblem that they will be used on: they will only have access to
/// parameter settings provided in their XML nodes. Derived classes that have
/// problem-dependent internal state should override the @c Initialize() method
/// to set that data: this method will be called whenever the owning MPLibrary's
/// current MPProblem is changed.
////////////////////////////////////////////////////////////////////////////////
class MPBaseObject {

  public:

    ///@name Local Types
    ///@{

    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> RoadmapType;
    typedef GroupCfg<RoadmapType> GroupCfgType;
    typedef GroupLocalPlan<RoadmapType> GroupWeightType;
    typedef GroupRoadmap<GroupCfgType, GroupWeightType> GroupRoadmapType;

    // typedef typename MPLibrary::SamplerPointer         SamplerPointer;
    // typedef typename MPLibrary::LocalPlannerPointer    LocalPlannerPointer;
    // typedef typename MPLibrary::ExtenderPointer        ExtenderPointer;
    // typedef typename MPLibrary::PathModifierPointer    PathModifierPointer;
    // typedef typename MPLibrary::EdgeValidityCheckerPointer    
    //                                                    EdgeValidityCheckerPointer;
    // typedef typename MPLibrary::ConnectorPointer       ConnectorPointer;
    // typedef typename MPLibrary::MetricPointer          MetricPointer;
    // typedef typename MPLibrary::MapEvaluatorPointer    MapEvaluatorPointer;
    // typedef typename MPLibrary::MPStrategyPointer      MPStrategyPointer;
    // typedef typename MPLibrary::DistanceMetricPointer  DistanceMetricPointer;
    // typedef typename MPLibrary::ValidityCheckerPointer ValidityCheckerPointer;
    // typedef typename MPLibrary::NeighborhoodFinderPointer
    //                                                    NeighborhoodFinderPointer;

    ///@}
    ///@name Construction
    ///@{

    /// Default constructor explicitly gives name, label, and debug.
    /// @param _label ID of the object, i.e., user defined label
    /// @param _name Name of the object, i.e., derived class name
    /// @param _debug Turn debug output on or off
    MPBaseObject(const std::string& _label = "", const std::string& _name = "",
        bool _debug = false);

    /// XML constructor pulls label and debug from an XML node.
    /// @param _node XMLNode to parse for this object
    MPBaseObject(XMLNode& _node);

    virtual ~MPBaseObject();

    ///@}
    ///@name I/O
    ///@{

    /// Print internal state of this object.
    /// @param _os The std::ostream to print to.
    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name Initialization
    ///@{

    /// Initialize this object for the current MPProblem. This should reset any
    /// internal state of the algorithms so that they are ready for execution. It
    /// is also the place to initialize any state that depends on the current
    /// problem.
    /// @warning This member will be called for every compiled algorithm in the
    ///          planning library - even those that will not be used. If an
    ///          algorithm needs to do expenisve setup, then this method should
    ///          only set a flag that tells it to do so on first use. The only
    ///          exceptions are the MPStrategies, which will only have their
    ///          initialize called on first use.
    virtual void Initialize() {}

    ///@}
    ///@name Name and Label Accessors
    ///@{

    /// Get the class name for this object.
    const std::string& GetName() const;

    /// Get the unique label for this object.
    const std::string& GetLabel() const;

    /// Get the unique string identifier for this object "m_name::m_label".
    std::string GetNameAndLabel() const;

    /// Set the unique label for this object.
    void SetLabel(const std::string&);

    ///@}
    ///@name MPLibrary Accessors
    ///@{

    /// Set the owning MPLibrary.
    void SetMPLibrary(MPLibrary*) noexcept;

    /// Get the owning MPLibrary.
    MPLibrary* GetMPLibrary() const noexcept;

    /// Check the library's running flag.
    bool IsRunning() const noexcept;

    // /// Get a distance metric by label from the owning MPLibrary.
    // DistanceMetricPointer GetDistanceMetric(const std::string&) const noexcept;

    // /// Get a validity checker by label from the owning MPLibrary.
    // ValidityCheckerPointer GetValidityChecker(const std::string&) const noexcept;

    // /// Get a neighborhood finder by label from the owning MPLibrary.
    // NeighborhoodFinderPointer GetNeighborhoodFinder(const std::string&) const noexcept;

    // /// Get a sampler by label from the owning MPLibrary.
    // SamplerPointer GetSampler(const std::string&) const noexcept;

    // /// Get a local planner by label from the owning MPLibrary.
    // LocalPlannerPointer GetLocalPlanner(const std::string&) const noexcept;

    // /// Get an extender by label from the owning MPLibrary.
    // ExtenderPointer GetExtender(const std::string&) const noexcept;

    // /// Get a path modifier by label from the owning MPLibrary.
    // PathModifierPointer GetPathModifier(const std::string&) const noexcept;

    // /// Get a path modifier by label from the owning MPLibrary.
    // EdgeValidityCheckerPointer GetEdgeValidityChecker(const std::string&) const noexcept;

    // /// Get a connector by label from the owning MPLibrary.
    // ConnectorPointer GetConnector(const std::string&) const noexcept;

    // /// Get a metric by label from the owning MPLibrary.
    // MetricPointer GetMetric(const std::string&) const noexcept;

    // /// Get a map evaluator by label from the owning MPLibrary.
    // MapEvaluatorPointer GetMapEvaluator(const std::string&) const noexcept;

    // /// Get a strategy by label from the owning MPLibrary.
    // MPStrategyPointer GetMPStrategy(const std::string&) const noexcept;

    // /// Get the MPTools container from the owning MPLibrary.
    // typename MPTraits::MPTools* GetMPTools() const noexcept;

    ///@}
    ///@name Problem Accessors
    ///@{

    /// Get the library's current MPProblem.
    MPProblem* GetMPProblem() const noexcept;

    /// Get the current environment.
    Environment* GetEnvironment() const noexcept;

    /// Get the current task.
    MPTask* GetTask() const noexcept;

    /// Get the current group task.
    GroupTask* GetGroupTask() const noexcept;

    ///@}
    ///@name Solution Accessors
    ///@{

    MPSolutionType* GetMPSolution() const noexcept;

    /// Get the current free-space roadmap.
    RoadmapType* GetRoadmap(Robot* const _r = nullptr) const noexcept;

    /// Get the current free-space group roadmap.
    GroupRoadmapType* GetGroupRoadmap(RobotGroup* const _g = nullptr) const
        noexcept;

    /// Get the current obstacle-space roadmap.
    RoadmapType* GetBlockRoadmap(Robot* const _r = nullptr) const noexcept;

    // /// Get the current best path.
    Path* GetPath(Robot* const _r = nullptr) const noexcept;

    /// Get the current best group path.
    GroupPath* GetGroupPath(RobotGroup* const _g = nullptr) const noexcept;

    /// Get the current StatClass.
    StatClass* GetStatClass() const noexcept;

    /// Get the local obstacle map.
    LocalObstacleMap* GetLocalObstacleMap() const noexcept;

    /// Get the goal tracker.
    // GoalTracker* GetGoalTracker() const noexcept;

    ///@}

  protected:

    /// @param _s Class name
    void SetName(const std::string& _s) {m_name = _s;}

    /// @return base file name from MPProblem
    const std::string& GetBaseFilename() const;

    bool m_debug;                  ///< Print debug info?

  private:

    std::string m_name;            ///< Class name
    std::string m_label;           ///< Unique identifier.
    MPLibrary* m_library{nullptr}; ///< The owning MPLibrary.

    template<typename U> friend class MethodSet;
};

#endif
