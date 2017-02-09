#ifndef MP_BASE_OBJECT_H_
#define MP_BASE_OBJECT_H_

#include <iostream>
#include <string>
using namespace std;

#include "MPProblem/MPProblem.h"
#include "Utilities/IOUtils.h"
#include "Utilities/MethodSet.h"
#include "Utilities/XMLNode.h"

class Environment;
class MPTask;
class StatClass;

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
template <typename MPTraits>
class MPBaseObject {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::RoadmapType             RoadmapType;
    typedef typename MPTraits::Path                    Path;
    typedef typename MPTraits::MPLibrary               MPLibrary;

    typedef typename MPLibrary::SamplerPointer         SamplerPointer;
    typedef typename MPLibrary::LocalPlannerPointer    LocalPlannerPointer;
    typedef typename MPLibrary::ExtenderPointer        ExtenderPointer;
    typedef typename MPLibrary::PathModifierPointer    PathModifierPointer;
    typedef typename MPLibrary::ConnectorPointer       ConnectorPointer;
    typedef typename MPLibrary::MetricPointer          MetricPointer;
    typedef typename MPLibrary::MapEvaluatorPointer    MapEvaluatorPointer;
    typedef typename MPLibrary::MPStrategyPointer      MPStrategyPointer;
    typedef typename MPLibrary::DistanceMetricPointer  DistanceMetricPointer;
    typedef typename MPLibrary::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPLibrary::NeighborhoodFinderPointer
                                                       NeighborhoodFinderPointer;

    ///@}
    ///@name Construction
    ///@{

    /// Default constructor explicitly gives name, label, and debug.
    /// @param _label ID of the object, i.e., user defined label
    /// @param _name Name of the object, i.e., derived class name
    /// @param _debug Turn debug output on or off
    MPBaseObject(const string& _label = "", const string& _name = "",
        bool _debug = false);

    /// XML constructor pulls label and debug from an XML node.
    /// @param _node XMLNode to parse for this object
    MPBaseObject(XMLNode& _node);

    virtual ~MPBaseObject() = default;

    ///@}
    ///@name I/O
    ///@{

    /// Print internal state of this object.
    /// @param _os The std::ostream to print to.
    virtual void Print(ostream& _os) const;

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
    const string& GetName() const {return m_name;}

    /// Get the unique label for this object.
    const string& GetLabel() const {return m_label;}

    /// Get the unique string identifier for this object "m_name::m_label".
    string GetNameAndLabel() const;

    /// Set the unique label for this object.
    void SetLabel(const string&);

    ///@}
    ///@name MPLibrary Accessors
    ///@{

    /// Set the owning MPLibrary.
    void SetMPLibrary(MPLibrary*) noexcept;

    /// Get the owning MPLibrary.
    MPLibrary* GetMPLibrary() const noexcept;

    /// Get a distance metric by label from the owning MPLibrary.
    DistanceMetricPointer GetDistanceMetric(const string&) const noexcept;

    /// Get a validity checker by label from the owning MPLibrary.
    ValidityCheckerPointer GetValidityChecker(const string&) const noexcept;

    /// Get a neighborhood finder by label from the owning MPLibrary.
    NeighborhoodFinderPointer GetNeighborhoodFinder(const string&) const noexcept;

    /// Get a sampler by label from the owning MPLibrary.
    SamplerPointer GetSampler(const string&) const noexcept;

    /// Get a local planner by label from the owning MPLibrary.
    LocalPlannerPointer GetLocalPlanner(const string&) const noexcept;

    /// Get an extender by label from the owning MPLibrary.
    ExtenderPointer GetExtender(const string&) const noexcept;

    /// Get a path modifier by label from the owning MPLibrary.
    PathModifierPointer GetPathModifier(const string&) const noexcept;

    /// Get a connector by label from the owning MPLibrary.
    ConnectorPointer GetConnector(const string&) const noexcept;

    /// Get a metric by label from the owning MPLibrary.
    MetricPointer GetMetric(const string&) const noexcept;

    /// Get a map evaluator by label from the owning MPLibrary.
    MapEvaluatorPointer GetMapEvaluator(const string&) const noexcept;

    /// Get a strategy by label from the owning MPLibrary.
    MPStrategyPointer GetMPStrategy(const string&) const noexcept;

    ///@}
    ///@name Problem Accessors
    ///@{

    /// Get the library's current MPProblem.
    MPProblem* GetMPProblem() const noexcept;

    /// Get the current environment.
    Environment* GetEnvironment() const noexcept;

    /// Get the current query filename.
    string GetQueryFilename() const noexcept;

    /// Get the current task.
    MPTask* GetTask() const noexcept;

    ///@}
    ///@name Solution Accessors
    ///@{

    /// Get the current free-space roadmap.
    RoadmapType* GetRoadmap() const noexcept;

    /// Get the current obstacle-space roadmap.
    RoadmapType* GetBlockRoadmap() const noexcept;

    /// Get the current best path.
    Path* GetPath() const noexcept;

    /// Get the current StatClass.
    StatClass* GetStatClass() const noexcept;

    ///@}

  protected:

    /// @param _s Class name
    void SetName(const string& _s) {m_name = _s;}

    /// @return base file name from MPProblem
    const string& GetBaseFilename() const {
      return GetMPProblem()->GetBaseFilename();
    }

    bool m_debug;                  ///< Print debug info?

  private:

    string m_name;                 ///< Class name
    string m_label;                ///< Unique identifier.
    MPLibrary* m_library{nullptr}; ///< The owning MPLibrary.

    template<typename T, typename U> friend class MethodSet;
};

/*-------------------------------- Construction ------------------------------*/

template <typename MPTraits>
MPBaseObject<MPTraits>::
MPBaseObject(const string& _label, const string& _name, bool _debug) :
    m_debug(_debug), m_name(_name), m_label(_label) { }


template <typename MPTraits>
MPBaseObject<MPTraits>::
MPBaseObject(XMLNode& _node) {
  m_label = _node.Read("label", true, "", "Label Identifier");
  m_debug = _node.Read("debug", false, false, "Show run-time debug info?");
}

/*------------------------------------ I/O -----------------------------------*/

template <typename MPTraits>
void
MPBaseObject<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

/*-------------------------- Name and Label Accessors ------------------------*/

template <typename MPTraits>
inline
string
MPBaseObject<MPTraits>::
GetNameAndLabel() const {
  return m_name + "::" + m_label;
}


template <typename MPTraits>
inline
void
MPBaseObject<MPTraits>::
SetLabel(const string& _s) {
  m_label = _s;
}

/*----------------------------- MPLibrary Accessors --------------------------*/

template <typename MPTraits>
inline
void
MPBaseObject<MPTraits>::
SetMPLibrary(MPLibrary* _l) noexcept {
  m_library = _l;
}


template <typename MPTraits>
inline
typename MPTraits::MPLibrary*
MPBaseObject<MPTraits>::
GetMPLibrary() const noexcept {
  return m_library;
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::DistanceMetricPointer
MPBaseObject<MPTraits>::
GetDistanceMetric(const string& _label) const noexcept {
  return m_library->GetDistanceMetric(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::ValidityCheckerPointer
MPBaseObject<MPTraits>::
GetValidityChecker(const string& _label) const noexcept {
  return m_library->GetValidityChecker(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::NeighborhoodFinderPointer
MPBaseObject<MPTraits>::
GetNeighborhoodFinder(const string& _label) const noexcept {
  return m_library->GetNeighborhoodFinder(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::SamplerPointer
MPBaseObject<MPTraits>::
GetSampler(const string& _label) const noexcept {
  return m_library->GetSampler(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::LocalPlannerPointer
MPBaseObject<MPTraits>::
GetLocalPlanner(const string& _label) const noexcept {
  return m_library->GetLocalPlanner(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::ExtenderPointer
MPBaseObject<MPTraits>::
GetExtender(const string& _label) const noexcept {
  return m_library->GetExtender(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::PathModifierPointer
MPBaseObject<MPTraits>::
GetPathModifier(const string& _label) const noexcept {
  return m_library->GetPathModifier(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::ConnectorPointer
MPBaseObject<MPTraits>::
GetConnector(const string& _label) const noexcept {
  return m_library->GetConnector(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::MetricPointer
MPBaseObject<MPTraits>::
GetMetric(const string& _label) const noexcept {
  return m_library->GetMetric(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::MapEvaluatorPointer
MPBaseObject<MPTraits>::
GetMapEvaluator(const string& _label) const noexcept {
  return m_library->GetMapEvaluator(_label);
}


template <typename MPTraits>
inline
typename MPBaseObject<MPTraits>::MPStrategyPointer
MPBaseObject<MPTraits>::
GetMPStrategy(const string& _label) const noexcept {
  return m_library->GetMPStrategy(_label);
}

/*------------------------------ Problem Accessors ---------------------------*/

template <typename MPTraits>
inline
MPProblem*
MPBaseObject<MPTraits>::
GetMPProblem() const noexcept {
  return m_library->GetMPProblem();
}


template <typename MPTraits>
inline
Environment*
MPBaseObject<MPTraits>::
GetEnvironment() const noexcept {
  return GetMPProblem()->GetEnvironment();
}


template <typename MPTraits>
inline
string
MPBaseObject<MPTraits>::
GetQueryFilename() const noexcept {
  return GetMPProblem()->GetQueryFilename();
}


template <typename MPTraits>
inline
MPTask*
MPBaseObject<MPTraits>::
GetTask() const noexcept {
  return m_library->GetTask();
}

/*--------------------------- Solution Accessors -----------------------------*/

template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPBaseObject<MPTraits>::
GetRoadmap() const noexcept {
  return m_library->GetRoadmap();
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPBaseObject<MPTraits>::
GetBlockRoadmap() const noexcept {
  return m_library->GetBlockRoadmap();
}


template <typename MPTraits>
inline
typename MPTraits::Path*
MPBaseObject<MPTraits>::
GetPath() const noexcept {
  return m_library->GetPath();
}


template <typename MPTraits>
inline
StatClass*
MPBaseObject<MPTraits>::
GetStatClass() const noexcept {
  return m_library->GetStatClass();
}

/*----------------------------------------------------------------------------*/

#endif
