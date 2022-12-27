#ifndef PMPL_TMP_BASE_OBJECT_H_
#define PMPL_TMP_BASE_OBJECT_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPLibrary/PMPL.h"
#include "MPProblem/MPProblem.h"
#include "TMPLibrary/TMPLibrary.h"
#include "Utilities/IOUtils.h"
#include "Utilities/TMPMethodSet.h"
#include "Utilities/XMLNode.h"

class TMPStrategyMethod;
class PoIPlacementMethod;
class TaskEvaluatorMethod;
class TaskDecomposerMethod;
class TaskAllocatorMethod;
class StateGraph;
template<typename TMPMethod> class TMPMethodSet;
class TMPTools;

template <typename C, typename W>
class MPTraits;
class MPProblem;
class TMPLibrary;

////////////////////////////////////////////////////////////////////////////////
/// Abstract base class for all TMP algorithm abstractions in PMPL.
///
/// The TMPBaseObject carries a class name @c m_name and unique label @c m_label
/// for each algorithm. The name refers to the class from which the object was
/// instantiated, while the label refers to a specific instantiation of that
/// class.
///
/// All algorithms are owned by a TMPLibrary, which is referenced here as
/// @c m_tmpLibrary. When initially created, these objects will have no knowledge
/// of the MPProblem that they will be used on: they will only have access to
/// parameter settings provided in their XML nodes. Derived classes that have
/// problem-dependent internal state should override the @c Initialize() method
/// to set that data: this method will be called whenever the owning TMPLibrary's
/// current TMPProblem is changed.
////////////////////////////////////////////////////////////////////////////////
class TMPBaseObject {
  public:
    ///@name LocalTypes
    ///@{

    typedef GenericStateGraph<Cfg,DefaultWeight<Cfg>>     RoadmapType;

    typedef GroupCfg<RoadmapType>                         GroupCfgType;
    typedef GroupLocalPlan<RoadmapType>                   GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfgType,GroupLocalPlanType> GroupRoadmapType;

    typedef typename MPTraits<Cfg>::MPLibrary             MPLibrary;
    typedef typename MPTraits<Cfg>::MPSolution            MPSolution;

    ///@}
    ///@name Method Set Types
    ///@{

    typedef TMPMethodSet<TMPStrategyMethod>        TMPStrategyMethodSet;
    typedef TMPMethodSet<PoIPlacementMethod>       PoIPlacementMethodSet;
    typedef TMPMethodSet<TaskEvaluatorMethod>      TaskEvaluatorMethodSet;
    typedef TMPMethodSet<TaskDecomposerMethod>     TaskDecomposerMethodSet;
    typedef TMPMethodSet<TaskAllocatorMethod>      TaskAllocatorMethodSet;
    typedef TMPMethodSet<StateGraph>               StateGraphSet;

    ///@}
    ///@name Method Pointer Types
    
  	typedef typename TMPLibrary::InteractionStrategyMethodPointer InteractionStrategyMethodPointer;
  	typedef typename TMPLibrary::PoIPlacementMethodPointer        PoIPlacementMethodPointer;
  	typedef typename TMPLibrary::StateGraphPointer                StateGraphPointer;
  	typedef typename TMPLibrary::TaskAllocatorMethodPointer       TaskAllocatorMethodPointer;
  	typedef typename TMPLibrary::TaskDecomposerMethodPointer      TaskDecomposerMethodPointer;
  	typedef typename TMPLibrary::TaskEvaluatorMethodPointer       TaskEvaluatorMethodPointer;
		typedef typename TMPLibrary::TMPStrategyMethodPointer         TMPStrategyMethodPointer;
		
    ///@}
    ///@name Construction
    ///@{

    /// Default constructor explicitly gives name, label, and debug.
    /// @param _label ID of the object, i.e., user defined label
    /// @param _name Name of the object, i.e., derived class name
    /// @param _debug Turn debug output on or off
    TMPBaseObject(const std::string& _label = "", const std::string& _name = "",
        bool _debug = false);

    /// XML constructor pulls label and debug from an XML node.
    /// @param _node XMLNode to parse for this object
    TMPBaseObject(XMLNode& _node);

    virtual ~TMPBaseObject() = default;

    ///@}
    ///@name I/O
    ///@{

    /// Print internal state of this object.
    /// @param _os The std::ostream to print to.
    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name Initialization
    ///@{

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
    ///@name TMPLibrary Accessors
    ///@{

    /// Set the owning TMPLibrary.
    void SetTMPLibrary(TMPLibrary*) noexcept;

    /// Get the owning TMPLibrary.
    TMPLibrary* GetTMPLibrary() noexcept;

    /// Get a TMPStrategyMethod method from the owning TMPLibrary
    TMPStrategyMethodPointer GetTMPStrategy(const std::string&) const noexcept;

    /// Get a Point-of-Interest placement method from the owning TMPLibrary
    PoIPlacementMethodPointer GetPoIPlacementMethod(const std::string&) const noexcept;

    /// Get a TaskEvaluator from the owning TMPLibrary
    TaskEvaluatorMethodPointer GetTaskEvaluator(const std::string&) const noexcept;

    /// Get a TaskDecomposition from the owning TMPLibrary
    TaskDecomposerMethodPointer GetTaskDecomposer(const std::string&) const noexcept;

    /// Get a TaskAllocator from the owning TMPLibrary
    TaskAllocatorMethodPointer GetTaskAllocator(const std::string&) const noexcept;

    /// Get the TMP tool container from the TMPLibrary
    TMPTools* GetTMPTools() const noexcept;

    /// Get the TMP tool container from the TMPLibrary
    InteractionStrategyMethodPointer GetInteractionStrategyMethod(
                                      const std::string& _label) const noexcept;

    ///@}
    ///@name Problem Accessors
    ///@{

    /// Get the underlying MPLibrary
    MPLibrary* GetMPLibrary() const noexcept;

    /// Get the library's current TMPProblem
    MPProblem* GetMPProblem() const noexcept;

    ///@}
    ///@name Solution Accessors
    ///@{

    /// Get the current Plan
    Plan* GetPlan() const noexcept;

    /// Get the underlying StateGraph
    StateGraphPointer GetStateGraph(const std::string&) const noexcept;

    /// Get the library's mp solution object
    MPSolution* GetMPSolution() const noexcept;

    ///@}
  protected:

    /// @param _s Class name
    void SetName(const std::string& _s) {m_name = _s;}

    /// @return base file name from MPProblem
    const std::string& GetBaseFilename() const {
      return GetMPProblem()->GetBaseFilename();
    }

    bool m_debug;                  ///< Print debug info?

  private:

    std::string m_name;                ///< Class name
    std::string m_label;               ///< Unique identifier.
    TMPLibrary* m_tmpLibrary{nullptr}; ///< The owning MPLibrary.

    template<typename T> friend class TMPMethodSet;

};


/*----------------------------------------------------------------------------*/

#endif
