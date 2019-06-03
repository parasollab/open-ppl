#ifndef PMPL_TMP_BASE_OBJECT_H_
#define PMPL_TMP_BASE_OBJECT_H_

#include "MPLibrary/MPLibrary.h"
#include "MPProblem/MPProblem.h"
#include "Utilities/IOUtils.h"
#include "Utilities/TMPMethodSet.h"
#include "Utilities/XMLNode.h"

class PoIPlacementMethod;
class StateGraph;
class TaskAllocatorMethod;
class TaskDecomposerMethod;
class TaskEvaluator;
class TMPStrategyMethod;
class TaskPlan;

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
    void SetTMPLibrary(MPLibrary*) noexcept;

    /// Get the owning TMPLibrary.
    TMPLibrary* GetTMPLibrary() const noexcept;

    /// Get a TMPStrategyMethod method from the owning TMPLibrary
    TMPStrategyMethod* GetTMPStrategyMethod(const std::string&) const noexcept;

    /// Get a Point-of-Interest placement method from the owning TMPLibrary
    PoIPlacementMethod* GetPoIPlacementMethod(const std::string&) const noexcept;

    TaskEvaluatorMethod* GetTaskEvaluator(const std::string) const noexcept;

    /// Get a TaskDecomposition from the owning TMPLibrary
    TaskDecomposerMethod* GetTaskDecomposer(const std::string&);

    /// Get a TaskAllocator from the owning TMPLibrary
    TaskAllocatorMethod* GetTaskAllocator(const std::string&);

    /// Get the TMP tool container from the TMPLibrary
    TMPTools* GetTMPTools() const noexcept;

    ///@}
    ///@name Problem Accessors
    ///@{

    /// Get the library's current TMPProblem
    MPProblem* GetMPProblem() const noexcept;

    ///@}
    ///@name Solution Accessors
    ///@{

    /// Get the current TaskPlan
    TaskPlan* GetTaskPlan();

    /// Get the underlying StateGraph
    StateGraph* GetStateGraph(const std::string&);

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

  	std::string m_name;            ///< Class name
    std::string m_label;           ///< Unique identifier.
    MPLibrary* m_library{nullptr}; ///< The owning MPLibrary.

    template<typename T, typename U> friend class MethodSet;

}

/*-------------------------------- Construction ------------------------------*/

TMPBaseObject::
TMPBaseObject(const std::string& _label, const std::string& _name, bool _debug) :
    m_debug(_debug), m_name(_name), m_label(_label) { }


TMPBaseObject::
TMPBaseObject(XMLNode& _node) {
  m_label = _node.Read("label", true, "", "Label Identifier");
  m_debug = _node.Read("debug", false, false, "Show run-time debug info?");
}

/*------------------------------------ I/O -----------------------------------*/

void
TMPBaseObject<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

/*-------------------------- Name and Label Accessors ------------------------*/

inline
const std::string&
TMPBaseObject::
GetName() const {
  return m_name;
}


inline
const std::string&
TMPBaseObject::
GetLabel() const {
  return m_label;
}


inline
std::string
TMPBaseObject::
GetNameAndLabel() const {
  return m_name + "::" + m_label;
}


inline
void
TMPBaseObject::
SetLabel(const std::string& _s) {
  m_label = _s;
}

/*----------------------------- TMPLibrary Accessors --------------------------*/

inline
void
TMPLibrary::
SetTMPLibrary(TMPLibrary* _l) noexcept {
	m_tmpLibrary = _l;
}

inline
TMPLibrary*
TMPLibrary::
GetTMPLibrary(){
	return m_tmpLibrary;
}

inline 
TMPStrategyMethod* 
TMPLibrary::
GetTMPStrategyMethod(const std::string& _label) const noexcept {
	return m_tmpLibrary->GetTMPStrategyMethod(_label);
}

inline 
PoIPlacementMethod* 
TMPLibrary::
GetPoIPlacementMethod(const std::string& _label) const noexcept {
	return m_tmpLibrary->GetPoIPlacementMethod(_label);
}

inline 
TaskEvaluatorMethod* 
TMPLibrary::
GetTaskEvaluator(const std::string& _label) const noexcept {
	return m_tmpLibrary->GetTaskEvaluator(_label);
}

inline 
TaskDecomposerMethod* 
TMPLibrary::
GetTaskDecomposer(const std::string& _label) const noexcept {
	return m_tmpLibrary->GetTaskDecomposer(_label);
}

inline 
TaskAllocatorMethod* 
TMPLibrary::
GetTaskAllocator(const std::string& _label) const noexcept {
	return m_tmpLibrary->GetTaskAllocator(_label);
}

inline 
TMPTools* 
TMPLibrary::
GetTMPTools(const std::string& _label) const noexcept {
	return m_tmpLibrary->GetTMPTools(_label);
}

/*------------------------------ Problem Accessors ---------------------------*/

inline
MPProblem*
TMPBaseObject::
GetMPProblem() const noexcept {
  return m_tmpLibrary->GetMPProblem();
}

/*--------------------------- Solution Accessors -----------------------------*/

inline
TaskPlan*
TMPBaseObject::
GetTaskPlan() const noexcept {
  return m_tmpLibrary->GetTaskPlan();
}

inline
StateGraph*
TMPBaseObject::
GetStateGraph() const noexcept {
  return m_tmpLibrary->GetStateGraph(std::string _label);
}