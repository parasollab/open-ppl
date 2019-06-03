#ifndef PMPL_TMP_LIBRARY_H_
#define PMPL_TMP_LIBRARY_H_

#include "MPLibrary/MPLibrary.h"

#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"
#include "Utilities/TMPMethodSet.h"
#include "Utilities/XMLNode.h"

#include "Traits/TMPTraits.h"

#include "TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.h"
#include "TMPLibrary/StateGraphs/StateGraph.h"
#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/TaskDecomposers/TaskDecomposerMethod.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"
#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"
#include "TMPLibrary/TMPTools/TMPTools.h"

#include <algorithm>
#include <atomic>
#include <unordered_map>

////////////////////////////////////////////////////////////////////////////////
/// A collection of TMP planning algorithms that can operate on a specific
/// MPProblem comprised of MPTasks.
////////////////////////////////////////////////////////////////////////////////

class TMPLibrary {
  public:
  	///@name Local Types
    ///@{

    /// Solver represents an input set to MPLibraryType. It includes an
    /// MPStrategy label, seed, base file name, and vizmo debug option.
    struct Solver {
      std::string label;         ///< The XML label for the strategy to use.
      std::string baseFilename;  ///< The base name for output files.
      bool vizmoDebug;           ///< Save vizmo debug info?
    };

    ///@}
    ///@name Method Set Types
    ///@{
  	
		typedef TMPMethodSet<TMPStrategyMethod>        TMPStrategyMethodSet;
  	typedef TMPMethodSet<PoIPlacementMethod>       PoIPlacementMethodSet;
  	typedef TMPMethodSet<TaskEvaluatorMethod>      TaskEvaluatorMethodSet;
  	typedef TMPMethodSet<TaskDecomposerMethod>     TaskDecomposerMethodSet;
  	typedef TMPMethodSet<TaskAllocatorMethod>      TaskAllocatorMethodSet;
  	typedef TMPMethodSet<StateGraph>			   			 StateGraphSet;

    ///@}
    ///@name Method Pointer Types
    ///@{
    
		typedef typename TMPStrategyMethodSet::TMPMethodPointer     TMPStrategyMethodPointer;
  	typedef typename PoIPlacementMethodSet::TMPMethodPointer    PoIPlacementMethodPointer;
  	typedef typename TaskEvaluatorMethodSet::TMPMethodPointer   TaskEvaluatorMethodPointer;
  	typedef typename TaskDecomposerMethodSet::TMPMethodPointer  TaskDecomposerMethodPointer;
  	typedef typename TaskAllocatorMethodSet::TMPMethodPointer 	TaskAllocatorMethodPointer;
  	typedef typename StateGraphSet::TMPMethodPointer  			    StateGraphPointer;
		
		///@}
    ///@name Construction
    ///@{

    TMPLibrary();

    TMPLibrary(const std::string& _filename);

    ~TMPLibrary();

    ///@}
    ///@name Configuration
    ///@{

    /// Read an XML file to set the algorithms and parameters in this instance.
    /// @param _filename The XML file name.
    void ReadXMLFile(const std::string& _filename);

    ///@}
    ///@name TMPStrategyMethod Accessors
    ///@{

    /// Get a TMPStrategyMethod method 
    TMPStrategyMethodPointer GetTMPStrategy(const std::string& _l){
    	return m_tmpStrategies->GetMethod(_l);
    }

    void AddTMPStrategy(TMPStrategyMethodPointer _sm, const std::string& _l) {
    	m_tmpStrategies->AddMethod(_sm,_l);
    }

    ///@}
    ///@name PoIPlacementMethod Accessors
    ///@{

    /// Get a Point-of-Interest placement method 
    PoIPlacementMethodPointer GetPoIPlacementMethod(const std::string& _l){
    	return m_poiPlacementMethods->GetMethod(_l);
    }

    void AddPoIPlacementMethod(PoIPlacementMethodPointer _pm, const std::string& _l) {
    	m_poiPlacementMethods->AddMethod(_pm,_l);
    }

	///@}
    ///@name TaskEvaluator Accessors
    ///@{

    TaskEvaluatorMethodPointer GetTaskEvaluator(const std::string& _l){
    	return m_taskEvaluators->GetMethod(_l);
    }

    void AddTaskEvaluator(TaskEvaluatorMethodPointer _te, const std::string& _l) {
    	m_taskEvaluators->AddMethod(_te,_l);
    }

    ///@}
    ///@name Task Decomposition Accessors
    ///@{

    /// Get a TaskDecomposition 
    TaskDecomposerMethodPointer GetTaskDecomposer(const std::string& _l){
    	return m_taskDecomposers->GetMethod(_l);
    }

    void AddTaskDecomposer(TaskDecomposerMethodPointer _td, const std::string& _l) {
    	m_taskDecomposers->AddMethod(_td,_l);
    }

    ///@}
    ///@name Task ALlocator Accessors
    ///@{

    /// Get a TaskAllocator 
    TaskAllocatorMethodPointer GetTaskAllocator(const std::string& _l){
    	return m_taskAllocators->GetMethod(_l);
    }

    void AddTaskAllocator(TaskAllocatorMethodPointer _ta, const std::string& _l) {
    	m_taskAllocators->AddMethod(_ta,_l);
    }

    ///@}
    ///@name TMPTool Accessors
    ///@{

    /// Get the TMP tool container 
    TMPTools* GetTMPTools() {
    	return m_tmpTools;
    }

    ///@}
    ///@name Input Accessors
    ///@{

		MPLibrary* GetMPLibrary() const noexcept;
		void SetMPLibrary(MPLibrary* _l) noexcept;
    MPProblem* GetMPProblem() const noexcept;
    void SetMPProblem(MPProblem* const _problem) noexcept;

    std::vector<std::shared_ptr<MPTask>>& GetTasks() noexcept;
    void AddTask(MPTask* const _task) noexcept;
    void AddTask(std::shared_ptr<MPTask> const _task) noexcept;
    void ClearTasks();

    std::vector<std::shared_ptr<GroupTask>> GetGroupTasks() const noexcept;
    void AddGroupTask(GroupTask* const _task) noexcept;
    void AddGroupTask(std::shared_ptr<GroupTask> const _task) noexcept;
    void ClearGroupTasks();

    const std::string& GetBaseFilename() const noexcept;
    void SetBaseFilename(const std::string& _s) noexcept;

    ///@}
    ///@name Solution Accessors
    ///@{

    TaskPlan* GetTaskPlan(){
    	return m_taskPlan;
    }

    StateGraphPointer GetStateGraph(const std::string& _l){
    	return m_stateGraphs->GetMethod(_l);
    }
    void AddStateGraph(StateGraphPointer _sg, const std::string& _l){
    	m_stateGraphs->AddMethod(_sg,_l);
    }

		///@}
		///@name Execution
		///@{

		
		void Solve(MPProblem* _problem, std::vector<std::shared_ptr<MPTask>> _tasks, 
	  					 TaskPlan* _taskPlan);

		void Solve(MPProblem* _problem, std::vector<std::shared_ptr<MPTask>> _tasks);

		void Solve(MPProblem* _problem, std::vector<std::shared_ptr<GroupTask>> _tasks);

		void Solve(MPProblem* _problem, std::vector<std::shared_ptr<MPTask>> _tasks, 
							 TaskPlan* _taskPlan, const std::string& _label, const long _seed,
    					 const std::string& _baseFilename);

		void InitializeMPProblem(MPProblem* _problem);

		///@
		///@name Debug
		///@{

		void Print(std::ostream&) const;

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

    MPLibrary* 															m_library;	  ///< The underlying MPLibrary
  	MPProblem* 															m_problem;	  ///< The current MPProblem
  	std::vector<std::shared_ptr<MPTask>> 		m_tasks; 	  	///< Current set of tasks
  	std::vector<std::shared_ptr<GroupTask>> m_groupTasks; ///< Current set of group tasks
  	std::vector<Solver> 										m_solvers;    ///< Set of inputs to execute
		TMPTools*															  m_tmpTools;   ///< TMPTools container

  	///@}
  	///@name TMPMethod Sets
    ///@{
    /// Method sets hold and offer access to the tmp planning objects of the
    /// corresponding type.
  	TMPStrategyMethodSet*     m_tmpStrategies;
  	PoIPlacementMethodSet*    m_poiPlacementMethods;
  	TaskEvaluatorMethodSet*   m_taskEvaluators;
  	TaskDecomposerMethodSet*  m_taskDecomposers;
  	TaskAllocatorMethodSet*   m_taskAllocators;
  	StateGraphSet*  		  		m_stateGraphs;

  	///@}
  	///@name Solution
  	///@{

  	TaskPlan* m_taskPlan;   ///< Current task plan

  	///@}
};

/*---------------------------- Construction ----------------------------------*/

TMPLibrary::
TMPLibrary(){
  m_tmpStrategies = new TMPStrategyMethodSet(this,
  	typename TMPTraits::TMPStrategyMethodList(), "TMPStrategies");
  m_poiPlacementMethods = new PoIPlacementMethodSet(this,
  	typename TMPTraits::PoIPlacementMethodList(), "PoIPlacementMethods");
  m_taskAllocators = new TaskAllocatorMethodSet(this,
  	typename TMPTraits::TaskAllocatorMethodList(), "TaskAllocators");
  m_taskDecomposers = new TaskDecomposerMethodSet(this,
  	typename TMPTraits::TaskDecomposerMethodList(), "TaskDecomposers");
  m_taskEvaluators = new TaskEvaluatorMethodSet(this,
  	typename TMPTraits::TaskEvaluatorMethodList(), "TaskEvaluators");
}


TMPLibrary::
TMPLibrary(const std::string& _filename) : TMPLibrary() {
  m_library = new MPLibrary(_filename);
  ReadXMLFile(_filename);
}


TMPLibrary::
~TMPLibrary() {
  delete m_tmpStrategies;
  delete m_poiPlacementMethods;
  delete m_taskEvaluators;
  delete m_taskDecomposers;
  delete m_taskAllocators;
  delete m_stateGraphs;
}

void
TMPLibrary::
Initialize(){
  m_tmpStrategies->Initialize();
  m_poiPlacementMethods->Initialize();
  m_taskEvaluators->Initialize();
  m_taskDecomposers->Initialize();
  m_taskAllocators->Initialize();
  m_stateGraphs->Initialize();
}

void
TMPLibrary::
Uninitialize(){}

/*---------------------------- XML Helpers -----------------------------------*/

void
TMPLibrary::
ReadXMLFile(const std::string& _filename) {
  // Open the XML and get the root node.
  XMLNode tmpNode(_filename, "TaskAndMotionPlanning");

  // Find the 'TMPLibrary' node.
  XMLNode* tmpLibrary = nullptr;
  for(auto& child : tmpNode)
    if(child.Name() == "TMPLibrary")
      tmpLibrary = &child;

  // Throw exception if we can't find it.
  if(!tmpLibrary)
    throw ParseException(WHERE) << "Cannot find MPLibrary node in XML file '"
                                << _filename << "'.";

  // Parse the library node to set algorithms and parameters.
  for(auto& child : *tmpLibrary)
    ParseChild(child);

  // Ensure we have at least one solver.
  if(m_solvers.empty())
    throw ParseException(WHERE) << "Cannot find Solver node in XML file '"
                                << _filename << "'.";

  // Print XML details if requested.
  bool print = tmpNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  bool warnings = tmpNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    bool warningsAsErrors = tmpNode.Read("warningsAsErrors", false, false,
        "XML warnings considered errors");
    tmpLibrary->WarnAll(warningsAsErrors);
  }
}

bool
TMPLibrary::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "TMPStrategies") {
    m_tmpStrategies->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "PoIPlacementMethods") {
    m_poiPlacementMethods->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskEvaluators") {
    m_taskEvaluators->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskDecomposers") {
    m_taskDecomposers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskAllocators") {
    m_taskAllocators->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "StateGraphs") {
  	m_stateGraphs->ParseXML(_node);
  	return true;
  }
  else if(_node.Name() == "Solver") {
    const std::string label = _node.Read("tmpStrategyLabel", true, "",
        "The strategy to use.");

    const std::string baseFilename = _node.Read("baseFilename", true, "",
        "BaseFilename for the solver.");// + "." + std::to_string(seed);

    const bool vdOutput = _node.Read("vizmoDebug", false, false,
        "True yields VizmoDebug output for the solver.");

    m_solvers.emplace_back(Solver{label, baseFilename, vdOutput});
    return true;
  }
  else
    return false;
}

/*-------------------------------- Debugging ---------------------------------*/

void
TMPLibrary::
Print(ostream& _os) const {
  _os << "TMPLibrary" << std::endl;
  m_tmpStrategies->Print(_os);
  m_poiPlacementMethods->Print(_os);
  m_taskEvaluators->Print(_os);
  m_taskDecomposers->Print(_os);
  m_taskAllocators->Print(_os);
  m_stateGraphs->Print(_os);
}

/*----------------------------- Input Accessors ------------------------------*/

inline
MPLibrary*
TMPLibrary::
GetMPLibrary() const noexcept {
  return m_library;
}

inline
void
TMPLibrary::
SetMPLibrary(MPLibrary* _l) noexcept {
  m_library = _l;
}

inline
MPProblem*
TMPLibrary::
GetMPProblem() const noexcept {
  return m_problem;
}

void
TMPLibrary::
SetMPProblem(MPProblem* const _problem) noexcept {
  m_problem = _problem;
}

inline
std::vector<std::shared_ptr<MPTask>>& 
TMPLibrary::
GetTasks() noexcept {
  return m_tasks;
}

inline
void 
TMPLibrary::
AddTask(MPTask* const _task) noexcept {
  m_tasks.emplace_back(std::shared_ptr<MPTask>(_task));
}

inline
void 
TMPLibrary::
AddTask(std::shared_ptr<MPTask> const _task) noexcept{
  m_tasks.push_back(_task);
}
    
inline
void 
TMPLibrary::
ClearTasks(){
  m_tasks.clear();
}

inline
std::vector<std::shared_ptr<GroupTask>> 
TMPLibrary::
GetGroupTasks() const noexcept {
  return m_groupTasks;
}
  
inline  
void 
TMPLibrary::
AddGroupTask(GroupTask* const _task) noexcept {
  m_groupTasks.emplace_back(std::shared_ptr<GroupTask>(_task));
}

inline
void 
TMPLibrary::
AddGroupTask(std::shared_ptr<GroupTask> const _task) noexcept {
  m_groupTasks.push_back(_task);
}
   
inline 
void 
TMPLibrary::
ClearGroupTasks(){
  m_groupTasks.clear();
}

inline
const std::string& 
TMPLibrary::
GetBaseFilename() const noexcept{
  return m_problem->GetBaseFilename();
}

inline    
void 
TMPLibrary::
SetBaseFilename(const std::string& _s) noexcept {
  m_problem->SetBaseFilename(_s);
}

/*---------------------------- Solution Accessors ----------------------------*/

/*--------------------------- Execution Interface ----------------------------*/

void
TMPLibrary::
Solve(MPProblem* _problem, 
	  std::vector<std::shared_ptr<MPTask>> _tasks, 
	  TaskPlan* _taskPlan) {

  m_problem = _problem;
  m_tasks = _tasks;
  m_taskPlan = _taskPlan;

  for(auto& solver : m_solvers)
    RunSolver(solver);
}

void
TMPLibrary::
Solve(MPProblem* _problem, std::vector<std::shared_ptr<MPTask>> _tasks) {
  m_problem = _problem;
  m_tasks = _tasks;

  for(auto& solver : m_solvers) {
    // Create storage for the solution.
    m_taskPlan = new TaskPlan();

    RunSolver(solver);

    delete m_taskPlan;
  }

  m_taskPlan = nullptr;
}

void
TMPLibrary::
Solve(MPProblem* _problem, std::vector<std::shared_ptr<GroupTask>> _tasks) {
  m_problem = _problem;
  m_groupTasks = _tasks;

  for(auto& solver : m_solvers) {
    // Create storage for the solution.
    m_taskPlan = new TaskPlan();

    RunSolver(solver);

    delete m_taskPlan;
  }

  m_taskPlan = nullptr;
}

void
TMPLibrary::
Solve(MPProblem* _problem, 
	std::vector<std::shared_ptr<MPTask>> _tasks, 
	TaskPlan* _taskPlan,
    const std::string& _label, const long _seed,
    const std::string& _baseFilename) {

  m_problem = _problem;
  m_tasks = _tasks;
  m_taskPlan = _taskPlan;

  Solver s{_label, _baseFilename, false};
  RunSolver(s);
}

void
TMPLibrary::
InitializeMPProblem(MPProblem* _problem){
  SetMPProblem(_problem);
  Initialize();
}


void
TMPLibrary::
RunSolver(const Solver& _solver) {
  std::string originalBaseFilename = m_problem->GetBaseFilename();
  Initialize();

  // Call solver
  std::cout << "\n\nTMPLibrary is solving with TMPStrategyMethod labeled "
            << _solver.label << std::endl;

  // If this task has a label, append it to the solver's output file name.
  std::string baseFilename = _solver.baseFilename;

  if(!m_groupTasks.empty()) {
    if(!m_groupTasks.empty() and !m_groupTasks[0]->GetLabel().empty())
      baseFilename += "." + m_groupTasks[0]->GetLabel();
  }
  else if(!m_tasks.empty() and !m_tasks[0]->GetLabel().empty())
    baseFilename += "." + m_tasks[0]->GetLabel();

  // Remove spaces to keep file names nice.
  {
    auto newEnd = std::remove_if(baseFilename.begin(), baseFilename.end(),
        ::isspace);
    baseFilename.erase(newEnd, baseFilename.end());
  }

  SetBaseFilename(GetMPProblem()->GetPath(baseFilename));
  m_library->GetStatClass()->SetAuxDest(GetBaseFilename());

  // Initialize vizmo debug if there is a valid filename
  //if(_solver.vizmoDebug)
  //  VDInit(GetBaseFilename() + ".vd");

  GetTMPStrategy(_solver.label)->operator()();

  // Close vizmo debug if necessary
  //if(_solver.vizmoDebug)
  //  VDClose();

  Uninitialize();
  m_problem->SetBaseFilename(originalBaseFilename);
}

/*----------------------------------------------------------------------------*/

#endif
