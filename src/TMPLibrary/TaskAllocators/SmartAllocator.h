#ifndef PMPL_SMART_ALLOCATOR_METHOD_H_
#define PMPL_SMART_ALLOCATOR_METHOD_H_


#include "MPLibrary/MPSolution.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"

#include "Traits/CfgTraits.h"

#include <iostream>

class SmartAllocator : public TaskAllocatorMethod {
  public:

    ///@name Local Types
    ///@{

    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;

    ///@}
    ///@name Construction
    ///@{

    SmartAllocator();

    SmartAllocator(XMLNode& _node);

    virtual ~SmartAllocator() = default;

    ///@}
    ///@name Call Method
    ///@{

    virtual void AllocateTasks();

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    /// Initialize the member variables
    void Initialize();

    /// Allocate an individual task
    void AllocateTask(vector<SemanticTask*> _semanticTasks);

    /// Save the allocation in the plan
    void SaveAllocation(Robot* _robot, SemanticTask* _task);

    /// Create a MPTask from cfgs
    std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, const Cfg& _start, const Cfg& _goal);

    /// Create a MPTask from cfg and constraint
    std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, const Cfg& _start, const Constraint*  _goal);

    ///@}
    ///@name Internal State
    ///@{

    /// Flag keeping track of initialization status
    bool m_initialized = false;

    std::string m_singleSolver;

    /// Local MPSolution to store roadmaps and query paths from
    std::unique_ptr<MPSolution> m_solution;

    /// Map tracking robot positions through allocation process
    std::map<Robot*,Cfg> m_currentPositions;

    ///@}
};

/*----------------------------------------------------------------------------*/

#endif
