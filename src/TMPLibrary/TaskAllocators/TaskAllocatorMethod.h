#ifndef PMPL_TASK_ALLOCATOR_METHOD_H_
#define PMPL_TASK_ALLOCATOR_METHOD_H_

#include "MPLibrary/MPSolution.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "TMPLibrary/TMPBaseObject.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include <iostream>

class TaskAllocatorMethod : public TMPBaseObject {
  public:

    ///@name Construction
    ///@{

    TaskAllocatorMethod() = default;

    TaskAllocatorMethod(XMLNode& _node);

    virtual ~TaskAllocatorMethod() = default;

    ///@}
    ///@name Call Method
    ///@{

    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;

    virtual void AllocateTasks();

    void Initialize();

    void SaveAllocation(Robot* _robot, SemanticTask* _task,std::unique_ptr<Path>
        _path);

    bool m_initialized = false;

    /// Local MPSolution to store roadmaps and query paths from
    std::unique_ptr<MPSolution> m_solution;

    /// Map tracking robot positions through allocation process
    std::map<Robot*,Cfg> m_currentPositions;

    std::map<Robot*,double> m_nextFreeTime;

    bool m_clearAfterInitializing{false};

    ///@}
};

/*----------------------------------------------------------------------------*/

#endif
