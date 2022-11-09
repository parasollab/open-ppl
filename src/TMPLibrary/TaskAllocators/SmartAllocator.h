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
    //void Initialize();

    /// Allocate an individual task
    void AllocateTask(vector<SemanticTask*> _semanticTasks);

    /// Save the allocation in the plan
    //void SaveAllocation(Robot* _robot, SemanticTask* _task,std::unique_ptr<Path> _path);

    /// Create a MPTask from cfgs
    std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, const Cfg& _start, const
        Cfg& _goal);

    /// Create a MPTask from cfg and constraint
    std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, const Cfg& _start, const
        Constraint*  _goal);

    std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, const Constraint*
        _start, const Constraint* _goal);

    void SubtractSmallest();
    void Munkres();
    void StarZeros();
    void CoverColumns();
    void PrimeZeros();
    void ConstructSeries();
    void AddAndSubtractValue();
    vector<int> FindZero(int row, int col);

    ///@}
    ///@name Internal State
    ///@{

    /// Flag keeping track of initialization status
    bool m_initialized = false;

    std::string m_solver;

    /// Local MPSolution to store roadmaps and query paths from
    //std::unique_ptr<MPSolution> m_solution;

    /// Map tracking robot positions through allocation process
    //std::map<Robot*,Cfg> m_currentPositions;

    //std::map<Robot*,double> m_nextFreeTime;

    //bool m_clearAfterInitializing{false};

    /// Cost matrix
    vector<vector<double>> m_costMatrix;

    /// Path matrix
    vector<vector<std::unique_ptr<Path>>> m_pathMatrix;

    /// Mask matrixm_
    vector<vector<double>> m_mask;
    int m_n;
    int m_m;
    int m_step =1;
    int m_row0;
    int m_col0;


    vector<int> m_rowCover;
    vector<int> m_colCover;

    ///@}
};

/*----------------------------------------------------------------------------*/

#endif
