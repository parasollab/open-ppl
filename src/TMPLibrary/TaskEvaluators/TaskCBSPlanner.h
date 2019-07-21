#ifndef TASK_CBS_PLANNER_H_
#define TASK_CBS_PLANNER_H_

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"
#include "Utilities/CBS/NewConflict.h"
#include "Utilities/CBS/NewCBSTree.h"
#include "Utilities/CBS/TaskCBSNode.h"

class TaskCBSPlanner : public TaskEvaluatorMethod {
  public:

    ///@name Construction
    ///@{

    TaskCBSPlanner();

    TaskCBSPlanner(XMLNode& _node);

    virtual ~TaskCBSPlanner();

    ///}@
    ///@name Helpers
    ///@{

    virtual bool Run(std::vector<WholeTask*> _wholeTasks = {}, TaskPlan* _plan = nullptr) override;

    /// Adds new nodes that stem from a parent node to the CBS tree.
    std::vector<TaskCBSNode<WholeTask, OccupiedInterval>*>
    GrowTree(TaskCBSNode<WholeTask, OccupiedInterval>* _node,
        NewCBSTree<WholeTask, OccupiedInterval>& _tree,
				std::unordered_map<WholeTask*,NewConflict<OccupiedInterval>*> _conflictMap);

    /// Determines whether a node contains a conflict
    std::unordered_map<WholeTask*,NewConflict<OccupiedInterval>*>
    FindConflict(TaskCBSNode<WholeTask, OccupiedInterval>* _node);

    /// compares two intervals to determine if a conflict exists
    bool CompareIntervals(WholeTask* _task1, WholeTask* _task2,
        std::unordered_map<WholeTask*, std::list<OccupiedInterval>> _TIM,
        OccupiedInterval& _interval1, OccupiedInterval& _interval2);

    /// checks if two intervals can be reached by the same agent without
    /// conflictions
    bool CheckReachable(OccupiedInterval& _interval1,
        OccupiedInterval& _interval2);

    /// inserts conflict into the conflict map in order of interval start time
    /// and merges intervals as necessary.
    void InsertConflict(std::list<NewConflict<OccupiedInterval>*>& _conflicts,
        NewConflict<OccupiedInterval>* _newConflict);

    std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, Cfg _start, Cfg _goal);

    void FinalizeTaskPlan(TaskPlan* _plan);

    ///@}

  private:
    ///@name Internal State
    //@{

    std::string m_madLabel; ///< multi-agent dijkstra label

    ///@}

};


#endif
