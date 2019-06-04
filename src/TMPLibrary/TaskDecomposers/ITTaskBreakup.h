
#ifndef TASK_BREAKUP_H_
#define TASK_BREAKUP_H_

#include "TMPLibrary/TaskDecomposers/TaskDecomposerMethod.h"
#include "TMPLibrary/WholeTask.h"


class Robot;

class ITTaskBreakup : public TaskDecomposerMethod {
  public:
    ///@name Construction
    ///@{

		ITTaskBreakup(XMLNode& _node);

		ITTaskBreakup() = default;

    ITTaskBreakup(Robot* _robot);

    ~ITTaskBreakup();
    ///@}
    ///@name Interface
    ///@{

    /// Breaks up whole task into subtasks at each robot interaction and stores
    /// the subtasks in the whole task's set
    void BreakupTask(WholeTask* _wholeTask);

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    /// Makes the subtask for the given robot, start and goal
    std::shared_ptr<MPTask> MakeSubtask(Robot* _robot, Cfg _start, Cfg _goal, WholeTask* _wholeTask);

    ///@}

    Robot* m_robot; ///< Coordinator Robot

    bool m_debug{true};

};

#endif
