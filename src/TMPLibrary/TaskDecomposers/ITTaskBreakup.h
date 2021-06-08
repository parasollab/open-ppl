#ifndef TASK_BREAKUP_H_
#define TASK_BREAKUP_H_

#include "TMPLibrary/TaskDecomposers/TaskDecomposerMethod.h"

class Robot;

class ITTaskBreakup : public TaskDecomposerMethod {
  public:
    ///@name Construction
    ///@{

    ITTaskBreakup();

    ITTaskBreakup(XMLNode& _node);

    ~ITTaskBreakup();

    ///@}
    ///@name Interface
    ///@{

    /// Breaks up whole task into subtasks at each robot interaction and stores
    /// the subtasks in the whole task's set
    virtual void BreakupTask(WholeTask* _wholeTask) override;

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    /// Makes the subtask for the given robot, start and goal
    std::shared_ptr<MPTask> MakeSubtask(Robot* _robot, Cfg _start, Cfg _goal, WholeTask* _wholeTask);

    ///@}
};

#endif
