#ifndef TASK_BREAKUP_H_
#define TASK_BREAKUP_H_

#include "WholeTask.h"

class Robot;

class TaskBreakup {
  public:
    ///@name Construction
    ///@{
    TaskBreakup(Robot* _robot);

    ~TaskBreakup();
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
    std::shared_ptr<MPTask> MakeSubtask(Robot* _robot, Cfg _start, Cfg _goal);

    ///@}

    Robot* m_robot; ///< Coordinator Robot

    bool m_debug{true};

};

#endif
