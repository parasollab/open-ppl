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
    Robot* m_robot; ///< Coordinator Robot

    bool m_debug{true};
    
};

#endif
