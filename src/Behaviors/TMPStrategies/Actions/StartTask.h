#ifndef START_TASK_H_
#define START_TASK_H_


#include <vector>
#include "Action.h"


////////////////////////////////////////////////////////////////////////////////
/// Action for moving a robot
////////////////////////////////////////////////////////////////////////////////


class StartTask : public Action {

  public:

    ///@name Construction
    ///@{

    /// @param _robot Robot that is goint to start the task
    /// @param _location Location that the task will be started at
    StartTask(Robot* _robot, const Boundary* _location);

    virtual ~StartTask();

    ///@}
    ///@name Interface Functions
    ///@{

    virtual bool CheckPreConditions(const FactLayer* _factLayer) override;

    virtual std::vector<Robot*> GetRobots() override;

    virtual std::string PrintAction() override;
    ///@}

  private:

    Robot* m_robot{nullptr}; ///< Robot that is starting the task

    const Boundary* m_location{nullptr}; ///< Location the task is started at
};

#endif
