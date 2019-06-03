#ifndef HANDOFF_TASK_H_
#define HANDOFF_TASK_H_

#include "Action.h"

#include "Geometry/Boundaries/WorkspaceBoundingBox.h"

#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"

#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Action for moving a robot
////////////////////////////////////////////////////////////////////////////////


class HandoffTask : public Action {

  public:

    ///@name Construction
    ///@{

    /// @param _passer robot to initiate handoff
    /// @param _receiver robot to receive handoff
    /// @param _passingLocation locatino to initiate handoff
    /// @param _receivingLocation location to receive task in handoff
    /// @param _library Library used to perform mp checks
    HandoffTask(Robot* _passer, Robot* _receiver, const Boundary* _passingLocation,
                const Boundary* _receivingLocation, MPLibrary* _library);

    virtual ~HandoffTask();

    ///@}
    ///@name Interface Functions
    ///@{

    virtual bool CheckPreConditions(const FactLayer* _factLayer) override;

    virtual std::vector<Robot*> GetRobots() override;

    virtual std::string PrintAction() override;

    ///@}

  private:

    Robot* m_passer{nullptr}; ///< Robot that initially owns the task

    Robot* m_receiver{nullptr}; ///< Robot that is receiving the task

    MPTask* m_task{nullptr}; ///< Task that is being handed off

    const Boundary* m_passingLocation{nullptr}; ///< Location handoff is initiated at

    const Boundary* m_receivingLocation{nullptr}; ///< Location handoff is received at

};

#endif
