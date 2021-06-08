#ifndef PMPL_AUCTION_METHOD_H_
#define PMPL_AUCTION_METHOD_H_

#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"
#include "TMPLibrary/WholeTask.h"

class AuctionMethod : public TaskAllocatorMethod {
  public:

    ///@name Construction
    ///@{

    AuctionMethod();

  AuctionMethod(XMLNode& _node);

  virtual ~AuctionMethod() = default;

    ///@}
    ///@name Call Method
    ///@{

  virtual void AllocateTasks() override;

  ///@}
  private:
    ///@name Helpers
    ///@{

    void AuctionSubtasks(WholeTask* _wholeTask);

    std::shared_ptr<MPTask> AuctionTask(std::shared_ptr<MPTask> _nextTask);

    /// Inserts the subtask into the unassignedTasks list at the appropriate point
     void AddSubtask(std::shared_ptr<MPTask> _subtask);

    ///@}
    ///@name Internal State
    ///@{

    std::list<std::shared_ptr<MPTask>> m_unassignedTasks;

    ///@}
};

/*----------------------------------------------------------------------------*/

#endif
