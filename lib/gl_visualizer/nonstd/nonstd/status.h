#ifndef NONSTD_STATUS_H_
#define NONSTD_STATUS_H_


namespace nonstd
{

  //////////////////////////////////////////////////////////////////////////////
  /// Describes the completion progress of a process or task.
  ///
  /// Note that this class is only meant for tracking the progress toward
  /// completion, and does not track errors. Error conditions are usually too
  /// complex to be handled by a simple flag indicating their presence and thus
  /// are not considered here.
  //////////////////////////////////////////////////////////////////////////////
  class status
  {

    public:

      ///@name Construction
      ///@{

      status();

      ~status();

      ///@}
      ///@name Queries
      ///@{

      bool is_started() const noexcept;  ///< Is the task started?
      bool is_on_hold() const noexcept;  ///< Is the task on hold?
      bool is_complete() const noexcept; ///< Is the task complete?

      ///@}
      ///@name Modifiers
      ///@{

      void start() noexcept;    ///< Mark the task as started.
      void hold() noexcept;     ///< Mark the task as on hold.
      void resume() noexcept;   ///< Mark the task as not on hold.
      void complete() noexcept; ///< Mark the task as complete.
      void reset() noexcept;    ///< Reset to the ready state.

      ///@}

    private:

      ///@name Local Types
      ///@{

      /// The possible completion states for a process/task.
      enum state {ready, in_progress, on_hold, completed};

      ///@}
      ///@name Internal State
      ///@{

      state m_state{ready};     ///< Current state of the task.

      ///@}

  };

}

#endif
