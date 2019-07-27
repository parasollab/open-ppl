#ifndef OCCUPIED_INTERVAL_H_
#define OCCUPIED_INTERVAL_H__

#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Agents/HandoffAgent.h"
#include "MPProblem/MPTask.h"

#include <list>

class OccupiedInterval{
  public:

    ///@name Construction
    ///@{

    OccupiedInterval();

    OccupiedInterval(HandoffAgent* _r, Cfg _sL, Cfg _eL, double _sT, double _eT);

    OccupiedInterval(const OccupiedInterval& _other);

    ~OccupiedInterval();

    OccupiedInterval& operator=(const OccupiedInterval& _other);


    ///@}
    ///@name Accessors
    ///@{

    HandoffAgent* GetAgent();

    Cfg GetStartLocation();

    Cfg GetEndLocation();

    double GetStartTime();

    double GetEndTime();

    std::pair<Cfg, double> GetStart();

    std::pair<Cfg, double> GetEnd();

    std::shared_ptr<MPTask> GetTask();

    void SetStartTime(double _start);

    void SetEndTime(double _end);

    void SetStartLocation(Cfg _start);

    void SetEndLocation(Cfg _end);

    void SetStart(Cfg _startLoc, double _startTime);

    void SetEnd(Cfg _endLoc, double _endTime);

    void SetTask(std::shared_ptr<MPTask> _task);

    ///@}
    ///@name Helpers
    ///@{

    /// Checks if the time of the input interval overlaps with its own time interval.
    bool CheckTimeOverlap(OccupiedInterval _interval);

    /// Merges any intersecting intervals in the input list.
    static void MergeIntervals(std::list<OccupiedInterval*>& _intervals);

    /// Compares the start time of intervals.
    bool operator<(OccupiedInterval _interval);

    bool operator==(const OccupiedInterval& _interval) const;

    ///@}
    ///@name Debug
    ///@{

    std::string Print();

    ///@}
  private:

    ///@name Internal State
    ///@{

    HandoffAgent* m_agent{nullptr}; ///< HandoffAgent that is occupied for the interval.

    Cfg m_startLocation; ///< Physical start of the interval.

    Cfg m_endLocation; ///< Physical end of the interval.

    double m_startTime; ///< Start time of the interval.

    double m_endTime; ///< End time of the interval.

    /// Represents the motion task for the interval.
    std::shared_ptr<MPTask> m_task{nullptr};
    ///@}
};

#endif
