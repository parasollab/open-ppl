#ifndef _PMPL_TASK_SOLUTION_H_
#define _PMPL_TASK_SOLUTION_H_

#include "MPLibrary/MPSolution.h"
#include "Traits/CfgTraits.h"

class Robot;
class RobotGroup;
class SemanticTask;

class TaskSolution {
  public:
    ///@name Local Types
    ///@{

    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;

    ///@}
    ///@name Construction
    ///@{

    TaskSolution(SemanticTask* _task);

    ~TaskSolution();

    ///@}
    ///@name Accessors
    ///@{

    SemanticTask* GetTask();

    void SetRobot(Robot* _robot);

    Robot* GetRobot();

    void SetRobotGroup(RobotGroup* _group);

    RobotGroup* GetRobotGroup();

    void SetMotionSolution(MPSolution* _solution);

    MPSolution* GetMotionSolution();

    void SetStartTime(double _startTime);

    double GetStartTime();

    ///@}
    ///@name Print
    ///@{

    void Print();

    ///@}

  private:
    ///@name Internal State
    ///@{

    SemanticTask* m_task;

    Robot* m_robot{nullptr};

    RobotGroup* m_robotGroup{nullptr};

    MPSolution* m_motionSolution{nullptr};

    double m_startTime{0};

    ///@}
};

#endif
