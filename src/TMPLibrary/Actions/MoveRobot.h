#ifndef MOVE_ROBOT_H_
#define MOVE_ROBOT_H_


#include "Action.h"

#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"

#include "Geometry/Boundaries/WorkspaceBoundingBox.h"

#include "MPLibrary/PMPL.h"

#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Action for moving a robot
////////////////////////////////////////////////////////////////////////////////


class MoveRobot : public Action {

  public:


    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> RoadmapType;

    ///@name Construction
    ///@{

    /// @param _robot Robot performing the move
    /// @param _start Start location of the move
    /// @param _goal Goal location of the move
    /// @param _hasObject Indiciates if the robot is moving with the task/object
    /// @param _roadmap Preloaded roadmap graph to use in feasibilty check
    /// @param _library Library to perform mp checks
    /// @param _manipulator Indicates type of robot team used
    //
    MoveRobot(Robot* _robot, const Boundary* _start, const Boundary* _goal, bool _hasObject,
                GenericStateGraph<Cfg, DefaultWeight<Cfg>>* _roadmap, MPLibrary* _library, bool _manipulator);

    virtual ~MoveRobot();

    ///@}
    ///@name Interface Functions
    ///@{
    virtual bool CheckPreConditions(const FactLayer* _factLayer) override;

    virtual std::vector<Robot*> GetRobots() override;

    virtual std::string PrintAction() override;

    ///@}

  private:

    /// Roadmap graph preloaded and used to check feasibility of move action
    GenericStateGraph<Cfg, DefaultWeight<Cfg>>* m_roadmapGraph{nullptr};

    Robot* m_robot{nullptr}; ///< Robot being moved

    const Boundary* m_start{nullptr}; ///< Start location of move action

    const Boundary* m_goal{nullptr}; ///< Goal location of move action

    bool m_hasObject{false}; ///< Indicates if the robot is moving with the task

    bool m_providedRoadmap{false};

    static size_t countChecked;
};

#endif
