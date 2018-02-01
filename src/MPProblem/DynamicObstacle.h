#ifndef DYNAMIC_OBSTACLE_H_
#define DYNAMIC_OBSTACLE_H_

#include <memory>
#include <vector>

#include "ConfigurationSpace/Cfg.h"

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// A model of a dynamic obstacle with a known trajectory.
///
/// The obstacle body and motion properties are modeled as a robot. The
/// trajectory is interpreted as a sequence of configurations which the obstacle
/// will assume starting from time 0 and continuing at a rate of one Cfg per time
/// step.
////////////////////////////////////////////////////////////////////////////////
class DynamicObstacle {

  public:

    ///@name Construction
    ///@{

    /// Construct a dynamic obstacle.
    /// @param _robot The robot model for the obstacle.
    /// @param _path The obstacle's known trajectory.
    DynamicObstacle(std::unique_ptr<Robot>&& _robot, std::vector<Cfg> _path);

    ~DynamicObstacle();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the obstacle's robot model.
    Robot* GetRobot() const noexcept;

    /// Get the obstacle's path.
    const std::vector<Cfg> GetPath() const noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::unique_ptr<Robot> m_robot; ///< The obstacle robot.
    std::vector<Cfg> m_path; ///< For now, assuming 1 cfg per time resolution.

    ///@}

};

#endif
