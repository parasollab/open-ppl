#include "DynamicObstacle.h"

#include "Robot/Robot.h"


/*------------------------------- Construction -------------------------------*/

DynamicObstacle::
DynamicObstacle(std::unique_ptr<Robot>&& _robot, std::vector<Cfg> _path)
  : m_robot(std::move(_robot)), m_path(_path) {
}


DynamicObstacle::
~DynamicObstacle() = default;

/*-------------------------------- Accessors ---------------------------------*/

Robot*
DynamicObstacle::
GetRobot() const noexcept {
  return m_robot.get();
}


const std::vector<Cfg>
DynamicObstacle::
GetPath() const noexcept {
  return m_path;
}

/*----------------------------------------------------------------------------*/
