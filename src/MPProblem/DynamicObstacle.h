#include "ConfigurationSpace/Cfg.h"
#include "Robot/Robot.h"

#include <vector>

struct DynamicObstacle {

  Robot m_robot;
  std::vector<Cfg> m_path;

  DynamicObstacle(Robot&& _robot, std::vector<Cfg> _path) : m_robot(std::move(_robot)), m_path(_path){}

};
