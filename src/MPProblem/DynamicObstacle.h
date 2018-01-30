#ifndef DYNAMIC_OBSTACLE_H_
#define DYNAMIC_OBSTACLE_H_

#include "ConfigurationSpace/Cfg.h"
#include "Robot/Robot.h"

#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A model of a dynamic obstacle with a known trajectory.
////////////////////////////////////////////////////////////////////////////////
struct DynamicObstacle {

  ///@name Internal State
  ///@{

  std::unique_ptr<Robot> m_robot;
  std::vector<Cfg> m_path; ///< For now, assuming 1 cfg per time resolution.

  ///@}
  ///@name Construction
  ///@{

  DynamicObstacle(std::unique_ptr<Robot>&& _robot, std::vector<Cfg> _path) : m_robot(std::move(_robot)), m_path(_path){}

  ///@}

};

#endif
