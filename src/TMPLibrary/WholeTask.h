#ifndef WHOLE_TASK_H_
#define WHOLE_TASK_H_

#include <vector>
#include "Behaviors/Agents/HandoffAgent.h"
#include "MPProblem/MPTask.h"

struct WholeTask{

  std::shared_ptr<MPTask> m_task; ///< The original task to be split up
  std::vector<std::shared_ptr<MPTask>> m_subtasks; ///< split up tasks
  std::vector<HandoffAgent*> m_agentAssignment; ///< corresponding agents assigned to each subtask
  /// Maps of capabilities to cfgs/vids of start and end points for corresponding
  /// capability
  /// vectors of these so that they can be treated the same as stored roadmaps 
  /// in combined roadmap construction
  std::unordered_map<std::string, std::vector<Cfg>> m_startPoints;
  std::unordered_map<std::string, std::vector<size_t>> m_startVIDs;
  std::unordered_map<std::string, std::vector<Cfg>> m_goalPoints;
  std::unordered_map<std::string, std::vector<size_t>> m_goalVIDs;

	std::unordered_map<std::shared_ptr<MPTask>,std::pair<Cfg,Cfg>> m_subtaskStartEndCfgs;

  std::unordered_map<Cfg*,std::vector<Cfg>*> m_interactionPoints;

  std::unordered_map<std::shared_ptr<MPTask>,std::vector<Cfg>> m_interactionPathsDelivering;
  std::unordered_map<std::shared_ptr<MPTask>,std::vector<Cfg>> m_interactionPathsReceiving;

  size_t m_subtaskIterator{0}; ///< Keeps track of which subtasks have been performed

  std::vector<Cfg> m_wholePath; ///< Path across all capabilities solving m_task

};

#endif
