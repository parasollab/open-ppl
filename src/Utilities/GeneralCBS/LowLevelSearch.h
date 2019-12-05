#ifndef LOW_LEVEL_SEARCH_H_
#define LOW_LEVEL_SEARCH_H_

#include "TMPLibrary/TMPLibrary.h"
#include "Utilities/GeneralCBS/GeneralCBS.h"

class LowLevelSearch {
  public:

		///@name Local Types
		///@{

		typedef RoadmapGraph<Cfg,DefaultWeight<Cfg>> Roadmap;

    /// A set of conflicts for a single robot, keyed on timestep.
    typedef std::multimap<size_t, Cfg> MotionConstraintSet;

    /// A mapping from robot to conflict set.
    typedef std::map<Agent*, MotionConstraintSet> MotionConstraintMap;

		///@}
		///@name Construction
		///@{

		LowLevelSearch() = default;

		LowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel, bool _debug = false); 

		///@}
		///@name Interface
		///@{

		///@input _node contains the solution we are trying to update
		///@input _task is the task which has a new constraint and needs updating
		///@output bool indicating if there is a valid plan for the task being updated
		virtual bool UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task);
	
		virtual std::pair<double,std::vector<size_t>> MotionPlan(Cfg _start, Cfg _goal, 
																		double _startTime=0, double _minEndTime=0);
		//@}

  protected:	

		///@name Helper Functions
		///@{

  	/// Define a function for computing path weights w.r.t. multi-robot
    /// problems. Here the metric is the number of timesteps, and we return
    /// infinity if taking an edge would result in a inter-robot collision.
    /// @param _ei             An iterator to the edge we are checking.
    /// @param _sourceTimestep The shortest time to the source node.
    /// @param _bestTimestep   The best known time to the target node.
    /// @return The time to the target node via this edge, or infinity if taking
    ///         this edge would result in a collision with dynamic obstacles.
    double MultiRobotPathWeight(typename Roadmap::adj_edge_iterator& _ei,
        const double _sourceTimestep, const double _bestTimestep) const;

    /// Check if an edge is collision-free with respect to another robot's cfg.
    /// @param _source The cfg source of the edge.
    /// @param _target The cfg target of the edge.
    /// @param _conflictCfg The other robot's cfg to check.
    /// @return True if there is no collision between this robot traveling the
    ///         edge from _source to _target and the other robot at _conflictCfg.
    bool IsEdgeSafe(const size_t _source, const size_t _target,
        const Cfg& _conflictCfg) const;

		///@}
		///@name Internal State
		///@{

		TMPLibrary* m_tmpLibrary;

		std::string m_sgLabel;

		std::string m_vcLabel;

		bool m_debug;//TODO::add this to constructor

    /// The current set of conflicts to avoid.
    const MotionConstraintSet* m_currentMotionConstraints{nullptr};

		Robot* m_currentRobot{nullptr};

		MotionConstraintMap m_motionConstraintMap;
		///@}

};

#endif
