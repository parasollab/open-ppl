#ifndef TMP_LOW_LEVEL_SEARCH_H_
#define TMP_LOW_LEVEL_SEARCH_H_

#include "LowLevelSearch.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include <unordered_set>

class TMPLibrary;

class TMPLowLevelSearch : public LowLevelSearch {
  public:

		///@name Local Types
		///@{
	
		typedef std::pair<double,double> AvailInterval;
		typedef std::unordered_map<size_t,std::unordered_map<Agent*,std::vector<AvailInterval>>> IntervalMap;
		//typedef std::pair<double,std::pair<std::vector<size_t>,size_t>> PathInfo;

		struct AvailElem{
			size_t 				m_vid;
			Agent* 				m_agent;
			AvailInterval m_availInt;

			AvailElem() {}

			AvailElem(size_t _vid, Agent* _agent, AvailInterval _availInt) :
				m_vid(_vid), m_agent(_agent), m_availInt(_availInt) {}

			bool operator==(const AvailElem& _elem) const {
				return (m_vid == _elem.m_vid and
					 		  m_agent == _elem.m_agent and
					 		  m_availInt == _elem.m_availInt);
			}

			bool operator<(const AvailElem& _elem) const {
				return m_availInt.first < _elem.m_availInt.first;
			}

		};
		/*
		struct AvailElemHasher {
			std::size_t operator()(const AvailElem& _elem) const {
				return ((std::hash<size_t>()(_elem.m_vid))
								^ (std::hash<Agent*>()(_elem.m_agent))
								^ (std::hash<size_t>()(_elem.m_availInt.first))
								^ (std::hash<size_t>()(_elem.m_availInt.second)));
			}
		};
		*/
		///@}
		///@name Construction
		///@{

		TMPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel, bool _debug = false);

		///@}
		///@name Interface
		///@{

		///@input _node contains the solution we are trying to update
		///@input _task is the task which has a new constraint and needs updating
		///@output bool indicating if there is a valid plan for the task being updated
		virtual bool UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) override;
	
		virtual std::pair<double,std::pair<std::vector<size_t>,size_t>> MotionPlan(Cfg _start, Cfg _goal,
						double _startTime = 0, double _minEndTime = 0, SemanticTask* _currentTask = nullptr) override;

		bool CheckDeliveringAgentFutureConstraints(double _interactionTime, AvailElem _elem);
		//@}

  private:

		///@name Helper Functions
		///@{
	
		void Initialize(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task, std::pair<size_t,size_t> _query);

		std::vector<AvailInterval> ComputeIntervals(GeneralCBSNode& _node, size_t vid, 
														std::shared_ptr<SemanticTask> _task, Agent* _agent); 

		void Uninitialize();

		std::vector<Assignment> Search(std::shared_ptr<SemanticTask> _task, std::pair<size_t,size_t> _query);

		std::vector<std::pair<double,AvailElem>> ValidNeighbors(const AvailElem& _elem, 
															size_t _vid, double _currentCost, double _edgeCost);

		std::pair<double,std::pair<std::vector<size_t>,size_t>>
	 	ComputeSetup(AvailElem _elem, double _minTime, AvailElem _parent);

		std::pair<double,std::pair<std::vector<size_t>,size_t>>
		ComputeExec(AvailElem _elem, size_t _endVID, double _startTime);

		std::vector<Assignment> PlanDetails(std::vector<AvailElem> _plan, std::shared_ptr<SemanticTask> _task);

		Assignment CreateAssignment(AvailElem _start, AvailElem _end, std::shared_ptr<SemanticTask> _parentTask, double _endTime);

		bool CheckExec(AvailElem _elem, double _endTime, AvailElem _post);
		///@}
		///@name Internal State
		///@{

		IntervalMap m_intervalMap;
		/*
		std::unordered_map<AvailElem,AvailElem,AvailElemHasher> m_parentMap;

		std::unordered_map<AvailElem,double,AvailElemHasher> m_distance;

		std::unordered_set<AvailElem,AvailElemHasher> m_seen;

		std::unordered_set<AvailElem,AvailElemHasher> m_visited;

		std::unordered_map<AvailElem,AllocationConstraint,AvailElemHasher> m_availSourceMap;

		std::unordered_map<AvailElem,std::vector<size_t>,AvailElemHasher> m_execPathMap;
	
		std::unordered_map<AvailElem,std::vector<size_t>,AvailElemHasher> m_setupPathMap;

		std::unordered_map<AvailElem,double,AvailElemHasher> m_setupStartTimes;

		std::unordered_map<AvailElem,std::unordered_set<Agent*>,AvailElemHasher> m_usedAgents;
		*/
		std::map<AvailElem,AvailElem> m_parentMap;

		std::map<AvailElem,double> m_distance;
		
		std::set<AvailElem> m_seen;

		std::set<AvailElem> m_visited;

		std::map<AvailElem,AllocationConstraint> m_availSourceMap;

		std::map<AvailElem,AllocationConstraint> m_availEndMap;

		std::map<AvailElem,std::pair<std::vector<size_t>,size_t>> m_execPathMap;
	
		std::map<AvailElem,std::pair<std::vector<size_t>,size_t>> m_setupPathMap;

		std::map<AvailElem,double> m_setupStartTimes;

		std::map<AvailElem,std::unordered_set<Agent*>> m_usedAgents;

		std::map<AvailElem,std::map<AvailElem,std::pair<std::vector<size_t>,size_t>>> m_reExecPathMap;
		///@}
};

#endif
