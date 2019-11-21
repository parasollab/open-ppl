#ifndef TMP_LOW_LEVEL_SEARCH_H_
#define TMP_LOW_LEVEL_SEARCH_H_

#include "LowLevelSearch.h"

class TMPLibrary;

class TMPLowLevelSearch : public LowLevelSearch {
  public:

	///@name Local Types
	///@{
	
	typedef std::pair<double,double> AvailInterval;
	typedef std::unordered_map<size_t,std::unordered_map<Agent*,std::vector<AvailInterval>>> IntervalMap;
	typedef std::pair<double,std::vector<size_t>> PathInfo;

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

	struct AvailElemHasher {
		std::size_t operator()(const AvailElem& _elem) const {
			return ((std::hash<size_t>()(_elem.m_vid)
							^ (std::hash<Agent*>()(_elem.m_agent) << 1)) >> 1)
							^ (std::hash<size_t>()(_elem.m_availInt.first) << 1)
							^ (std::hash<size_t>()(_elem.m_availInt.second));
		}
	};

	///@}
	///@name Construction
	///@{

	TMPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel);

	///@}
	///@name Interface
	///@{

	///@input _node contains the solution we are trying to update
	///@input _task is the task which has a new constraint and needs updating
	///@output bool indicating if there is a valid plan for the task being updated
	virtual bool UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) override;
	
	//@}

  private:

	///@name Helper Functions
	///@{
	
	void Initialize(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task);

	std::vector<AvailInterval> ComputeIntervals(GeneralCBSNode& _node, size_t vid, Agent* _agent); 

	void Uninitialize();

	std::vector<Assignment> Search(size_t _start, size_t _goal);

	std::vector<std::pair<double,AvailElem>> ValidNeighbors(const AvailElem& _elem, 
															size_t _vid, double _currentCost, double _edgeCost);

	PathInfo ComputeSetup(Agent* _agent, AvailInterval _avail, double _minTime, size_t _endVID);

	PathInfo ComputeExec(AvailElem& _elem, size_t _endVID);

	std::vector<Assignment> PlanDetails(std::vector<AvailElem> _plan);

	Assignment CreateAssignment(AvailElem _start, AvailElem _end);

	///@}
	///@name Internal State
	///@{

	TMPLibrary* m_tmpLibrary;

	std::string m_sgLabel;

	IntervalMap m_intervalMap;

	std::unordered_map<AvailElem,AvailElem,AvailElemHasher> m_parentMap;

	std::unordered_map<AvailElem,double,AvailElemHasher> m_distance;

	///@}

};

#endif
