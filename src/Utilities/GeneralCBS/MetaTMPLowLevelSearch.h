#ifndef META_TMP_LOW_LEVEL_SEARCH_
#define META_TMP_LOW_LEVEL_SEARCH_

#include "TMPLowLevelSearch.h"

class MetaTMPLowLevelSearch : public TMPLowLevelSearch {
	public:
		///@name Local Types
		///@{

			typedef std::map<SemanticTask*,AvailElem> MetaElem;

			struct MetaElemCompare {
				bool operator()(const MetaElem& _one, const MetaElem& _two) const {
					for(auto kv : _one) {
						auto& one = _one.at(kv.first);
						auto& two = _two.at(kv.first);
						if(one != two) {
							if(one.m_vid != two.m_vid) {
								return one.m_vid > two.m_vid;
							}
							else if(one.m_agent != two.m_agent) {
								return one.m_agent > two.m_agent;
							}
							else if(one.m_availInt.first != two.m_availInt.first) {
								return one.m_availInt.first > two.m_availInt.first;
							}
							else if(one.m_availInt.second != two.m_availInt.second) {
								return one.m_availInt.second > two.m_availInt.second;
							}
						}
					}
					return false;
				}
			};

			struct MetaElemGreaterThan {
				bool operator()(const std::pair<double,MetaElem>& _one, 
												const std::pair<double,MetaElem>& _two) const {
					return _one.first > _two.first;
				}
			};

		///@}
		///@name Construction
		///@{
		MetaTMPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel,
													std::string _vcLabel, bool _debug);

		~MetaTMPLowLevelSearch();
	
		///@}
		///@name
		///@{

		virtual bool UpdateSolution(GeneralCBSNode& _node, SemanticTask* _task) override;

		///@}
	private:

		///@name Helper Functions
		///@{

			void Initialize(GeneralCBSNode& _node, SemanticTask* _task, std::pair<size_t,size_t> _query);

			void Uninitialize();

			bool Search(GeneralCBSNode& _node, SemanticTask* _metaTask);

			std::vector<std::pair<double,MetaTMPLowLevelSearch::MetaElem>> ValidMetaNeighbors(
					MetaElem _current, SemanticTask* _task, size_t _vid, double _currentCost, double _edgeCost);

			void UpdateSolution(GeneralCBSNode& _node, std::vector<MetaElem> _plan);

		///@}
		///@name
		///@{

		std::map<MetaElem,MetaElem,MetaElemCompare>			m_parentMap;

		std::map<MetaElem,std::map<SemanticTask*,double>,MetaElemCompare>				m_distance;

		std::set<MetaElem,MetaElemCompare>							m_seen;

		std::set<MetaElem,MetaElemCompare> 							m_visited;

		std::map<MetaElem,std::unordered_map<Agent*,std::pair<double,Cfg>>,MetaElemCompare>		m_usedAgents;

		

		std::map<MetaElem,std::pair<std::vector<size_t>,size_t>,MetaElemCompare> m_execPathMap;
		std::map<MetaElem,std::pair<std::vector<size_t>,size_t>,MetaElemCompare> m_setupPathMap;

		std::map<MetaElem,std::map<MetaElem,std::pair<std::vector<size_t>,size_t>>,MetaElemCompare> m_reExecPathMap;

		///@}
};

#endif
