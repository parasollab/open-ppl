#ifndef PMPL_CBS_TREE_H_
#define PMPL_CBS_TREE_H_
#include "MPLibrary/MPBaseObject.h"
#include<queue>


template <typename MPTraits>
class CBSNode  : public MPBaseObject<MPTraits> {

	typedef typename MPTraits::CfgType    			CfgType;
	typedef typename MPTraits::Path                 Path;
	public:

		double m_cost; /// The total cost of individual robot paths

		std::vector<std::vector<size_t>> m_invalidVertices;

		std::vector<std::vector<pair<size_t,size_t>>> m_invalidEdges;

		std::vector<std::vector<pair<size_t,double>>> m_invalidVerticesAt;

		std::vector<std::vector<pair<pair<size_t,
			size_t>,double>>> m_invalidEdgesAt;

		std::vector<std::vector<pair<CfgType,double>>> m_conflictCfgsAt;

};

template <typename MPTraits>
class LessThanByCost {
	public:

    	bool operator()(const CBSNode<MPTraits>* _low, 
    		const CBSNode<MPTraits>* _high) {
     		return _low->m_cost  > _high->m_cost;
    	}
};

template <typename MPTraits>
class CBSTree {
	public:
		CBSTree(){}
		
		void Insert(CBSNode<MPTraits>* _leaf) {m_pq.push(_leaf);}
		
		CBSNode<MPTraits>* GetMinNode(){
			CBSNode<MPTraits>* min = m_pq.top();
			// m_visitedNodes.push_back(min);
			m_pq.pop();
			return min;
		}

		bool Empty() {return m_pq.empty();}

		size_t Length() {return m_pq.size();}

		std::priority_queue<CBSNode<MPTraits>*, 
			vector<CBSNode<MPTraits>*> , LessThanByCost<MPTraits>> m_pq; 
};

#endif
