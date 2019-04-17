#ifndef PMPL_CBS_TREE_H_
#define PMPL_CBS_TREE_H_
//#include "MPLibrary/Conflict.h"
#include "MPLibrary/MPBaseObject.h"
#include<queue>


template <typename MPTraits>
class CBSNode  : public MPBaseObject<MPTraits> {

	typedef typename MPTraits::CfgType    CfgType;
	public:
		//typedef typename MPTraits::Path Path;
		// typedef int Path;
		// typedef int Robot;
        /// Total cost of individual robot paths
		double m_cost;
		//std::vector<Path*> paths;
		//std::vector<Robot*> m_robots;
		//std::vector<Conflict> m_conflicts;

		std::vector<std::vector<size_t>> m_invalidVertices;

		std::vector<std::vector<pair<size_t,size_t>>> m_invalidEdges;

		std::vector<std::vector<pair<size_t,double>>> m_invalidVerticesAt;

		std::vector<std::vector<pair<pair<size_t,size_t>,double>>> m_invalidEdgesAt;

		std::vector<std::vector<pair<CfgType,double>>> m_conflictCfgsAt;

		void PrintInvalidEdges() {
			std::cout << "Invalid Edges: ";
			for(size_t i = 0 ; i < m_invalidEdges.size() ; ++i )
				for(size_t j = 0 ; j < m_invalidEdges[i].size() ; ++j ) {
					std::cout << "Robot " << i << ": " << "Edge: (" << m_invalidEdges[i][j].first 
					<< "," << m_invalidEdges[i][j].second << ")   ";
				}
			std::cout << std::endl;
		}

		// void ClearNode() {
		// 	for(size_t i = 0 ; i < m_invalidVertices.size() ; ++i) 
		// 		for(size_t j = 0 ; j < m_invalidVertices[i].size() ; ++j)
		// 			m_invalidVertices[i].clear()
		// 	m_invalidVertices.clear()
			

		// }
		//CBSNode* left;	
		//CBSNode* right;
};

template <typename MPTraits>
class LessThanByCost {
	//typedef typename MPTraits::CBSNode CBSNode;
	public:

    	bool operator()(const CBSNode<MPTraits>* _low, const CBSNode<MPTraits>* _high) {
     		return _low->m_cost  > _high->m_cost;
    	}
};

template <typename MPTraits>
class CBSTree {
	//typedef typename MPTraits::CBSNode CBSNode;
	//typedef typename MPTraits::LessThanByCost LessThanByCost;
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

		//CBSNode* root;
		//std::vector<CBSNode*> m_visitedNodes;
		std::priority_queue<CBSNode<MPTraits>*, vector<CBSNode<MPTraits>*> , LessThanByCost<MPTraits>> m_pq; 
};

// CBSTree::CBSTree() {}

// void CBSTree::insert(CBSNode* _leaf){
// 	m_pq.push(_leaf);
// }

// CBSNode* CBSTree::get_min_node() {

// 	CBSNode* min = m_pq.top();
// 	m_visitedNodes.push_back(min);
// 	m_pq.pop();
// 	return min;
// }

#endif
