#ifndef PMPL_NEW_CBS_TREE_H_
#define PMPL_NEW_CBS_TREE_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/CBS/NewConflict.h"
#include "Utilities/CBS/NewCBSNode.h"

#include<queue>

/****************************** NewLessThanByCost *****************************/

template <typename T, typename U>
class NewLessThanByCost {
  //typedef typename MPTraits::CBSNode CBSNode;
  public:

    bool operator()(const NewCBSNode<T, U>* _low, const NewCBSNode<T, U>* _high) {
      return _low->GetCost(false)  > _high->GetCost(false);
    }
};

/******************************** NewCBSTree **********************************/

template <typename T, typename U>
class NewCBSTree {
  //typedef typename MPTraits::CBSNode CBSNode;
  //typedef typename MPTraits::LessThanByCost LessThanByCost;
  public:
    NewCBSTree(){}

    void Insert(NewCBSNode<T, U>* _leaf) {m_pq.push(_leaf);}

    NewCBSNode<T, U>* GetMinNode(){
      NewCBSNode<T, U>* min = m_pq.top();
      // m_visitedNodes.push_back(min);
      m_pq.pop();
      return min;
    }

    bool Empty() {return m_pq.empty();}

    size_t Length() {return m_pq.size();}

    //CBSNode* root;
    //std::vector<CBSNode*> m_visitedNodes;
    std::priority_queue<NewCBSNode<T, U>*, vector<NewCBSNode<T, U>*> ,
      NewLessThanByCost<T, U>> m_pq;
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

