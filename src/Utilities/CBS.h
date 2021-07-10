#ifndef PPL_CBS_H_
#define PPL_CBS_H_

#include <functional>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>

template <typename IndividualTask, typename Constraint, typename IndividualSolution>
struct CBSNode {

  /// Map of task to its constraints  
  std::unordered_map<IndividualTask*,std::unordered_set<Constraint>> constraintMap;

  /// Map of task to its solution
  std::unordered_map<IndividualTask*,IndividualSolution*> solutionMap;

  /// Cost of current solution
  double cost;


  CBSNode(std::vector<IndividualTask*> _tasks) {
    for(auto& task : _tasks) {
      constraintMap[task] = {};
      solutionMap[task] = nullptr;
    }
  }

  CBSNode(const CBSNode& _node) {
    constraintMap(_node.constraintMap);
    solutionMap(_node.solutionMap);
    cost = _node.cost;
  }

  CBSNode& operator=(const CBSNode& _node) {
    if(this != &_node) {
      constraintMap.clear();
      constraintMap = _node.constraintMap;
      solutionMap.clear();
      solutionMap = _node.solutionMap;
      cost = _node.cost;
    }
    return *this;
  }

  CBSNode& operator=(CBSNode&& _node) {
    if(this != &_node) {
      constraintMap.clear();
      constraintMap = std::move(_node.constraintMap);
      solutionMap.clear();
      solutionMap = std::move(_node.solutionMap);
      cost = std::move(_node.cost);
    }
    return *this;
  }

  bool operator==(const CBSNode& _node) const {

    // Check if costs are equal
    if(cost != _node.cost)
      return false;

    for(auto& kv : constraintMap) {
      auto task = kv.first;
    
      // Check if other node has the task in the constraint map
      auto iter = _node.constraintMap.find(task);
      if(iter == _node.constraintMap.end())
        return false;

      // Check if constraint sets match
      if(kv.second != *iter) 
        return false;
    }

    for(auto& kv : solutionMap) {
      auto task = kv.first;
    
      // Check if other node has the task in the solution map
      auto iter = _node.solutionMap.find(task);
      if(iter == _node.solutionMap.end())
        return false;

      // Check if solutions match
      if(kv.second != *(iter->second)) 
        return false;
    }
  }

  bool operator>(const CBSNode& _node) const noexcept {
    return cost > _node.cost;
  }

};


template <typename IndividualTask, typename Constraint, typename IndividualSolution>
using CBSLowLevelPlanner = 
  std::function<std::vector<std::pair<IndividualTask,Constraint>>(
    CBSNode<IndividualTask,Constraint,IndividualSolution> _node,
    IndividualTask* _task)>;
  
template <typename IndividualTask, typename Constraint, typename IndividualSolution>
using CBSValidationFunction = 
  std::function<std::vector<std::pair<IndividualTask*,Constraint>>(
    CBSNode<IndividualTask,Constraint,IndividualSolution> _node)>;
  
template <typename IndividualTask, typename Constraint, typename IndividualSolution>
using CBSSplitNodeFunction =
  std::function<std::vector<std::pair<IndividualTask,Constraint>>(
    CBSNode<IndividualTask,Constraint,IndividualSolution> _node,
    std::vector<std::pair<IndividualTask*,Constraint>> _constraints,
    CBSLowLevelPlanner<IndividualTask,Constraint,IndividualSolution>& _lowlevel)>;
  
template <typename IndividualTask, typename Constraint, typename IndividualSolution>
using CBSCostFunction = 
  std::function<std::vector<std::pair<IndividualTask,Constraint>>(
    CBSNode<IndividualTask,Constraint,IndividualSolution> _node)>;

template <typename IndividualTask, typename Constraint, typename IndividualSolution>
CBSNode<IndividualTask,Constraint,IndividualSolution>
CBS(
  std::vector<IndividualTask*> _tasks,
  CBSValidationFunction<IndividualTask,Constraint,IndividualSolution>& _validate,
  CBSSplitNodeFunction<IndividualTask,Constraint,IndividualSolution>& _split,
  CBSLowLevelPlanner<IndividualTask,Constraint,IndividualSolution>& _lowlevel,
  CBSCostFunction<IndividualTask,Constraint,IndividualSolution>& _cost)
{
  
  using CBSNodeType = CBSNode<IndividualTask,Constraint,IndividualSolution>;

  // Create the conflict tree
  std::priority_queue<CBSNodeType,
                      std::vector<CBSNodeType>,
                      std::greater<CBSNodeType>> ct;

  // Create root node with initial plans
  CBSNodeType root;
  for(auto task : _tasks) {
    auto sol = _lowlevel(root,task);
    root.solutionMap[task] = sol;
  }

  ct.insert(root);

  // Search conflict tree
  while(!ct.empty()) {
    
    // Grab minimum cost node
    auto node = ct.top();
    ct.pop();
    
    // Validate solution in node
    auto constraints = _validate(node);

    // If there are no conflicts, return the valid solution
    if(constraints.empty())
      return node;

    // Create child nodes
    auto children = _split(node,constraints,_lowlevel);

    // Add child nodes to the tree
    for(const auto& child : children) {
      ct.insert(child);
    }
  }

  return CBSNodeType();
}

#endif
