#ifndef NEW_CBS_NODE_H_
#define NEW_CBS_NODE_H_

#include "Utilities/CBS/NewConflict.h"

template <typename T, typename U>
class NewCBSNode{
  public:
    ///@name Construction
    ///@{

    NewCBSNode();

    NewCBSNode(double _c, std::unordered_map<T*, std::list<NewConflict<U>*>>* _m, T* _t);

		NewCBSNode(const NewCBSNode<T,U>& _other);

		virtual ~NewCBSNode();
    /// Copy constructor

    NewCBSNode& operator=(const NewCBSNode& _other);

    ///@}
    ///@name Getters
    ///@{

    virtual double GetCost(bool _makespan = false) const;

    std::unordered_map<T*, std::list<NewConflict<U>*>>* GetConflicts();

    T* GetToReplan();

		bool HasReplanned();

    ///@}
    ///@name Setters
    ///@{

    void SetCost(double _c);

    void SetToReplan(T* _t);
	
		void SetReplanned(bool _hasReplanned);		

    ///@}
    ///@name Helpers
    ///@{

    /// Adds a new conflict to its associated task's conflicts.
    virtual void AddConflict(T* _t, NewConflict<U>* _c);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    double m_cost; ///< cost of current node

    T* m_toReplan; ///< task plan that needs to be replanned

    std::unordered_map<T*, std::list<NewConflict<U>*>>* m_conflicts; ///< conflict map

		bool m_hasReplanned{false};

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template<typename T, typename U>
NewCBSNode<T, U>::
NewCBSNode(){
  m_conflicts = new std::unordered_map<T*, std::list<NewConflict<U>*>>();
}

template<typename T, typename U>
NewCBSNode<T, U>::
NewCBSNode(double _c, std::unordered_map<T*, std::list<NewConflict<U>*>>* _m, T* _t) :
  m_cost(_c), m_conflicts(_m), m_toReplan(_t){
}

template <typename T, typename U>
NewCBSNode<T, U>::
~NewCBSNode(){
	if(m_conflicts)
		delete m_conflicts;
}

template <typename T, typename U>
NewCBSNode<T, U>::
NewCBSNode(const NewCBSNode<T,U>& _other){
  m_cost( _other.GetCost());
  m_conflicts = new std::unordered_map<T*, std::list<NewConflict<U>*>>();
  *m_conflicts = *(_other.m_conflicts);
}

template <typename T, typename U>
NewCBSNode<T, U>&
NewCBSNode<T, U>::
operator=(const NewCBSNode<T,U>& _other){
  m_cost = _other.GetCost();
	if(m_conflicts)
		delete m_conflicts;
	m_conflicts = new std::unordered_map<T*, std::list<NewConflict<U>*>>();
  *m_conflicts = *(_other.m_conflicts);
	return *this;
}

/*--------------------------------- Accessors --------------------------------*/

template <typename T, typename U>
double
NewCBSNode<T, U>::
GetCost(bool _makespan) const{
  return 0;
}

template <typename T, typename U>
std::unordered_map<T*, std::list<NewConflict<U>*>>*
NewCBSNode<T, U>::
GetConflicts(){
  return m_conflicts;
}

template <typename T, typename U>
T*
NewCBSNode<T, U>::
GetToReplan(){
  return m_toReplan;
}


template <typename T, typename U>
void
NewCBSNode<T, U>::
SetCost(double _c){
  m_cost = _c;
}

template <typename T, typename U>
void
NewCBSNode<T, U>::
SetToReplan(T* _t){
  m_toReplan = _t;
}

template <typename T, typename U>
bool
NewCBSNode<T, U>::
HasReplanned(){
  return m_hasReplanned;
}

template <typename T, typename U>
void
NewCBSNode<T, U>::
SetReplanned(bool _hasReplanned){
	m_hasReplanned = _hasReplanned;
}
/*--------------------------------- Helpers ----------------------------------*/

template <typename T, typename U>
void
NewCBSNode<T, U>::
AddConflict(T* _t, NewConflict<U>* _c){
  (*m_conflicts)[_t].push_back(_c);
}

#endif
