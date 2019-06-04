#ifndef STATE_H_
#define STATE_H_


#include "MPProblem/Robot/Robot.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"

class Action;

////////////////////////////////////////////////////////////////////////////////
/// States to be used in TMP methods
////////////////////////////////////////////////////////////////////////////////


struct State {

  public:

    ///@name Construction
    ///@{

    State();

    ~State();

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const State& _state) const;

    bool operator!=(const State& _state) const;
    
    ///@}
  
    State ApplyAction(std::shared_ptr<Action> _action);
  
  //private:

    ///@name Internal State
    ///@{

    bool m_debug{true};               ///< Toggle debug messages.
    
    ///@}

    std::unordered_map<Robot*, const Boundary*>  m_robotLocations;

    std::vector<const Boundary*>  m_objectLocations;

    std::vector<Robot*>     m_taskOwners;



};

#endif
