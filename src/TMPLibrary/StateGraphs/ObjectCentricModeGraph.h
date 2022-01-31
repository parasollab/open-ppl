#ifndef PPL_OBJECT_CENTRIC_MODE_GRAPH_H_
#define PPL_OBJECT_CENTRIC_MODE_GRAPH_H_

#include "StateGraph.h"

#include "ConfigurationSpace/GenericStateGraph.h"
#include "MPProblem/Environment/Environment.h"
#include "TMPLibrary/ActionSpace/Interaction.h"

#include <map>
#include <unordered_map>

class ObjectCentricModeGraph : public StateGraph {

  public: 

    ///@name LocalTypes
    ///@{

    typedef Condition::State State;

    /// Key is the robot pointer for the object.
    /// Value is either the robot holding the object or the stable 
    /// placement region it is in.
    typedef std::map<Robot*,std::pair<Robot*,const Terrain*>> ObjectMode;

    /// Key is the robot pointer for the robot or object involved.
    /// Value is the Interaction (and reverse statues) the key is involved in and the role (string)
    /// it plays in the interaction.
    typedef std::map<Robot*,std::pair<std::pair<Interaction*,bool>,std::string>> ObjectModeSwitch;

    typedef GenericStateGraph<ObjectMode,ObjectModeSwitch> GraphType;
    typedef GraphType::VID                                 VID;
  
    ///@}
    ///@name Construction
    ///@{

    ObjectCentricModeGraph();

    ObjectCentricModeGraph(XMLNode& _node);

    ~ObjectCentricModeGraph();

    ///@}
    ///@name Interface
    ///@{

    void Initialize() override;

    void GenerateRepresentation(const State& _start);

    ///@}
    ///@name Accessors
    ///@{

    const GraphType* GetGraph() const;

    ///@}
    ///@name Debug
    ///@{

    void Print();

    ///@{

  private:

    ///@name Helper Functions
    ///@{

    ObjectMode GenerateInitialMode(const State& _start);

    void BuildModeGraph(ObjectMode& _initialMode);

    std::vector<std::vector<std::pair<Robot*,std::string>>> GetAllApplications(
                Interaction* _interaction, VID _source, bool _reverse = false);

    void ExpandApplications(
                   const std::vector<std::vector<std::pair<Robot*,std::string>>>& _roleCombos, 
                   std::vector<std::pair<ObjectModeSwitch,std::set<Robot*>>>& _outgoing, 
                   Interaction* _interaction, bool _reverse = false);

    void ApplyEdge(ObjectModeSwitch _edge, VID _source, std::set<VID>& _newModes);

    ///@}
    ///@name Internal State
    ///@{

    GraphType m_graph;
 
    std::unique_ptr<MPSolution> m_solution;

    std::unordered_map<const Terrain*, size_t> m_capacities;

    std::vector<Robot*> m_robots;
 
    ///@}

};

#endif
