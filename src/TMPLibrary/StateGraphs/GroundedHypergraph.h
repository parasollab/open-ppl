#ifndef PPL_GROUNDED_HYPERGRAPH_H_
#define PPL_GROUNDED_HYPERGRAPH_H_

#include "StateGraph.h"

#include "ConfigurationSpace/Formation.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/ActionSpace/Action.h"

#include "Traits/CfgTraits.h"

#include "Utilities/Hypergraph.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>

class GroundedHypergraph : public StateGraph {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef size_t                                               VID;
    typedef TMPBaseObject::GroupCfgType                          GroupCfgType;
    typedef TMPBaseObject::GroupLocalPlanType                    GroupLocalPlanType;
    typedef TMPBaseObject::GroupRoadmapType                      GroupRoadmapType;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>>          GroupPathType;
    typedef PathType<MPTraits<Cfg,DefaultWeight<Cfg>>>           Path;
    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>     MPSolution;

    ///@}
    ///@name Local Types
    ///@{
   
    
    typedef std::vector<std::unique_ptr<Constraint>> PathConstraints;
    typedef std::vector<std::vector<std::shared_ptr<GroupTask>>> TransitionTaskSet;

    struct Transition {

      std::unordered_map<Robot*,std::vector<Cfg>> explicitPaths;
      std::unordered_map<Robot*,std::pair<double,std::pair<VID,VID>>> implicitPaths;
      std::pair<VID,VID> compositeImplicitPath;
      TransitionTaskSet taskSet;
      std::unordered_map<GroupTask*,std::unordered_set<Formation*>> taskFormations;
      double cost;

      bool operator==(const Transition& _other) const {
        return explicitPaths  == _other.explicitPaths
           and implicitPaths  == _other.implicitPaths
           and taskSet        == _other.taskSet
           and taskFormations == _other.taskFormations
           and cost           == _other.cost;
      }

      bool operator!=(const Transition& _other) const {
        return !(*this == _other);
      }

    };

    typedef std::pair<GroupRoadmapType*,VID>                     Vertex;
    typedef Hypergraph<Vertex,Transition>                        GH;
    typedef size_t                                               HID;
    typedef Action::State                                        State;

    ///@}
    ///@name Construction
    ///@{

    GroundedHypergraph();

    GroundedHypergraph(XMLNode& _node);

    virtual ~GroundedHypergraph();

    ///@}
    ///@name Interface
    ///@{

    void Initialize() override;

    void SetStartSet(const std::set<VID>& _startSet);

    bool ConnectTransition(const VID& _tail, const VID& _head, const PathConstraints& _pathConstraints={}, 
                           bool _bidirectional=false);

    void ConnectAllTransitions(const std::vector<VID>& _vertices, const PathConstraints& _pathConstraint={},
                               const bool& _bidirectional=false);

    bool ContainsSolution();

    ///@}
    ///@name Vertex Accessors
    ///@{

    VID AddVertex(const Vertex& _vertex);

    Vertex GetVertex(const VID& _vid); 

    ///@}
    ///@name Hyperarc Accessors
    ///@{

    HID AddTransition(const std::set<VID>& _tail, const std::set<VID>& _head, 
                      const Transition& _transition, const bool& _override=false);

    GH::Hyperarc GetHyperarc(const HID& _hid);

    HID GetHID(const std::set<VID>& _tail, const std::set<VID>& _head);

    Transition GetTransition(const HID& _hid);

    Transition GetTransition(const std::set<VID>& _tail, const std::set<VID>& _head);

    const std::set<HID> GetOutgoingHyperarcs(const VID& _vid);

    void Print();

    ///@}
    ///@name Miscellaneous Accessors
    ///@{

    GH::GraphType* GetReverseGraph();

    ///@}

  private:

    ///@name Helper Functions
    ///@{



    ///@}
    ///@name Internal State
    ///@{

    //std::unique_ptr<MPSolution> m_solution;

    std::unique_ptr<GH> m_hypergraph;

    std::set<VID> m_startSet;

    ///@}
    ///@name Parameters
    ///@{

    std::string m_queryStrategy;

    ///@}
};

std::ostream& operator<<(std::ostream& _os, const GroundedHypergraph::Vertex _vertex);
std::istream& operator>>(std::istream& _is, const GroundedHypergraph::Vertex _vertex);

std::ostream& operator<<(std::ostream& _os, const GroundedHypergraph::Transition _t);
std::istream& operator>>(std::istream& _is, const GroundedHypergraph::Transition _t);
#endif
