#ifndef PMPL_COMBINED_ROADMAP_H_
#define PMPL_COMBINED_ROADMAP_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPLibrary/MPSolution.h"
#include "MPProblem/MPProblem.h"
#include "TMPLibrary/ActionSpace/Condition.h"
#include "TMPLibrary/StateGraphs/StateGraph.h"
#include "Traits/CfgTraits.h"
#include "Utilities/Hypergraph.h"
#include <iostream>

class Interaction;

class CombinedRoadmap : public StateGraph {
  public:

    ///@name Local Types
    ///@{

    typedef GroupLocalPlan<Cfg>                       GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType> GroupRoadmapType;
    typedef Condition::State                          State;

    struct ActionUpdate {
      /// Pairs of before and after changes.
      std::vector<std::pair<State,State>> updates;

      bool operator==(const ActionUpdate& _other) {

        if(updates.size() != _other.updates.size())
          return false;

        for(size_t i = 0; i < updates.size(); i++) {
          if(updates[i] != _other.updates[i])
            return false;
        }

        return true;
      }

      bool operator!=(const ActionUpdate& _other) {
        return !(*this == _other);
      }
    };

    typedef std::pair<GroupRoadmapType*,ActionUpdate> SemanticRoadmap;
    typedef std::set<SemanticRoadmap*>                CompositeSemanticRoadmap;

    struct TMPVertex {
      size_t rvid; ///< Roadmap vid
      SemanticRoadmap* sr;
 
      TMPVertex() {} 
      TMPVertex(size_t _rvid, SemanticRoadmap* _sr) : rvid(_rvid), sr(_sr) {}
    };

    struct TMPHyperarc {
      bool semantic{false};
      GroupLocalPlanType glp;
    };

    typedef Hypergraph<TMPVertex,TMPHyperarc> TMPHypergraph;
    typedef std::unordered_map<SemanticRoadmap*,std::unordered_map<size_t,size_t>> VertexMap; 
    
    typedef GroupRoadmapType::VI VI;
    typedef GroupRoadmapType::EI EI;

    ///@}
  	///@name Construction
    ///@{

  	CombinedRoadmap();

		CombinedRoadmap(XMLNode& _node);

		virtual ~CombinedRoadmap();  	

    ///@}
    ///@name Initialization
    ///@{

		virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    ///@param _csr   The composite semantic roadmap that is being
    ///              connected with the interaction.
    ///@param _input The initial state of the interaction, used to spawn
    ///              new semantic roadmaps.
    ///@param _output The final state of the interaction, used to spawn
    ///              new semantic roadmaps.
    ///@param _it    The interaction being added to the hypergraph.
    void AddInteraction(CompositeSemanticRoadmap _csr, State _input, 
                        State _output, Interaction* _it);

    TMPHypergraph* GetHypergraph();

    const std::set<SemanticRoadmap*>& GetSemanticRoadmaps() const;

    MPSolution* GetMPSolution() const;

    void AddRobotGroup(RobotGroup* _group);

    ///@}

  protected:

		///@name Helpers Functions
		///@{

    SemanticRoadmap* AddSemanticRoadmap(GroupRoadmapType* _grm, 
                                        const ActionUpdate& _update);

    void AddInteractionRoadmap(SemanticRoadmap* _sr);

    void CheckForGoalState(std::set<size_t> _hvids);

    ///@description Recursive build composite semantic roadmaps that cover all
    ///             robots in the problem with the robots in each semantic roadmap
    ///             being disjoint from the other semantic roadmaps.
    std::vector<CombinedRoadmap::CompositeSemanticRoadmap> BuildRobotGroups(
                  CompositeSemanticRoadmap _csr, std::set<Robot*> _robots, 
                  size_t _offset);

    ActionUpdate MergeActionUpdates(ActionUpdate _one, ActionUpdate _two);

    State ApplyActionUpdate(State _initial, ActionUpdate _update);

    bool IsGoalState(State _state);

    ///@param _sr  The semantic roadmap to add the configuration too.
    ///@param _vi  The vertex iterator that has been added to the underlying 
    ///            Cspace roadmap.
    ///@return     The hypergraph vid of the added configuration.
    size_t AddHypergraphVertex(SemanticRoadmap* _sr, VI _vi);

    ///@param _sr  The semantic roadmap to add the configuration too.
    ///@param _ei  The edge iterator that has been added to the underlying 
    ///            Cspace roadmap.
    ///@return     The hypergraph hid of the added hyperarc.
    size_t AddHypergraphArc(SemanticRoadmap* _sr, EI _ei);

    ///@param _original The cfg to be copied to the new roadmap.
    ///@param _newRoadmap The roadmap to copy the cfg into.
    size_t MoveGroupCfg(GroupCfg& _original, GroupRoadmapType* _newRoadmap);

    ///@param _originalSource the source of the edge to copy.
    ///@param _originalTarget the target of the edge to copy.
    ///@param _originalRoadmap The roadmap to copy the cfg from.
    ///@param _newRoadmap The roadmap to copy the cfg into.
    void MoveGroupEdge(size_t _originalSource, size_t _originalTarget, 
              size_t _newSource, size_t _newTarget, 
              GroupRoadmapType* _originalRoadmap, GroupRoadmapType* _newRoadmap);

		///@}
		///@name Internal State
		///@{

    /// Hypergraph representation of the combined roadmap.
    std::unique_ptr<TMPHypergraph> m_hypergraph;

    /// Set of all semantic roadmaps contained in the hypergraph.
    std::set<SemanticRoadmap*> m_semanticRoadmaps;

    /// Set of all interaction roadmaps contained in the hypergraph.
    std::set<SemanticRoadmap*> m_interactionRoadmaps;

    std::set<GroupRoadmapType*> m_cspaceRoadmaps;

    /// Map of roadmap vertices to hypergraph vertices for each semantic roadmap.
    VertexMap m_vertexMap;

    /// MPSolution containing all roadmaps used in the combined roadmap.
		std::unique_ptr<MPSolution> m_mpSolution;

    State m_initialState;

    size_t m_hookCounter{0};
		///@}

};

std::ostream& operator<<(std::ostream& _os, const CombinedRoadmap::TMPVertex& _vertex);
//std::istream& operator>>(std::istream& _is, TMPVertex& _vertex);

/*----------------------------------------------------------------------------*/

#endif
