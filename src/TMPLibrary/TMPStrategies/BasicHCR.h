#ifndef PPL_BASIC_HRC_H_
#define PPL_BASIC_HRC_H_

#include "TMPStrategyMethod.h"

#include "TMPLibrary/ActionSpace/Condition.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"

class Action;
class Interaction;

class BasicHCR : public TMPStrategyMethod {
	public:
    ///@name Local Types
    ///@{

    typedef Condition::State                 State;
    typedef CombinedRoadmap::SemanticRoadmap SemanticRoadmap;
    typedef CombinedRoadmap::CompositeSemanticRoadmap 
            CompositeSemanticRoadmap;
    typedef std::set<std::string>            RoleSet;

    ///@} 
		///@name Construction
		///@{

		BasicHCR();

		BasicHCR(XMLNode& _node);

		~BasicHCR();

		///@}
		///@name Overrides
		///@{

    virtual void Initialize() override;
	
  	virtual void PlanTasks() override;       

	  ///@}

  private: 
    ///@name Helper Functions
    ///@{

    /// Sample a semantic roadmap from the current distribution
    ///@output The sampled semantic roadmap
    SemanticRoadmap* SampleSemanticRoadmap();

    /// Sample an interaction from the current distribution and
    /// then sample the interaction motions in the environment.
    ///@input _sr The semantic roadmap to connect to the interaction.
    ///@output bool If the interaction was successfully planned.
    bool SampleInteraction(SemanticRoadmap* _sr);

    /// Expand the underlying motion planning roadmap in the 
    /// semantic roadmap.
    ///@input _sr The semantic roadmap to expand.
    ///@output The addition nodes and edges in the underlying 
    ///        motion planning roadmap.
    void ExpandRoadmap(SemanticRoadmap* _sr);

    /// Searches for a valid state from which the input semantic
    /// roadmap can be connected to the input interaction. This
    /// may require evaluating other semantic roadmaps if the
    /// interaction requires it.
    ///@input _interaction The interaction to evaluate the pre-
    ///                    conditions of.
    ///@input _sr The semantic roadmap to include in the start
    ///           state.
    ///@output The composite semantic roadmap and the corresponding
    ///        state that satisfies the interaction preconditions.
    std::pair<CompositeSemanticRoadmap,State> FindStartState(
        Interaction* _interaction, SemanticRoadmap* _sr);

    std::vector<std::unordered_map<SemanticRoadmap*,size_t>> BuildCompositeSemanticRoadmaps(
      const std::unordered_map<SemanticRoadmap*,std::vector<RoleSet>>& _possibleRoles,
      RoleSet _totalRoles, RoleSet _satisfiedRoles, std::unordered_map<SemanticRoadmap*,size_t> _csr);


    // Stuff for proximity
    std::pair<CompositeSemanticRoadmap,State> FindStartState2(
        Interaction* _interaction, SemanticRoadmap* _sr);
   
    bool CheckCompositeStatesForMotionConstraints(Interaction* _interaction, State& _state);

    // Check if the previous function already dos this 
    std::vector<BasicHCR::CompositeSemanticRoadmap>
    BuildCompositeSemanticRoadmaps(CompositeSemanticRoadmap _csr, 
                  std::vector<std::vector<std::string>>& _types, size_t _offset);

    std::pair<CompositeSemanticRoadmap,State>
    CheckCompositeStatesForProximity(std::vector<SemanticRoadmap*> _csr, 
                                 std::vector<size_t>& _indices,size_t _csrIndex,
                                 Interaction* _interaction);
    ///@}
    ///@name Internal State
    ///@{

    std::unordered_map<Action*,double> m_interactionUtilityScore;

    std::unordered_map<SemanticRoadmap*,double> m_srUtilityScore;

    double m_expansionProbability;

    std::string m_mpStrategy;

    ///@}
};

#endif
