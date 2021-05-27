#ifndef PMPL_COMBINED_ROADMAP_H_
#define PMPL_COMBINED_ROADMAP_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPLibrary/MPSolution.h"
#include "MPProblem/MPProblem.h"
#include "TMPLibrary/StateGraphs/StateGraph.h"
#include "Traits/CfgTraits.h"
#include "Utilities/Hypergraph.h"
#include <iostream>

class Interaction;

class CombinedRoadmap : public StateGraph {
  public:

    ///@name Local Types
    ///@{

    struct ActionUpdate {

    };

    typedef GroupLocalPlan<Cfg>                       GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType> GroupRoadmapType;
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

    ///@param _sr  The semantic roadmap to add the configuration too.
    ///@param _cfg The GroupCfg that is being added to the underlying 
    ///            Cspace roadmap.
    ///@return     The hypergraph vid of the added configuration.
    size_t AddConfiguration(SemanticRoadmap* _sr, GroupCfg _cfg);

    ///@param _csr The composite semantic roadmap that is being
    ///            connected with the interaction/
    ///@param _it  The interaction being added to the hypergraph.
    ///@return     The hypergraph vid of the new vertex for each
    ///            spawned semantic roadmap.
    std::set<size_t> AddInteraction(CompositeSemanticRoadmap _csr, 
                                    Interaction* _it);

    const TMPHypergraph* GetHypergraph() const;

    const std::set<SemanticRoadmap*>& GetSemanticRoadmaps() const;

    MPSolution* GetMPSolution() const;

    void AddRobotGroup(RobotGroup* _group);

    ///@}

  protected:

		///@name Helpers Functions
		///@{

		///@}
		///@name Internal State
		///@{

    /// Hypergraph representation of the combined roadmap
    std::unique_ptr<TMPHypergraph> m_hypergraph;

    /// Set of all semantic roadmaps contained in the hypergraph
    std::set<SemanticRoadmap*> m_semanticRoadmaps;

    /// MPSolution containing all roadmaps used in the combined roadmap
		std::unique_ptr<MPSolution> m_mpSolution;

		///@}

};

/*----------------------------------------------------------------------------*/

#endif
