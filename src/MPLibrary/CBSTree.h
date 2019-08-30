#ifndef PMPL_CBS_TREE_H_
#define PMPL_CBS_TREE_H_
#include "MPLibrary/MPBaseObject.h"
#include<queue>


template <typename MPTraits>
class CBSNode  : public MPBaseObject<MPTraits> {

	public:

		///@name Motion Planning Types
    ///@{

		typedef typename MPTraits::CfgType    					CfgType;
		typedef typename MPTraits::Path                 Path;
		typedef typename MPTraits::RoadmapType          RoadmapType;
		typedef typename RoadmapType::VID          			VID;
		
	  ///@}
    ///@name Local Types
    ///@{

		struct ConflictEdge{
      VID sourceVID;
      VID taregetVID;
      double timestep; 
    };

    struct ConflictCfg{
      CfgType conflictCfg;
      double timestep; 
    };

    ///@}
    ///@name Overrides
    ///@{

    bool operator<(const CBSNode& _high) const noexcept {
  		return m_cost < _high.m_cost;
		}

		///@}
    ///@name Internal State
    ///@{

		double m_cost; /// The total cost of individual robot paths

		std::vector<std::vector<ConflictEdge>> m_invalidEdgesAt;

		std::vector<std::vector<ConflictCfg>> m_conflictCfgsAt;

		///@}
};

template <typename MPTraits>
using CBSTree = std::priority_queue<CBSNode<MPTraits>>;

#endif
