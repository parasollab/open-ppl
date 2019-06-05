#ifndef PMPL_STATE_GRAPH_H_
#define PMPL_STATE_GRAPH_H_

#include "TMPLibrary/StateGraphs/StateGraph.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include <iostream>

class CombinedRoadmap : public StateGraph {
  public:

  	///@name Construction
    ///@{

  	CombinedRoadmap() = default;

		CombinedRoadmap(XMLNode& _node);

		virtual ~CombinedRoadmap() = default;  	

    ///@}
    ///@name Initialization
    ///@{

		virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    ///@}

  protected:

		///@name Helpers
		///@{
		
		virtual void ConstructGraph() override;

		///@}
		///@name member variables
		///@{

		
		///@}

};

/*----------------------------------------------------------------------------*/

#endif
