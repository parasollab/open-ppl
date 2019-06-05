#ifndef PMPL_COMBINED_ROADMAP_H_
#define PMPL_COMBINED_ROADMAP_H_

#include "TMPLibrary/TMPBaseObject.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include <iostream>

class StateGraph : public TMPBaseObject {
  public:

  	///@name Construction
    ///@{

  	StateGraph() = default;

		StateGraph(XMLNode& _node);

		virtual ~StateGraph() = default;  	

    ///@}
    ///@name Initialization
    ///@{

		virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

		RoadmapGraph<Cfg,DefaultWeight<Cfg>>* GetGraph();

    ///@}

  protected:

		///@name Helpers
		///@{
		
		virtual void ConstructGraph();

		///@}
		///@name member variables
		///@{

		RoadmapGraph<Cfg,DefaultWeight<Cfg>>* m_graph;
		
		///@}

};

/*----------------------------------------------------------------------------*/

#endif
