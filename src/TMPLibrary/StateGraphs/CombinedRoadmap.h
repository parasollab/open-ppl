#ifndef PMPL_COMBINED_ROADMAP_H_
#define PMPL_COMBINED_ROADMAP_H_

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
		
		void ResetCapabilityRoadmaps();

		///@}
		///@name Construction Helpers
		///@{

	    virtual void ConstructGraph() override;

	    void GenerateITs();

    void FindITLocations(InteractionTemplate* _it);
    
		/// Transforms the ITs into the disovered locations
		void TransformITs();
    
		/// Initiallizes configurations for each capability at the start and end
    /// constriants of each whole task and adds them to the megaRoadmap
    void SetupWholeTasks();

		///@}
		///@name member variables
		///@{

    /// Map from each capability to the roadmap for that capability.
    std::unordered_map<std::string, std::shared_ptr<GraphType>> m_capabilityRoadmaps;

	/// The VIDs of the start and end points of the whole tasks in the
    /// megaRoadmap
    std::vector<std::vector<size_t>> m_wholeTaskStartEndPoints;
    
		double m_connectionThreshold{1.5};
		
		///@}

};

/*----------------------------------------------------------------------------*/

#endif
