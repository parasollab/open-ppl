#ifndef PMPL_TMP_TOOLS_H_
#define PMPL_TMP_TOOLS_H_

#include "TMPLibrary/TMPTools/MultiAgentDijkstra.h"

#include <iostream>

class TMPTools {
  public:

		///@name Local Types
		///@{

		template <typename Utility>
		using LabelMap = std::unordered_map<std::string, Utility*>;

  	///@}
		///@name Construction
    ///@{

  	TMPTools() = default;

		TMPTools(TMPLibrary* _tmpLibrary);

		virtual ~TMPTools() = default;  

		void ParseXML(XMLNode& _node);	

    ///@}
    ///@name Interaction Templates
    ///@{

		///@}
		///@name MultiAgentDijkstra
		///@{
		
		MultiAgentDijkstra* GetMultiAgentDijkstra(const std::string& _label);

		void SetMultiAgentDijkstra(const std::string& _label,
							MultiAgentDijkstra* _utility);

		///@}
    
	private:
	
		///@name Helpers
		///@{

		template <typename Utility>
		Utility* GetUtility(const std::string& _label,
				const LabelMap<Utility>& _map);

		template <typename Utility>
		void SetUtility(const std::string& _label, Utility* _utility,
				LabelMap<Utility>& _map);

		///@}
		///@name Internal State
		///@{

		TMPLibrary* const m_tmpLibrary; ///< The owning library.

		LabelMap<MultiAgentDijkstra> 		m_multiAgentDijkstras;
		LabelMap<InteractionTemplate>   m_interactionTemplates;

		///@}
};
/*----------------------------------------------------------------------------*/

#endif

