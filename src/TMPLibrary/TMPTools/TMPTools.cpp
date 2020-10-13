#include "TMPTools.h"

TMPTools::
TMPTools(TMPLibrary* const _tmpLibrary) : m_tmpLibrary(_tmpLibrary) {}

void
TMPTools::
ParseXML(XMLNode& _node) {
	for(auto& child : _node){
		if(child.Name() == "MultiAgentDijkstra") {
			auto utility = new MultiAgentDijkstra(child);
			if(m_multiAgentDijkstras.count(utility->GetLabel()))
				throw ParseException(child.Where(), "Second MultiAgentDijkstra node with the"
					"label '" + utility->GetLabel() + "'. Labels must be unique.");
		
			SetMultiAgentDijkstra(utility->GetLabel(), utility);
		}
		if(child.Name() == "DiscreteMAD") {
			auto utility = new DiscreteMAD(child);
			if(m_discreteMADs.count(utility->GetLabel()))
				throw ParseException(child.Where(), "Second DiscreteMAD node with the"
					"label '" + utility->GetLabel() + "'. Labels must be unique.");
		
			SetDiscreteMAD(utility->GetLabel(), utility);
		}
	}
}
		
void 
TMPTools::
SetMultiAgentDijkstra(const std::string& _label, 
											MultiAgentDijkstra* _utility) {
	SetUtility(_label, _utility, m_multiAgentDijkstras);
}

MultiAgentDijkstra*
TMPTools::
GetMultiAgentDijkstra(const std::string& _label) {
	return GetUtility(_label, m_multiAgentDijkstras);
}

void 
TMPTools::
SetDiscreteMAD(const std::string& _label, 
											DiscreteMAD* _utility) {
	SetUtility(_label, _utility, m_discreteMADs);
}

DiscreteMAD*
TMPTools::
GetDiscreteMAD(const std::string& _label) {
	return GetUtility(_label, m_discreteMADs);
}

template <typename Utility>
void
TMPTools::
SetUtility(const std::string& _label, Utility* _utility,
					 LabelMap<Utility>& _map) {
	// Set the library pointer.
	_utility->SetTMPLibrary(m_tmpLibrary);

	// Check if this label is already in use.
	auto iter = _map.find(_label);
	const bool alreadyExists = iter != _map.end();
	
	// If the label alrady exists, we need to release the previous utility first.
	if(alreadyExists) {
		delete iter->second;
		iter->second = _utility;
	}
	else {
		_map.insert({_label, _utility});
	}
}

template<typename Utility>
Utility*
TMPTools::
GetUtility(const std::string& _label, const LabelMap<Utility>& _map) {
	try {
		return _map.at(_label);
	}
	catch(const std::out_of_range&){
		Utility dummy;
		throw RunTimeException(WHERE) << "Requested " << dummy.GetName() <<
														" '" << _label << "' does not exist.";
	}
	catch(const std::exception& _e) {
		Utility dummy;
		throw RunTimeException(WHERE) << "Error when fetching " << dummy.GetName() <<
														" '" << _label << "': " << _e.what();
	}
	catch(...) {
		Utility dummy;
		throw RunTimeException(WHERE) << "Error when fetching " << dummy.GetName() <<
														" '" << _label << "': (unkown)";
	}
}

