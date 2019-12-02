#include "MPLowLevelSearch.h"

MPLowLevelSearch::
MPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel) : 
									LowLevelSearch(_tmpLibrary,_sgLabel,_vcLabel) {}

bool
MPLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) {
	return true;
}
