#include "TMPLowLevelSearch.h"

TMPLowLevelSearch::
TMPLowLevelSearch(TMPLibrary* _tmpLibrary) : m_tmpLibrary(_tmpLibrary) {}

bool
TMPLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) {
	return true;
}
