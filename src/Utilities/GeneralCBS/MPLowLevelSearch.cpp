#include "MPLowLevelSearch.h"

MPLowLevelSearch::
MPLowLevelSearch(MPLibrary* _library) : m_library(_library) {}

bool
MPLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) {
	return true;
}
