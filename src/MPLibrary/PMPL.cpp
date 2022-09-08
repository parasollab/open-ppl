#include "PMPL.h"


// Explicitly instantiate the large template classes here.
template class MPLibraryType<PMPLTraits>;
template class MPSolutionType<PMPLTraits>;
template class GenericStateGraph<PMPLTraits::CfgType, PMPLTraits::WeightType>;
template class PathType<PMPLTraits>;
