#ifndef PARALLELSBMPHEADER_H_
#define PARALLELSBMPHEADER_H_

#include <iostream>
#include <stdlib.h>

#ifdef _PARALLEL
#include <runtime.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/algorithms/connected_components.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/hierarchical_view.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/views/list_view.hpp>
#include <stapl/views/repeated_view.hpp>

#include "ParallelSBMPUtils.h"
#endif

#endif
