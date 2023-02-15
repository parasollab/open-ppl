/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_KRIPKE_KERNEL_COMMON_HEADERS_HPP
#define STAPL_KRIPKE_KERNEL_COMMON_HEADERS_HPP

#include <Kripke/Kernel.h>
#include <Grid.h>
#include <stapl/views/slices_view.hpp>
#include <stapl/views/metadata/coarseners/pg_aware_multiview_coarsener.hpp>
#include <stapl/skeletons/transformations/optimizers/nested.hpp>
#include <stapl/skeletons/transformations/optimizers/wavefront.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/reduce.hpp>
#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/nest.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/transformations/coarse.hpp>
#include "zip.hpp"
#include "wavefront.hpp"

#endif // STAPL_KRIPKE_KERNEL_COMMON_HEADERS_hpp
