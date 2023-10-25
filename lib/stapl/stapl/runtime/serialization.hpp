/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_HPP
#define STAPL_RUNTIME_SERIALIZATION_HPP

#include "serialization_fwd.hpp"
#include "config.hpp"
#include "serialization/typer.hpp"

// core support
#include "serialization/typer_traits.hpp"
#include "serialization/basic.hpp"
#include "serialization/array.hpp"
#include "serialization/pointer.hpp"
#include "serialization/polymorphic.hpp"
#include "serialization/p_object.hpp"
#ifndef STAPL_DONT_USE_BOOST_SERIALIZATION
# include "serialization/boost_serialization.hpp"
#endif

// specific types
#include "serialization/bind.hpp"
#include "serialization/mem_fn.hpp"
#include "serialization/pair.hpp"
#include "serialization/reference_wrapper.hpp"
#include "serialization/shared_ptr.hpp"
#include "serialization/std_array.hpp"
#include "serialization/tuple.hpp"

#endif
