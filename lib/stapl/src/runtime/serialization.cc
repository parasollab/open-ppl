/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/config.hpp>

#ifdef STAPL_DONT_USE_BOOST_SERIALIZATION

# warning "Boost.Serialization Support is disabled"

#else

# define BOOST_ARCHIVE_SOURCE
# include <stapl/runtime/serialization.hpp>
# include <boost/archive/detail/archive_serializer_map.hpp>
# include <boost/archive/impl/archive_serializer_map.ipp>

using stapl::runtime::size_oarchive;
using stapl::runtime::buffer_oarchive;
using stapl::runtime::buffer_iarchive;

template class boost::archive::detail::archive_serializer_map<size_oarchive>;
template class boost::archive::detail::archive_serializer_map<buffer_oarchive>;
template class boost::archive::detail::archive_serializer_map<buffer_iarchive>;

#endif
