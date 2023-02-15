/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/views/system_view.hpp>

namespace stapl {

distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, location_type, location_type>*
system_view(location_type nlocs)
{
  typedef distribution_spec_view<dist_view_impl::system_container,
            indexed_domain<location_type>, location_type, location_type> type;
  return new type(new dist_view_impl::system_container(nlocs),
    indexed_domain<location_type>(nlocs),
    identity_map<location_type, location_type, location_type, location_type>());
}


distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, location_type, location_type>*
system_view(std::vector<location_type> const& locs)
{
  typedef distribution_spec_view<dist_view_impl::system_container,
            indexed_domain<location_type>, location_type, location_type> type;
  return new type(new dist_view_impl::system_container(locs),
    indexed_domain<location_type>(locs.size()),
    identity_map<location_type, location_type, location_type, location_type>());
}


distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, location_type, location_type>*
system_view(level const lvl)
{
  typedef distribution_spec_view<dist_view_impl::system_container,
            indexed_domain<location_type>, location_type, location_type> type;
  return new type(new dist_view_impl::system_container(lvl),
    indexed_domain<location_type>(0),
    identity_map<location_type, location_type, location_type, location_type>());
}

}
