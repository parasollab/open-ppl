/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_MIGRATABLE_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_MIGRATABLE_HPP

#include <stapl/containers/type_traits/distribution_traits.hpp>
#include <stapl/utility/hash.hpp>
#include <stapl/runtime.hpp>
#include <boost/bind/bind.hpp>
#include <boost/unordered_map.hpp>

namespace stapl {

namespace operations {


//////////////////////////////////////////////////////////////////////
/// @brief Operations class for container distributions that provide
/// element migration.
///
/// This concept requires that the derived distribution
/// has get_element.
/// The derived container manager must support add_element and remove_element.
/// The derived directory must be able to handle update.
///
/// The migration protocol is a 5-point protocol as explained in LCPC 2012:
///
/// @dot
/// digraph X {
///   "Source" -> "Home" [label="1"]
///   "Home" -> "Source" [label="2"]
///   "Source" -> "Destination" [label="3"]
///   "Destination" -> "Home" [label="4"]
///   "Home" -> "Destination" [label="5"]
/// }
/// @enddot
///
/// @tparam Derived The most derived distribution class
/// @todo Formally verify the correctness of this protocol.
////////////////////////////////////////////////////////////////////////
template<typename Derived>
class migratable
{
private:
  typedef Derived derived_type;

public:
  typedef typename distribution_traits<derived_type>::value_type value_type;
  typedef typename distribution_traits<derived_type>::gid_type   gid_type;

private:
  typedef stapl::hash<gid_type>                                  hash_type;

  /// Elements that are currently in quarantine on the source
  boost::unordered_map<gid_type, value_type, hash_type>          m_outgoing;

  /// Elements that are currently in quarantine on the destination
  boost::unordered_map<gid_type, value_type, hash_type>          m_incoming;

  //////////////////////////////////////////////////////////////////////
  /// @brief Cast this object to its most derived class. Used for CRTP
  //////////////////////////////////////////////////////////////////////
  derived_type& derived(void)
  {
    return static_cast<derived_type&>(*this);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Migrate a single element with a specific GID to a given location
  ///
  /// This is the first (intent) message in the migration protocol described
  /// in LCPC 2012.
  ///
  /// @param gid The GID to migrate
  /// @param destination The location to migrate to
  ////////////////////////////////////////////////////////////////////////
  void migrate(gid_type const& gid, const location_type destination)
  {
    stapl_assert(destination < derived().get_num_locations(),
                 "attempting to migrate to an unknown location");

    if (derived().container_manager().contains(gid)) {

      // drop the request if we're migrating to the same location
      if (destination == derived().get_location_id())
        return;

      // hold the element in the outgoing quarantine and remove it from base
      // container manager
      m_outgoing.insert(std::make_pair(gid, derived().get_element_impl(gid)));
      derived().container_manager().remove_element(gid);

      // inform the directory that this element is being migrated
      derived().directory().unregister_apply(
        gid,
        boost::bind(&derived_type::migrate_fwd, lazy_ref(derived()),
                    gid, derived().get_location_id(), destination));
    } else {
      derived().directory().invoke_where(
        std::bind(
          [](p_object& d, gid_type const& gid, location_type destination)
            { down_cast<derived_type&>(d).migrate(gid, destination); },
          std::placeholders::_1, std::placeholders::_2, destination),
        gid);
    }
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Executed on the home location to inform the source that it
  /// has received all requests in flight for a specific GID and it is
  /// now safe for it to begin transferring the data to the destination.
  ///
  /// This is the second (ack) message in the migration protocol described
  /// in LCPC 2012.
  ///
  /// @param gid The GID to migrate
  /// @param src The location that the GID is currently on
  /// @param dest The location that the GID is migrating to
  ////////////////////////////////////////////////////////////////////////
  void migrate_fwd(gid_type const& gid,
                   const location_type src, const location_type dest)
  {
    typedef void (derived_type::*fn)(gid_type const&, const location_type);
    async_rmi(src, derived().get_rmi_handle(), (fn)&derived_type::migrate_impl,
              gid, dest);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Perform the actual transfer of the element from the source
  /// to the destination. The data should be in quarantine initially.
  //
  /// This is the third (migrate) message in the migration protocol described
  /// in LCPC 2012.
  ///
  /// @param gid The GID to migrate
  /// @param dest The location that the GID is migrating to
  ////////////////////////////////////////////////////////////////////////
  void migrate_impl(gid_type const& gid, const location_type dest)
  {
    stapl_assert(m_outgoing.find(gid) != m_outgoing.end(),
                 "gid not in outgoing quarantine area during migration");
    typedef void (derived_type::*fn)(gid_type const&, value_type const&);
    async_rmi(dest, derived().get_rmi_handle(), (fn) &derived_type::store_hold,
              gid, m_outgoing[gid]);
    m_outgoing.erase(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Store the element in the incoming quarantine and inform the
  /// directory that this location has now received the element.
  //
  /// This is the fourth (update) message in the migration protocol described
  /// in LCPC 2012.
  ///
  /// @param gid The GID to migrate
  /// @param val The actual value that is being migrated
  ////////////////////////////////////////////////////////////////////////
  void store_hold(gid_type const& gid, value_type const& val)
  {
    m_incoming.insert(std::make_pair(gid, val));
    derived().directory().register_apply(
      gid,
      boost::bind(&derived_type::buffer_flushed_impl, lazy_ref(derived()),
                  gid, derived().get_location_id()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flush the buffer of queued requests at the directory for the
  /// element after we register the element on the new location.
  ///
  /// This is the fifth and final (flush) message in the migration protocol
  /// described in LCPC 2012.
  ///
  /// @param gid The GID to migrate
  /// @param dest The location that the GID is migrating to
  ////////////////////////////////////////////////////////////////////////
  void buffer_flushed_impl(gid_type const& gid, const location_type dest)
  {
    typedef void (derived_type::*fn)(gid_type const&);

    async_rmi(dest, derived().get_rmi_handle(),
              (fn)&derived_type::buffer_flushed, gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function that is executed to free the element from the quarantine
  /// in the final (flush) step of the migration protocol.
  ///
  /// @param gid The GID to migrate
  /// @see buffer_flushed_impl
  ////////////////////////////////////////////////////////////////////////
  void buffer_flushed(gid_type const& gid)
  {
    stapl_assert(m_incoming.find(gid) != m_incoming.end(),
                 "gid not in incoming quarantine area during migration");

    derived().container_manager().add_element(gid, m_incoming[gid]);

    m_incoming.erase(gid);
  }
};

} // namespace operations

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_MIGRATABLE_HPP

