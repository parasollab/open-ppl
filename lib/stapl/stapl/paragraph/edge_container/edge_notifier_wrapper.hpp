/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_NOTIFIER_WRAPPER_HPP
#define STAPL_PARAGRAPH_EDGE_NOTIFIER_WRAPPER_HPP

#include <stapl/paragraph/edge_container/edge_container.h>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Function object encapsulating notifier migration request
/// passed through the @ref directory.
///
/// Hand written instead of lambda / bind due to symbol size blowup
/// consistently seen during compilation.
//////////////////////////////////////////////////////////////////////
template<typename Notifier>
class remote_notifier_migrator
  : private Notifier
{
private:
  location_type m_location;

public:
  remote_notifier_migrator(location_type location, Notifier notifier)
    : Notifier(std::move(notifier)), m_location(location)
  { }

  void operator()(p_object& d, edge_container::index_type tid) const
  {
    down_cast<edge_container&>(d).add_remote_notifier<
      typename Notifier::value_type>(
        tid, m_location, static_cast<Notifier const&>(*this), false);
  }

  void define_type(typer& t)
  {
    t.base<Notifier>(*this);
    t.member(m_location);
  }
}; // class remote_notifier_migrator


//////////////////////////////////////////////////////////////////////
/// @brief Wrap remote edge notifier objects to support moving them to a new
///   location when the associated producer task is migrated.
/// @ingroup pgEdgeNotifiers
///
/// @tparam T Value type the edge this object is associated with represents.
////@tparam Notifier The underlying notifier this wrapper hold and should
/// invoke when migration is not requested.
///
/// @sa edge_remote_notifier
/// @sa edge_remote_signal_notifier
/// @sa edge_entry::add_remote_signal_notifier
/// @sa remote_notifier_creator
///
/// @todo Investigate if / why default constructor is necessary, as it
/// shouldn't be.
///
/// @todo This can be viewed as a generalization of std::function, where
/// we want to erase the concrete type of object but allow polymorphic
/// invocation of two methods (i.e., operator() and migrate().
/// Boost.TypeErasure (unofficial library) implements this.  If there are
/// other use cases, we could generalize this as utility.  Otherwise, this
/// class can be combined with @p Notifier and maybe make migration support a
/// statically enabled behavior.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Notifier>
struct edge_notifier_wrapper
  : private Notifier
{
private:
  using index_type     = edge_container::index_type;
  using stored_value_t = typename df_stored_type<T>::type;

public:
  using value_type = T;

  edge_notifier_wrapper(void) = default;

  explicit
  edge_notifier_wrapper(Notifier notifier)
    : Notifier(std::move(notifier))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Based on @p b_migrate parameter, either call
  /// @ref edge_container::add_remote_notifier to move notifier to producer
  /// task's new, post-migration execution location.  Otherwise, treat as an
  /// edge fire, and invoke the notifier, forwarding the appropriate parameters.
  ///
  /// @param edge_ct The edge container which holds the entry this notifier is
  ///                associated with.
  /// @param loc     The location that the remote notifier held by wrapper is
  ///                meant to notify.
  /// @param tid     The task identifier of the producer task the edge
  ///                notifier is associated with.
  /// @param value   The value to be flowed along this PARAGRAPH edge(s) this
  ///                notifier represents.
  /// @param b_migrate Boolean representing whether migration is requested by
  ///                the caller.
  //////////////////////////////////////////////////////////////////////
  void operator()(edge_container& edge_ct,
                  size_t loc, std::size_t tid,
                  edge_version_storage<stored_value_t> const& version,
                  bool b_migrate = false,
                  bool b_move    = false) const
  {
    if (b_migrate)
    {
      gang g(edge_ct);

      // Construct a functor that will invoke add_remote_notifier (parameterized
      // with above callback notifier.  This is passed off to the directory to
      // forward to the location where producer tid is going to execute.
      edge_ct.loc_directory().invoke_where(
        remote_notifier_migrator<edge_notifier_wrapper>(loc, *this), tid);
    }
    else
    {
      // ignore remote notifiers to self, could alternatively be done inside
      // notifier.
      if (edge_ct.get_location_id() == loc)
        return;

      gang g(edge_ct);

      static_cast<Notifier const&>(*this)
        (edge_ct.get_rmi_handle(), loc, tid, version, b_move);
    }
  }

  void define_type(typer& t)
  {
    t.base<Notifier>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for signal (i.e., void) consumption.  Avoid
/// move and sharing related logic.
//////////////////////////////////////////////////////////////////////
template<typename Notifier>
struct edge_notifier_wrapper<void, Notifier>
  : private Notifier
{
  typedef edge_container::index_type          index_type;
  typedef typename df_stored_type<void>::type stored_value_t;

public:
  typedef void                                      value_type;

  edge_notifier_wrapper(void) = default;

  explicit
  edge_notifier_wrapper(Notifier const& notifier)
    : Notifier(notifier)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Based on @p b_migrate parameter, either call
  /// @ref edge_container::add_remote_notifier to move notifier to producer
  /// task's new, post-migration execution location.  Otherwise, treat as an
  /// edge fire, and invoke the notifier, forwarding the appropriate parameters.
  ///
  /// @param edge_ct The edge container which holds the entry this notifier is
  ///                associated with.
  /// @param loc     The location that the remote notifier held by wrapper is
  ///                meant to notify.
  /// @param tid     The task identifier of the producer task the edge
  ///                notifier is associated with.
  /// @param value   The value to be flowed along this PARAGRAPH edge(s) this
  ///                notifier represents.
  /// @param b_migrate Boolean representing whether migration is requested by
  ///                the caller.
  //////////////////////////////////////////////////////////////////////
  void operator()(edge_container& edge_ct,
                  size_t loc, std::size_t tid,
                  edge_version_storage<int> const& version,
                  bool b_migrate = false,
                  bool b_move    = false) const
  {
    if (b_migrate)
    {
      gang g(edge_ct);

      // Construct a functor that will invoke add_remote_notifier (parameterized
      // with above callback notifier.  This is passed off to the directory to
      // forward to the location where producer tid is going to execute.
      edge_ct.loc_directory().invoke_where(
        remote_notifier_migrator<edge_notifier_wrapper>(loc, *this), tid);
    }
    else
    {
      // ignore remote notifiers to self, could alternatively be done inside
      // notifier.
      if (edge_ct.get_location_id() == loc)
        return;

      gang g(edge_ct);

      static_cast<Notifier const&>(*this)
        (edge_ct.get_rmi_handle(), loc, tid,
         edge_version_storage<void>(), b_move);
    }
  }

  void define_type(typer& t)
  {
    t.base<Notifier>(*this);
  }
}; // struct edge_notifier_wrapper<void, Notifier>

} // namespace detail

} // namesapce stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_NOTIFIER_WRAPPER_HPP
