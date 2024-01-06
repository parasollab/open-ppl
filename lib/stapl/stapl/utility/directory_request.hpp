/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_DIRECTORY_REQUEST_HPP
#define STAPL_UTILITY_DIRECTORY_REQUEST_HPP

#include <stapl/runtime/new.hpp>
#include <stapl/utility/empty_class.hpp>
#include <utility>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Abstract base representing encapsulation of a buffered directory
/// request which must be buffered at location managing a given key, pending
/// key registration.
///
/// @tparam Key Type of key managed by directory using this request type.
///
/// @tparam IntrusiveHook The directory can optionally use intrusive containers
/// (as defined in Boost.Intrusive) to reduce heap allocations.  If using
/// non-intrusive (i.e., STL), this type defaults to an empty base.
///
/// @ingroup directory
///
/// @sa directory
/// @sa directory_request
///
/// The directory heap allocates instances of concrete types implementing this
/// abstract class template and then erases the type back to it by storing
/// objects in a container of directory_request_base*. This allows the
/// directory to buffer arbitrarily typed functor requests until asynchronous
/// registration of a key is completed.
///
/// @todo For directories with a single or restricted set of possible requests
/// we can probably remove the virtual function / heap allocation costs, in
/// a manner similar to how std::function accomplishes this.
///
/// @todo Wrap the dynamic polymorphic object in copyable wrapper that
/// held a shared_ptr, auto_deleted.  Best for interoperability with a
/// directory which allowed one, template specified request type.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename IntrusiveHook = empty_class>
class directory_request_base
  : public IntrusiveHook
{
public:
  typedef Key key_type;

  directory_request_base(void) = default;

  directory_request_base(directory_request_base const&) = delete;
  directory_request_base& operator=(directory_request_base const&) = delete;

  virtual ~directory_request_base(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply functor (held as member of derived class which implements
  /// this method) directly to avoid RMI overhead.
  ///
  /// @param directory_ref Reference to the @ref p_object base of the directory
  ///   associated with this request.  Passed to the underlying functor as an
  ///   argument (optional use via bind()).
  /// @param key Key value which will be passed as parameter to functor
  /// invocation
  //////////////////////////////////////////////////////////////////////
  virtual void send_message_local(p_object& directory_ref,
                                  key_type const& key) = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Send message held in this buffered request to location where
  /// @p key is registered. Uses a RMI of Directory::execute.
  ///
  /// @param key     Key value the message is associated with.
  /// @param loc     Location where key is registered.
  /// @param handle  rmi_handle of directory.
  //////////////////////////////////////////////////////////////////////
  virtual void send_message_rmi(key_type const& key,
                                location_type loc,
                                rmi_handle::reference const& handle) = 0;
};


//////////////////////////////////////////////////////////////////////
/// @brief Derived class of @ref directory_request_base which holds function
/// object associated with request and also defines the pointer to member method
/// of the directory which will be used to service this request.
///
/// @tparam Directory Directory type employing instantiations of this request
/// template. Used to compute @p Key for base and obtain pointer to member
/// function.
///
/// @tparam Functor Function object directory is asked to invoke on a location
/// (i.e., the parameter passed to directory::invoke_where invocation).
///
/// @tparam IntrusiveHook Hook for intrusive container.
///
/// @ingroup directory
///
/// @sa directory_request_base
//////////////////////////////////////////////////////////////////////
template<typename Directory, typename Functor,
         typename IntrusiveHook = empty_class>
class directory_request
  : public directory_request_base<typename Directory::key_type, IntrusiveHook>,
    private Functor
{
public:
  typedef typename Directory::key_type key_type;

  template<typename F>
  directory_request(F&& f)
    : Functor(std::forward<F>(f))
  { }

  directory_request(directory_request const&) = delete;
  directory_request& operator=(directory_request const&) = delete;

  STAPL_USE_MANAGED_ALLOC(directory_request)

private:
  Functor& functor(void)
  {
    return static_cast<Functor&>(*this);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc directory_request_base::send_message_local(p_object&,key_type const&)
  ///
  /// @todo old code note suggests functor invocation should go through
  /// @ref directory::request.  Investigate reasoning behind this.
  //////////////////////////////////////////////////////////////////////
  void send_message_local(p_object& directory_ref, key_type const& key)
  {
    functor()(directory_ref, key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directory_request_base::send_message_rmi(p_object&,const location_type loc,rmi_handle::reference const&)
  //////////////////////////////////////////////////////////////////////
  void send_message_rmi(key_type const& key,
                        const location_type loc,
                        rmi_handle::reference const& handle)
  {
    typedef void (Directory::*mem_fun_t)(key_type const&, Functor&&);

    constexpr mem_fun_t mem_fun = &Directory::execute;

    typedef typename Directory::transmitter_type transmitter_type;

    transmitter_type::transmit(loc, handle, mem_fun, key, std::move(functor()));
  }
}; // struct directory_request

} // namespace detail

} // namespace stapl

#endif // STAPL_UTILITY_DIRECTORY_REQUEST_HPP
