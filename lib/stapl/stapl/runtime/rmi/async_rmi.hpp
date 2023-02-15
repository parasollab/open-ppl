/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_ASYNC_RMI_HPP
#define STAPL_RUNTIME_RMI_ASYNC_RMI_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../exception.hpp"
#include "../future.hpp"
#include "../instrumentation.hpp"
#include "../location_range.hpp"
#include "../primitive_traits.hpp"
#include "../rmi_handle.hpp"
#include "../tags.hpp"
#include "../yield.hpp"
#include "../request/async_rmi_request.hpp"
#include "../type_traits/transport_qualifier.hpp"
#include "../type_traits/type_id.hpp"
#include "../utility/unused.hpp"
#include <type_traits>
#include <utility>
#ifndef STAPL_RUNTIME_DISABLE_COMBINING
# include "../request/rmi_delegate.hpp"
#endif

namespace stapl {

namespace detail {

template<typename Handle, typename MemFun, typename... T>
void async_rmi_impl(runtime::context& ctx, unsigned int dest, Handle const& h,
                    MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");
  STAPL_RUNTIME_ASSERT_MSG(h.is_valid(dest),
                           "p_object does not exist in destination" );

  {
    STAPL_RUNTIME_PROFILE("async_rmi()", (primitive_traits::non_blocking |
                                          primitive_traits::ordered      |
                                          primitive_traits::p2p          |
                                          primitive_traits::comm));

    aggregator a{ctx, h, dest, no_implicit_flush};
    const bool on_shmem = a.is_on_shmem();

#ifndef STAPL_RUNTIME_DISABLE_COMBINING

    if (on_shmem) {
      typedef async_rmi_request<
                packed_handle_type,
                MemFun,
                typename transport_qualifier<decltype(t)>::type...
              > request_type;

      const combined_request_size size =
        request_type::expected_size(std::forward<T>(t)...);

      std::pair<void*, void*> p =
        a.allocate(rmi_delegate{get_type_id<request_type>(), h, pmf},
                   size.non_combined_size(),
                   size.combined_size());
      if (p.first) {
        // combine requests
        typedef typename request_type::args_type args_type;

        static_cast<request_type*>(p.first)->combined(size.combined_size());
        std::size_t sz = size.combined_static_size();
        unused(sz); // if sizeof...(T)==0, sz is unused
        new(p.second) args_type{std::forward_as_tuple(std::forward<T>(t),
                                                      p.second,
                                                      sz)...};
      }
      else {
        // no combining possible
        new(p.second) request_type{h, pmf, std::forward<T>(t)...};
      }
    }
    else {
      typedef async_rmi_request<
                packed_handle_type,
                MemFun,
                typename std::remove_reference<T>::type...
              > request_type;

      const combined_request_size size =
        request_type::expected_size(std::forward<T>(t)...);

      std::pair<void*, void*> p =
        a.allocate(rmi_delegate{get_type_id<request_type>(), h, pmf},
                   size.non_combined_size(),
                   size.combined_size());
      if (p.first) {
        //  combine requests
        typedef typename request_type::args_type args_type;

        static_cast<request_type*>(p.first)->combined(size.combined_size());
        std::size_t sz = size.combined_static_size();
        unused(sz); // if sizeof...(T)==0, sz is unused
        new(p.second) args_type{std::forward_as_tuple(std::forward<T>(t),
                                                      p.second,
                                                      sz)...};
      }
      else {
        // no combining possible
        new(p.second) request_type{h, pmf, std::forward<T>(t)...};
      }
    }

#else // STAPL_RUNTIME_DISABLE_COMBINING

    if (on_shmem) {
      typedef nc_async_rmi_request<
                packed_handle_type,
                MemFun,
                typename transport_qualifier<decltype(t)>::type...
              > request_type;
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{h, pmf, std::forward<T>(t)...};
    }
    else {
      typedef nc_async_rmi_request<
                packed_handle_type,
                MemFun,
                typename std::remove_reference<T>::type...
              > request_type;
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{h, pmf, std::forward<T>(t)...};
    }

#endif // STAPL_RUNTIME_DISABLE_COMBINING
  }

  scheduling_point(ctx);
}

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous RMI primitive.
///
/// The given member function is called on the object in the destination
/// location. Any return values are discarded.
///
/// This function supports combining: if two or more @ref async_rmi() calls are
/// made to the same object, function and location one after another, then the
/// requests are compressed and only the arguments of the requests past the
/// first are sent, saving on space and request execution time.
///
/// @param dest Destination location.
/// @param h    Handle to the target object.
/// @param pmf  Member function to invoke.
/// @param t    Arguments to pass to the member function.
///
/// @ingroup ARMIOneSided
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
void async_rmi(unsigned int dest, Handle const& h, MemFun const& pmf, T&&... t)
{
  runtime::context& ctx = runtime::this_context::get();

  detail::async_rmi_impl(ctx, dest, h, pmf, std::forward<T>(t)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous RMI primitive to all locations the object exists on.
///
/// The given member function is called on the object in all the locations it
/// exists on. Any return values are discarded.
///
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @ingroup ARMIOneSided
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
void async_rmi(all_locations_t, Handle const& h, MemFun const& pmf,  T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  {
    STAPL_RUNTIME_PROFILE("async_rmi(all_locations)",
                          (primitive_traits::non_blocking |
                           primitive_traits::ordered      |
                           primitive_traits::p2m          |
                           primitive_traits::comm));


    bcast_aggregator a{ctx, h};
    typedef nc_async_rmi_request<
              packed_handle_type,
              MemFun,
              typename std::remove_reference<T>::type...
            > request_type;
    const std::size_t size =
      request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{h, pmf, std::forward<T>(t)...};
  }

  scheduling_point(ctx);
}


namespace unordered {

//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous RMI primitive to all locations the object exists on.
///
/// The given member function is called on the object in all the locations it
/// exists on. Any return values are discarded.
///
/// This is an unordered version of the @ref stapl::async_rmi() that may break
/// RMI ordering rules.
///
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @ingroup ARMIUnordered
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
void async_rmi(all_locations_t, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  {
    STAPL_RUNTIME_PROFILE("unordered::async_rmi(all_locations)",
                          (primitive_traits::non_blocking |
                           primitive_traits::unordered    |
                           primitive_traits::p2m          |
                           primitive_traits::comm));

    bcast_aggregator a{ctx, h, false};
    typedef nc_async_rmi_request<
              packed_handle_type,
              MemFun,
              typename std::remove_reference<T>::type...
            > request_type;
    const std::size_t size =
      request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{h, pmf, std::forward<T>(t)...};
  }

  scheduling_point(ctx);
}


//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous RMI primitive to a range of locations the object exists
///        on.
///
/// The given member function is called on the object on the range @p r of
/// locations it exists on. Any return values are discarded.
///
/// This is an unordered version of the @ref stapl::async_rmi() that may break
/// RMI ordering rules.
///
/// @param r   Destination locations.
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @ingroup ARMIUnordered
//////////////////////////////////////////////////////////////////////
template<typename U, typename Handle, typename MemFun, typename... T>
void async_rmi(location_range_wrapper<U> const& r,
               Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  auto& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");
  STAPL_RUNTIME_ASSERT_MSG(!r.has_invalid(h.get_num_locations()),
                           "Range contains invalid location id.");

  {
    STAPL_RUNTIME_PROFILE("unordered::async_rmi(location_range)",
                          (primitive_traits::non_blocking |
                           primitive_traits::unordered    |
                           primitive_traits::p2m          |
                           primitive_traits::comm));

    using request_type = nc_async_rmi_request<
                           packed_handle_type,
                           MemFun,
                           typename std::remove_reference<T>::type...>;
    const std::size_t size = request_type::expected_size(std::forward<T>(t)...);

    for (auto&& dest : r) {
      aggregator a{ctx, h, dest, no_implicit_flush};
      new(a.allocate(size)) request_type{h, pmf, std::forward<T>(t)...};
    }
  }

  scheduling_point(ctx);
}

//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous unordered RMI primitive.
///
/// The given member function is called on the object in the destination
/// location. Any return values are discarded.
///
/// This function supports combining: if two or more @ref async_rmi() calls are
/// made to the same object, function and location one after another, then the
/// requests are compressed and only the arguments of the requests past the
/// first are sent, saving on space and request execution time.
///
/// This is an unordered version of the @ref stapl::async_rmi() that may break
/// RMI ordering rules.
///
/// @param dest Destination location.
/// @param h    Handle to the target object.
/// @param pmf  Member function to invoke.
/// @param t    Arguments to pass to the member function.
///
/// @ingroup ARMIUnordered
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
void async_rmi(unsigned int dest, Handle const& h, MemFun const& pmf, T&&... t)
{
  runtime::context& ctx = runtime::this_context::base_of_top();

  detail::async_rmi_impl(ctx, dest, h, pmf, std::forward<T>(t)...);
}

} // namespace unordered

} // namespace stapl

#endif
