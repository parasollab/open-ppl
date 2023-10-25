/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_NON_RMI_RESTORE_HPP
#define STAPL_RUNTIME_NON_RMI_RESTORE_HPP

#include "../context.hpp"
#include "../future.hpp"
#include "../rmi_handle.hpp"
#include "../runqueue.hpp"
#include "../value_handle.hpp"
#include "../request/arguments.hpp"
#include "../request/location_rpc_request.hpp"
#include "../type_traits/callable_traits.hpp"
#include <memory>
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Request for restoring an SPMD section.
///
/// @tparam ObjectHandle Distributed object handle type.
/// @tparam MemFun       Member function pointer type.
/// @tparam T            Argument types.
///
/// This restores the SPMD execution for a gang that its locations are not
/// on the top of the tread-local stack.
///
/// Restoring the SPMD section is a procedure that requires multiple tries,
/// since another gang might be calling @ref restore() as well. Therefore, it is
/// quite costly as an operation.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ObjectHandle, typename MemFun, typename... T>
class restore_request final
: public location_rpc_request,
  private arguments_t<MemFun, T...>
{
private:
  using args_type          = arguments_t<MemFun, T...>;
  using seq_type           = make_index_sequence<sizeof...(T)>;
  using object_type        = typename callable_traits<MemFun>::object_type;
  using pmf_result_type    = typename callable_traits<MemFun>::result_type;
public:
  using return_handle_type = value_handle<pmf_result_type>;

private:
  ObjectHandle              m_handle;
  const MemFun              m_pmf;
  return_handle_type* const m_rhandle;

  //////////////////////////////////////////////////////////////////////
  /// @brief Executes the request when the return type is @c void.
  //////////////////////////////////////////////////////////////////////
  void apply(object_type& t, std::true_type)
  {
    invoke(m_pmf, t, static_cast<args_type&>(*this),
           static_cast<void*>(this), seq_type{});
    m_rhandle->set_value();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Executes the request when the return type is not @c void.
  //////////////////////////////////////////////////////////////////////
  void apply(object_type& t, std::false_type)
  {
    m_rhandle->set_value(
      invoke(m_pmf, t, static_cast<args_type&>(*this),
             static_cast<void*>(this), seq_type{}));
  }

public:
  template<typename... U>
  static std::size_t expected_size(U&&... u) noexcept
  {
    return (sizeof(restore_request) +
            dynamic_size<args_type>(seq_type{}, std::forward<U>(u)...));
  }

  template<typename RetHandle, typename Handle, typename F, typename... U>
  restore_request(RetHandle&& rh, Handle&& h, F&& f, U&&... u) noexcept
  : location_rpc_request(sizeof(*this)),
    args_type(std::forward_as_tuple(std::forward<U>(u),
                                    static_cast<void*>(this),
                                    this->size())...),
    m_handle(std::forward<Handle>(h)),
    m_pmf(std::forward<F>(f)),
    m_rhandle(std::forward<RetHandle>(rh))
  { }

  bool operator()(location_md& l, message_shared_ptr&) final
  {
    if (!this_context::can_restore(l)) {
      // cannot execute right now, try again later
      return false;
    }

    context ctx{l};

    auto* const t = m_handle.template get<object_type>(l);
    if (!t)
      STAPL_RUNTIME_ERROR("p_object does not exist.");

    if (l.is_leader()) {
      apply(*t, typename std::is_void<pmf_result_type>::type{});
    }
    else {
      invoke(m_pmf, *t, static_cast<args_type&>(*this),
             static_cast<void*>(this), seq_type{});
    }

    this->~restore_request();
    return true;
  }
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Restores the SPMD execution of the gang of the given object, using as
///        the entry point the given member function.
///
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return A @ref future object with the return value of the invoked member
///         function from location 0.
///
/// @ingroup distributedObjects
///
/// @todo It does not work for gangs that are on more that one process.
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
future<typename callable_traits<MemFun>::result_type>
restore(Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  typedef restore_request<
            packed_handle_type,
            MemFun,
            typename std::remove_reference<T>::type...
          > request_type;
  typedef typename request_type::return_handle_type return_handle_type;

  std::unique_ptr<return_handle_type> p{new return_handle_type};
  {
    STAPL_RUNTIME_PROFILE("restore()", (primitive_traits::non_blocking |
                                        primitive_traits::p2m          |
                                        primitive_traits::comm));

    gang_md* const g = gang_md_registry::try_get(h.get_gang_id());
    if (!g)
      STAPL_RUNTIME_ERROR("Could not restore gang.");
    if (!g->get_description().is_on_shmem())
      STAPL_RUNTIME_ERROR("Not implemented for gangs that span multiple "
                          "processes.");

    all_locations_rpc_aggregator a{*g, h.get_epoch()};
    const std::size_t size = request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{p.get(), h, pmf, std::forward<T>(t)...};
  }

  scheduling_point(ctx);
  return future<typename callable_traits<MemFun>::result_type>{std::move(p)};
}

} // namespace stapl

#endif
