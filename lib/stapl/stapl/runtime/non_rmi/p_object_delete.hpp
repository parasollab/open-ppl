/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_NON_RMI_P_OBJECT_DELETE_HPP
#define STAPL_RUNTIME_NON_RMI_P_OBJECT_DELETE_HPP

#include "../aggregator.hpp"
#include "../rmi_handle.hpp"
#include "../request/arg_storage.hpp"
#include "../request/rmi_request.hpp"
#include <memory>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Request to delete a distributed object.
///
/// @tparam T       Object to be deleted type.
/// @tparam Deleter Deleter type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T, typename Deleter>
class delete_object_request final
: public rmi_request,
  private arg_storage_t<rmi_handle::reference, rmi_handle::reference const&>,
  private arg_storage_t<Deleter, Deleter&>
{
private:
  using handle_storage_type  =
    arg_storage_t<rmi_handle::reference, rmi_handle::reference const&>;
  using deleter_storage_type =  arg_storage_t<Deleter, Deleter&>;

public:
  template<typename H, typename D>
  static std::size_t expected_size(H&& h, D&& d) noexcept
  {
    return (sizeof(delete_object_request)                        +
            handle_storage_type::packed_size(std::forward<H>(h)) +
            deleter_storage_type::packed_size(std::forward<D>(d)));
  }

  template<typename H, typename D>
  delete_object_request(H&& h, D&& d) noexcept
  : rmi_request(sizeof(*this)),
    handle_storage_type(std::forward<H>(h), this, this->size()),
    deleter_storage_type(std::forward<D>(d), this, this->size())
  { }

  bool operator()(context& ctx) final
  {
    auto& c = retrieve_object<T>(handle_storage_type::get(this),
                                 ctx.get_location_md());
    deleter_storage_type::get(this)(&c);

    this->~delete_object_request();
    return true;
  }
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Deletes a distributed object.
///
/// @tparam T       Object type.
/// @tparam Deleter Deleter type.
///
/// @ingroup distributedObjects
//////////////////////////////////////////////////////////////////////
template<typename T, typename Deleter = std::default_delete<T>>
class p_object_delete
: public Deleter
{
public:
  using deleter_type = Deleter;

  p_object_delete(void) = default;

  explicit p_object_delete(deleter_type const& d)
  : Deleter(d)
  { }

  explicit p_object_delete(deleter_type&& d)
  : Deleter(std::move(d))
  { }

  void operator()(rmi_handle::reference const& h) const
  {
    using namespace stapl::runtime;
    using request_type = delete_object_request<T, deleter_type>;

    const auto size = request_type::expected_size(h, *this);
    bcast_aggregator a{this_context::get(), h, false};
    new(a.allocate(size)) request_type{h, *this};
  }

  void operator()(T* p) const
  { operator()(p->get_rmi_handle()); }
};

} // namespace stapl

#endif
