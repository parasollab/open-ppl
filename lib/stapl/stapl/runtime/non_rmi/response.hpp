/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_NON_RMI_RESPONSE_HPP
#define STAPL_RUNTIME_NON_RMI_RESPONSE_HPP

#include "../aggregator.hpp"
#include "../rmi_handle.hpp"
#include "../runqueue.hpp"
#include "../request/rpc_request.hpp"
#include "../request/location_rpc_request.hpp"
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Function object to assign an object to a known address.
///
/// @tparam Handle Handle type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Handle>
class response
{
public:
  typedef Handle handle_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Request to assign an object to a known address.
  //////////////////////////////////////////////////////////////////////
  class request_impl final
  : public rpc_request,
    private handle_type::storage_type
  {
  private:
    typedef typename handle_type::storage_type storage_type;

    handle_type* const m_handle;

  public:
    template<typename T>
    static std::size_t expected_size(T&& t) noexcept
    {
      return (sizeof(request_impl) +
              storage_type::packed_size(std::forward<T>(t)));
    }

    template<typename T>
    request_impl(handle_type* const h, T&& t) noexcept
    : rpc_request(sizeof(*this)),
      storage_type(std::forward<T>(t), this, this->size()),
      m_handle(h)
    { }

    void operator()(message_shared_ptr& m) final
    { m_handle->set_value(this, this, m); }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Request to notify about a @c void return.
  //////////////////////////////////////////////////////////////////////
  class void_request_impl final
  : public rpc_request
  {
  private:
    handle_type* const m_handle;

  public:
    static constexpr std::size_t expected_size(void) noexcept
    { return sizeof(void_request_impl); }

    constexpr explicit void_request_impl(handle_type* const h) noexcept
    : rpc_request(sizeof(*this)),
      m_handle(h)
    { }

    void operator()(message_shared_ptr&) final
    { m_handle->set_value(); }
  };

  typedef typename std::conditional<
            std::is_void<typename handle_type::value_type>::value,
            void_request_impl,
            request_impl
          >::type request_type;

  handle_type* const m_handle;

public:
  constexpr explicit response(handle_type& h) noexcept
  : m_handle(&h)
  { }

  template<typename... T>
  void operator()(context& ctx, T&&... t)
  {
    const process_id pid = ctx.get_source_process_id();
    const bool on_shmem  = (pid==runqueue::get_process_id());
    if (on_shmem) {
      m_handle->set_value(std::forward<T>(t)...);
    }
    else {
      response_aggregator a{ctx};
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{m_handle, std::forward<T>(t)...};
    }
  }

  template<typename... T>
  void operator()(const process_id pid, T&&... t)
  {
    const bool on_shmem = (pid==runqueue::get_process_id());
    if (on_shmem) {
      m_handle->set_value(std::forward<T>(t)...);
    }
    else {
      rpc_aggregator a{pid};
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{m_handle, std::forward<T>(t)...};
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object to assign an object to a known address that expects
///        multiple objects.
///
/// @tparam Handle Handle type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Handle>
class indexed_response
{
public:
  typedef Handle                           handle_type;
  typedef typename handle_type::value_type value_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Request to assign an object to a known address.
  //////////////////////////////////////////////////////////////////////
  class request_impl final
  : public rpc_request,
    private handle_type::storage_type
  {
  private:
    typedef typename handle_type::storage_type storage_type;

    handle_type* const m_handle;
    const location_id  m_lid;

  public:
    template<typename T>
    static std::size_t expected_size(T&& t) noexcept
    {
      return (sizeof(request_impl) +
              storage_type::packed_size(std::forward<T>(t)));
    }

    template<typename T>
    request_impl(handle_type* const h, const location_id lid, T&& t) noexcept
    : rpc_request(sizeof(*this)),
      storage_type(std::forward<T>(t), this, this->size()),
      m_handle(h),
      m_lid(lid)
    { }

    void operator()(message_shared_ptr& m) final
    { m_handle->set_value(m_lid, this, this, m); }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Request to notify about a @c void return.
  //////////////////////////////////////////////////////////////////////
  class void_request_impl final
  : public rpc_request
  {
  private:
    handle_type* const m_handle;
    const location_id  m_lid;

  public:
    static constexpr std::size_t expected_size(void) noexcept
    { return sizeof(void_request_impl); }

    constexpr
    void_request_impl(handle_type* const h, const location_id lid) noexcept
    : rpc_request(sizeof(*this)),
      m_handle(h),
      m_lid(lid)
    { }

    void operator()(message_shared_ptr&) final
    { m_handle->set_value(m_lid); }
  };

  handle_type* const m_handle;

public:
  constexpr explicit indexed_response(handle_type& h) noexcept
  : m_handle(&h)
  { }

  template<typename... T>
  void operator()(context& ctx, T&&... t)
  {
    const process_id pid      = ctx.get_source_process_id();
    const bool on_shmem       = (pid==runqueue::get_process_id());
    const location_md::id lid = ctx.get_location_id();
    if (on_shmem) {
      m_handle->set_value(lid, std::forward<T>(t)...);
    }
    else {
      response_aggregator a{ctx};
      typedef typename std::conditional<
                std::is_void<typename handle_type::value_type>::value,
                void_request_impl,
                request_impl
              >::type request_type;
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{m_handle, lid, std::forward<T>(t)...};
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object to assign an object to a distributed object via the
///        @c Handle::set_value() function.
///
/// @tparam ObjectHandle Distributed object handle type.
/// @tparam Handle       Handle type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ObjectHandle, typename Handle>
class handle_response
{
public:
  typedef Handle handle_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Request to assign an object in a distributed object.
  //////////////////////////////////////////////////////////////////////
  class request_impl final
  : public location_rpc_request,
    private handle_type::storage_type
  {
  private:
    typedef typename handle_type::storage_type storage_type;

    ObjectHandle m_handle;

  public:
    template<typename T>
    static std::size_t expected_size(T&& t) noexcept
    {
      return (sizeof(request_impl) +
              storage_type::packed_size(std::forward<T>(t)));
    }

    template<typename ObjHandle, typename T>
    request_impl(ObjHandle&& h, T&& t) noexcept
    : location_rpc_request(sizeof(*this)),
      storage_type(std::forward<T>(t), this, this->size()),
      m_handle(std::forward<ObjHandle>(h))
    { }

    bool operator()(location_md& l, message_shared_ptr& m) final
    {
      auto* const h = m_handle.template get<handle_type>(l);
      if (!h)
        STAPL_RUNTIME_ERROR("p_object does not exist.");
      h->set_value(this, this, m);
      return true;
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Request to notify about a @c void return.
  //////////////////////////////////////////////////////////////////////
  class void_request_impl final
  : public location_rpc_request
  {
  private:
    ObjectHandle m_handle;

  public:
    static constexpr std::size_t expected_size(void) noexcept
    { return sizeof(void_request_impl); }

    template<typename ObjHandle>
    constexpr explicit void_request_impl(ObjHandle&& h) noexcept
    : location_rpc_request(sizeof(*this)),
      m_handle(std::forward<ObjHandle>(h))
    { }

    bool operator()(location_md& l, message_shared_ptr&) final
    {
      auto* const h = m_handle.template get<handle_type>(l);
      if (!h)
        STAPL_RUNTIME_ERROR("p_object does not exist.");
      h->set_value();
      return true;
    }
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Calls @c Handle::set_value(t...) to all locations of @p g.
  //////////////////////////////////////////////////////////////////////
  template<typename H, typename... T>
  void operator()(H& h, T&&... t)
  {
    all_locations_rpc_aggregator a{h.get_location_md().get_gang_md(),
                                   h.get_epoch()};
    typedef typename std::conditional<
              std::is_void<typename handle_type::value_type>::value,
              void_request_impl,
              request_impl
            >::type request_type;
    const std::size_t size =
      request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{h, std::forward<T>(t)...};
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object to assign an object to a distributed object that
///        accepts multiple objects via the @c Handle::set_value() function.
///
/// @tparam ObjectHandle Distributed object handle type.
/// @tparam Handle       Handle type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ObjectHandle, typename Handle>
class indexed_handle_response
{
public:
  typedef Handle                           handle_type;
  typedef typename handle_type::value_type value_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Request to assign an object to a distributed object.
  //////////////////////////////////////////////////////////////////////
  class request_type final
  : public location_rpc_request,
    private handle_type::storage_type
  {
  private:
    typedef typename handle_type::storage_type storage_type;

    ObjectHandle      m_handle;
    const location_id m_lid;

  public:
    template<typename T>
    static std::size_t expected_size(T&& t) noexcept
    {
      return (sizeof(request_type) +
              storage_type::packed_size(std::forward<T>(t)));
    }

    template<typename ObjHandle, typename T>
    request_type(ObjHandle&& h, const location_id lid, T&& t) noexcept
    : location_rpc_request(sizeof(*this)),
      storage_type(std::forward<T>(t), this, this->size()),
      m_handle(std::forward<ObjHandle>(h)),
      m_lid(lid)
    { }

    bool operator()(location_md& l, message_shared_ptr& m) final
    {
      auto* const h = m_handle.template get<handle_type>(l);
      if (!h)
        STAPL_RUNTIME_ERROR("p_object does not exist.");
      h->set_value(m_lid, this, this, m);
      return true;
    }
  };

public:
  template<typename H, typename T>
  void operator()(H& h, T&& t)
  {
    all_locations_rpc_aggregator a{h.get_location_md().get_gang_md(),
                                   h.get_epoch()};
    const std::size_t size = request_type::expected_size(std::forward<T>(t));
    new(a.allocate(size)) request_type{h,
                                       h.get_location_id(),
                                       std::forward<T>(t)};
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object to call the function operator of a distributed object
///        with the given value.
///
/// @tparam ObjectHandle Distributed object handle type.
/// @tparam Handle       Handle type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ObjectHandle, typename Handle>
class active_handle_response
{
public:
  typedef Handle handle_type;

private:
  ObjectHandle m_handle;

public:
  explicit active_handle_response(handle_type& h) noexcept
  : m_handle(h.get_rmi_handle())
  { }

  template<typename... T>
  void operator()(context& ctx, T&&... t)
  {
    auto* const h = m_handle.template get<handle_type>(ctx.get_location_md());
    if (!h)
      STAPL_RUNTIME_ERROR("p_object does not exist.");
    (*h)(std::forward<T>(t)...);
  }
};

} // namespace runtime

} // namespace stapl

#endif
