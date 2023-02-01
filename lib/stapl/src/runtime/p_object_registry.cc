/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/utility/hash_fwd.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/p_object_registry.hpp>
#include <mutex>
#include <typeindex>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>


namespace stapl {

namespace runtime {

namespace {

//////////////////////////////////////////////////////////////////////
/// @brief Typed @ref p_object registry entry.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
struct registry_entry
{
  const rmi_handle* handle;
  const void*       address;
  std::type_index   type;

  registry_entry(const rmi_handle* h,
                 const void* p,
                 std::type_info const& t) noexcept
  : handle{h},
    address{p},
    type{t}
  { }
};


/// Typed @ref p_object registry.
using registry_type =
  boost::multi_index::multi_index_container<
    registry_entry,
    boost::multi_index::indexed_by<
      // index by {object address, object type}
      boost::multi_index::hashed_unique<
        boost::multi_index::composite_key<
          registry_entry,
          BOOST_MULTI_INDEX_MEMBER(registry_entry, const void*, address),
          BOOST_MULTI_INDEX_MEMBER(registry_entry, std::type_index, type)>>,
      // index by rmi_handle*
      boost::multi_index::hashed_unique<
        boost::multi_index::tag<rmi_handle>,
        BOOST_MULTI_INDEX_MEMBER(registry_entry, const rmi_handle*, handle)>>>;

registry_type typed_object_registry;
std::mutex    typed_object_registry_mtx;

} // namespace


// Registers object p with typeid t
void p_object_registry::register_object(const rmi_handle* h,
                                        const void* p,
                                        std::type_info const& t)
{
  std::lock_guard<std::mutex> lock{typed_object_registry_mtx};
  if (typed_object_registry.count(boost::make_tuple(p, std::type_index{t}))!=0)
    STAPL_RUNTIME_ERROR("p_object is already registered.");
  if (!typed_object_registry.emplace(h, p, t).second)
    STAPL_RUNTIME_ERROR("p_object failed to be registered.");
}


// Unregisters object o
void p_object_registry::unregister_object(const rmi_handle* h)
{
  std::lock_guard<std::mutex> lock{typed_object_registry_mtx};
  if (typed_object_registry.get<rmi_handle>().erase(h)!=1)
    STAPL_RUNTIME_ERROR("p_object was not registered.");
}


// Verifies that object o can be cast to type t
void p_object_registry::verify_object_type(const void* p,
                                           std::type_info const& t)
{
  if (!p)
    return;
  std::lock_guard<std::mutex> lock{typed_object_registry_mtx};
  if (typed_object_registry.count(boost::make_tuple(p, std::type_index{t}))!=1)
    STAPL_RUNTIME_ERROR("p_object is not registered with the given type.");
}

} // namespace runtime

} // namespace stapl

