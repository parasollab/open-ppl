/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_SHARED_OBJECT_FACTORY_HPP
#define STAPL_RUNTIME_CONCURRENCY_SHARED_OBJECT_FACTORY_HPP

#include <memory>
#include <mutex>
#include <utility>
#include <unordered_map>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Factory for creating object shared between threads.
///
/// The shared objects are indexed with the @p Key type and are returned as
/// reference counted pointers through @p std::shared_ptr.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
template<typename Key,
         typename Hash     = std::hash<Key>,
         typename KeyEqual = std::equal_to<Key>>
class shared_object_factory
{
public:
  using size_type = std::size_t;
  using key_type  = Key;
  using hasher    = Hash;
  using key_equal = KeyEqual;
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Object storage base class.
  //////////////////////////////////////////////////////////////////////
  class storage_base
  {
  private:
    size_type m_count;

  public:
    explicit storage_base(const size_type count)
    : m_count(count)
    { }

    virtual ~storage_base(void) = default;

    bool release(void) noexcept
    { return (--m_count==0); }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Object storage.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  class storage final
  : public storage_base
  {
  public:
    T m_t;

  public:
    template<typename... U>
    explicit storage(const size_type count, U&&... u)
    : storage_base(count),
      m_t(std::forward<U>(u)...)
    { }

    T* get_ptr(void) noexcept
    { return &m_t; }
  };

  using container_type =
    std::unordered_map<
      key_type, std::shared_ptr<storage_base>, hasher, key_equal
    >;

  container_type m_map;
  std::mutex     m_mtx;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an object of type @p T that is shared among threads.
  ///
  /// The returned object is valid for @p n requesting threads and is indexed
  /// with @p key.
  ///
  /// @warning Only one shared object is allowed per key.
  ///
  /// @param key  Key to index a shared object of type @p T.
  /// @param N    Number of threads requesting the object.
  /// @param args Arguments to pass to the constructor of the shared object.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename... Args>
  std::shared_ptr<T> create(key_type const& key, size_type n, Args&&... args)
  {
    using storage_type = storage<T>;

    // for less than two threads there is no need to register
    if (n<2)
      return std::make_shared<T>(std::forward<Args>(args)...);

    std::shared_ptr<storage_base> p;
    {
      std::lock_guard<std::mutex> lock{m_mtx};
      auto it = m_map.find(key);
      if (it!=m_map.end()) {
        // object with the given key found
        p = it->second;
        // remove object from the container if this is the last thread
        if (p->release())
          m_map.erase(it);
      }
      else {
        // create a new shared object
        p = std::make_shared<storage_type>((n-1), std::forward<Args>(args)...);
        m_map.emplace(key, p);
      }
    }
    return std::shared_ptr<T>{p, static_cast<storage_type&>(*p).get_ptr()};
  }
};

} // namespace runtime

} // namespace stapl

#endif
