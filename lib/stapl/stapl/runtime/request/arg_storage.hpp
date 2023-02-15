/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_ARG_STORAGE_HPP
#define STAPL_RUNTIME_REQUEST_ARG_STORAGE_HPP

#include "../serialization.hpp"
#include "../serialization/object_storage.hpp"
#include "../serialization/packed_object_storage.hpp"
#include "../type_traits/is_basic.hpp"
#include "../type_traits/is_copyable.hpp"
#include "../type_traits/is_movable.hpp"
#include "../type_traits/is_p_object.hpp"
#include "../type_traits/polymorphic.hpp"
#include <tuple>
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Storage for an object of type @p T that is retrieved by lvalue
///        reference.
///
/// @tparam T Object type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class arg_storage
: private packed_object_storage<typename std::remove_cv<T>::type>
{
private:
  using value_type = typename std::remove_cv<T>::type;
  using base_type  = packed_object_storage<value_type>;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the packed size of @p t.
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  { return base_type::packed_size(const_cast<value_type const&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new object by packing @p t in it.
  ///
  /// This constructor will copy the dynamic part of the object at
  /// <tt>base + size</tt>, which should be big enough to fit it.
  ///
  /// @param t    Object to pack.
  /// @param base Pointer to buffer start where the dynamic part of @p t can be
  ///             stored in.
  /// @param size Offset from @p base where space is available.
  //////////////////////////////////////////////////////////////////////
  arg_storage(T const& t, void* const base, std::size_t& size) noexcept
  : base_type(const_cast<value_type const&>(t), base, size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Packs the object contained in tuple @p t.
  ///
  /// @param t Tuple with the object to pack, the start of the buffer for the
  ///          dynamic part of the object and the offset from the start of the
  ///          buffer.
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  arg_storage(Tuple&& t) noexcept
  : arg_storage(std::get<0>(t), std::get<1>(t), std::get<2>(t))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the stored object after unpacking it.
  ///
  /// @param base Buffer where the dynamic part of the object is stored.
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  { return base_type::get(base); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the stored object after unpacking it.
  ///
  /// @param base Buffer where the dynamic part of the object is stored.
  /// @param size Variable to store how many bytes were unpacked.
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  { return base_type::get(base, size); }
};


////////////////////////////////////////////////////////////////////
/// @brief Argument storage type creation metafunction.
///
/// @tparam T Stored object type.
/// @tparam R Object retrieval type.
///
/// This class provides member typedef @c type that has a suitable storage type
/// for @p T when retrieved as @p R.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R, typename = void>
struct make_arg_storage
{
  using type = arg_storage<T>;
};


////////////////////////////////////////////////////////////////////
/// @brief Argument storage type.
///
/// @tparam T Stored object type.
/// @tparam R Object retrieval type.
///
/// This class provides a storage type that is suitable for storing an object of
/// type @p T when retrieved as @p R.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
using arg_storage_t = typename make_arg_storage<T, R>::type;


//////////////////////////////////////////////////////////////////////
/// @brief Storage for a non-const, non-basic, non-distributed object that is
///        retrieved by non-const lvalue reference to non-basic object.
///
/// @tparam T Object type.
///
/// During unpacking, a new object is copy constructed from the stored object
/// and returned by reference. The copy constructed object is deleted when the
/// destructor is called.
///
/// @see is_basic, is_p_object
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class non_const_arg_storage
{
private:
  using value_type   = typename std::remove_cv<T>::type;
  using storage_type = packed_object_storage<value_type>;

  union
  {
    storage_type m_storage;
    /// Pointer to object clone.
    T*           m_p;
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  { return storage_type::packed_size(const_cast<value_type const&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  non_const_arg_storage(T const& t,
                        void* const base,
                        std::size_t& size) noexcept
  : m_storage(const_cast<value_type const&>(t), base, size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  non_const_arg_storage(Tuple&& t) noexcept
  : non_const_arg_storage(std::get<0>(t), std::get<1>(t), std::get<2>(t))
  { }

  non_const_arg_storage(non_const_arg_storage const&) = delete;
  non_const_arg_storage& operator=(non_const_arg_storage const&) = delete;

  ~non_const_arg_storage(void)
  { delete m_p; }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  {
    auto* const t = clone(m_storage.get(base));
    m_storage.~storage_type();
    m_p = t;
    return *m_p;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  {
    auto* const t = clone(m_storage.get(base, size));
    m_storage.~storage_type();
    m_p = t;
    return *m_p;
  }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for non-const, non-basic,
///        non-distributed objects that are retrieved by non-const lvalue
///        reference to non-basic object.
///
/// @see is_basic, is_p_object
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<T, R&,
                        typename std::enable_if<
                          !is_p_object<T>::value   &&
                          !std::is_const<T>::value &&
                          !is_basic<T>::value      &&
                          !std::is_const<R>::value &&
                          !is_basic<R>::value
                        >::type>
{
  using type = non_const_arg_storage<T>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Storage for an lvalue reference to a distributed object.
///
/// @tparam T Distributed object type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class p_object_ref_storage
: private packed_object_storage<T*>
{
private:
  using value_type = typename std::remove_cv<T>::type;
  using base_type  = packed_object_storage<value_type*>;

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  { return base_type::packed_size(std::addressof(const_cast<value_type&>(t))); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  p_object_ref_storage(T const& t, void* const base, std::size_t& size) noexcept
  : base_type(std::addressof(const_cast<value_type&>(t)), base, size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  p_object_ref_storage(Tuple&& t) noexcept
  : p_object_ref_storage(std::get<0>(t), std::get<1>(t), std::get<2>(t))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  { return *base_type::get(base); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  { return *base_type::get(base, size); }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for lvalue references to
///        distributed objects.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<T, R&,
                        typename std::enable_if<
                          is_p_object<T>::value
                        >::type>
{
  using type = p_object_ref_storage<T>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Storage for a pointer to non-const, non-basic, non-distributed object
///        that is retrieved by pointer to a non-const, non-basic object.
///
/// @tparam T Object pointer type.
///
/// During unpacking, a new object is copy constructed from the stored object
/// and returned by pointer. The copy constructed object is deleted when the
/// destructor is called.
///
/// @see is_basic, is_p_object
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class non_const_ptr_storage
{
private:
  using value_type   = typename std::remove_cv<T>::type;
  using storage_type = packed_object_storage<value_type>;

  union
  {
    storage_type m_storage;
    /// Pointer to object clone.
    T            m_p;
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  { return storage_type::packed_size(const_cast<value_type const&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  non_const_ptr_storage(T const& t,
                        void* const base,
                        std::size_t& size) noexcept
  : m_storage(const_cast<value_type const&>(t), base, size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  non_const_ptr_storage(Tuple&& t) noexcept
  : non_const_ptr_storage(std::get<0>(t), std::get<1>(t), std::get<2>(t))
  { }

  non_const_ptr_storage(non_const_ptr_storage const&) = delete;
  non_const_ptr_storage& operator=(non_const_ptr_storage const&) = delete;

  ~non_const_ptr_storage(void)
  { delete m_p; }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  {
    auto&& p = m_storage.get(base);
    if (p) {
      auto* const t = clone(*p);
      m_storage.~storage_type();
      m_p = t;
    }
    else {
      m_p = nullptr;
    }
    return m_p;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  {
    auto&& p = m_storage.get(base, size);
    if (p) {
      auto* const t = clone(*p);
      m_storage.~storage_type();
      m_p = t;
    }
    else {
      m_p = nullptr;
    }
    return m_p;
  }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for pointers to non-const,
///        non-basic, non-distributed objects that are retrieved by pointer to
///        non-const, non-basic objects.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<T, R*,
                        typename std::enable_if<
                          !std::is_same<T, std::nullptr_t>::value &&
                          !is_p_object_pointer<T>::value          &&
                          !is_p_object<T>::value                  &&
                          !std::is_const<T>::value                &&
                          !is_basic<T>::value                     &&
                          !std::is_const<R>::value                &&
                          !is_basic<R>::value
                        >::type>
{
  using type = non_const_ptr_storage<T>;
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for const pointers.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<T, R* const>
: public make_arg_storage<T, R*>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Storage for an object that is retrieved by rvalue reference.
///
/// @tparam T Object type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class rval_storage
: private packed_object_storage<T>
{
private:
  using base_type = packed_object_storage<T>;

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T&& t) noexcept
  { return base_type::packed_size(std::move(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  rval_storage(T&& t, void* const base, std::size_t& size) noexcept
  : base_type(std::move(t), base, size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  rval_storage(Tuple&& t) noexcept
  : rval_storage(std::move(std::get<0>(t)), std::get<1>(t), std::get<2>(t))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T get(void* const base)
  { return base_type::get(base); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T get(void* const base, std::size_t& size)
  { return base_type::get(base, size); }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for objects retrieved by
///        rvalue reference.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<T, R&&>
{
  using type = rval_storage<T>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Storage for an object that is moved and retrieved either by copy or
///        by rvalue reference.
///
/// @tparam T Object type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class moved_object_storage
: private object_storage<T>
{
private:
  using base_type = object_storage<T>;

  const bool m_moved;

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T&& t) noexcept
  { return base_type::packed_size(std::move(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  moved_object_storage(T&& t, void* const base, std::size_t& size) noexcept
  : m_moved(base_type::construct(std::move(t), base, size))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  moved_object_storage(Tuple&& t) noexcept
  : moved_object_storage(std::move(std::get<0>(t)),
                         std::get<1>(t),
                         std::get<2>(t))
  { }

  ~moved_object_storage(void)
  { (m_moved ? base_type::destroy() : base_type::destroy_packed()); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T get(void* const base)
  { return (m_moved ? base_type::moveout() : base_type::get(base)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T get(void* const base, std::size_t& size)
  { return (m_moved ? base_type::moveout() : base_type::get(base, size)); }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for moved objects retrieved
///        by copy.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<movable<T>, R>
{
  using type = moved_object_storage<T>;
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for moved objects retrieved
///        by rvalue reference.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<movable<T>, R&&>
{
  using type = moved_object_storage<T>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Storage for a moved object that is retrieved by const reference.
///
/// @tparam T Object type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class moved_cref_storage
: private object_storage<T>
{
private:
  using base_type = object_storage<T>;

  const bool m_moved;

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T&& t) noexcept
  { return base_type::packed_size(std::move(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  moved_cref_storage(T&& t, void* const base, std::size_t& size) noexcept
  : m_moved(base_type::construct(std::move(t), base, size))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  moved_cref_storage(Tuple&& t) noexcept
  : moved_cref_storage(std::move(std::get<0>(t)),
                       std::get<1>(t),
                       std::get<2>(t))
  { }

  ~moved_cref_storage(void)
  { (m_moved ? base_type::destroy() : base_type::destroy_packed()); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  { return (m_moved ? base_type::get() : base_type::get(base)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  { return (m_moved ? base_type::get() : base_type::get(base, size)); }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for moved objects retrieved
///        by const reference.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<movable<T>, R const&>
{
  using type = moved_cref_storage<T>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Storage for a moved object that is retrieved by reference.
///
/// @tparam T Object type.
///
/// During unpacking, a new object is copy constructed from the stored object if
/// the move has failed and returned by reference. The copy constructed object
/// is deleted when the destructor is called.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class moved_ref_storage
{
private:
  using storage_type = object_storage<T>;

  union
  {
    storage_type m_storage;
    /// Pointer to object clone.
    T*           m_p;
  };
  const bool     m_moved;

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T&& t) noexcept
  { return storage_type::packed_size(std::move(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  moved_ref_storage(T&& t, void* const base, std::size_t& size) noexcept
  : m_storage(),
    m_moved(m_storage.construct(std::move(t), base, size))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  moved_ref_storage(Tuple&& t) noexcept
  : moved_ref_storage(std::move(std::get<0>(t)), std::get<1>(t), std::get<2>(t))
  { }

  moved_ref_storage(moved_ref_storage const&) = delete;
  moved_ref_storage& operator=(moved_ref_storage const&) = delete;

  ~moved_ref_storage(void)
  { (m_moved ? m_storage.destroy() : delete m_p); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  {
    if (m_moved)
      return m_storage.get();
    auto& t       = m_storage.get(base);
    auto* const p = clone(t);
    m_storage.destroy_packed();
    m_p = p;
    return *m_p;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  {
    if (m_moved)
      return m_storage.get();
    auto& t       = m_storage.get(base, size);
    auto* const p = clone(t);
    m_storage.destroy_packed();
    m_p = p;
    return *m_p;
  }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for moved objects retrieved
///        by reference.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<movable<T>, R&>
{
  using type = moved_ref_storage<T>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Storage for a moved object that is retrieved by copy or by const
///        reference.
///
/// @tparam T Object type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class copied_arg_storage
: private object_storage<T>
{
private:
  using base_type = object_storage<T>;

  const bool m_copied;

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  { return base_type::packed_size(t); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  copied_arg_storage(T const& t, void* const base, std::size_t& size) noexcept
  : m_copied(base_type::construct(t, base, size))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  copied_arg_storage(Tuple&& t) noexcept
  : copied_arg_storage(std::get<0>(t), std::get<1>(t), std::get<2>(t))
  { }

  ~copied_arg_storage(void)
  { (m_copied ? base_type::destroy() : base_type::destroy_packed()); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  { return (m_copied ? base_type::get() : base_type::get(base)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  { return (m_copied ? base_type::get() : base_type::get(base, size)); }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for copied objects retrieved
///        by copy or by const reference.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<copyable<T>, R>
{
  using type = copied_arg_storage<T>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Storage for a moved object that is retrieved by non-const reference.
///
/// @tparam T Object type.
///
/// During unpacking, a new object is copy constructed from the stored object if
/// the copy has failed and returned by reference. The copy constructed object
/// is deleted when the destructor is called.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename T>
class copied_ref_storage
{
private:
  using storage_type = object_storage<T>;

  union
  {
    /// Stored object (packed or copied).
    storage_type m_storage;
    /// Pointer to object clone.
    T*           m_p;
  };
  const bool     m_copied;

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::packed_size()
  //////////////////////////////////////////////////////////////////////
  static std::size_t packed_size(T const& t) noexcept
  { return storage_type::packed_size(t); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(T const&,void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  copied_ref_storage(T const& t, void* const base, std::size_t& size) noexcept
  : m_storage(),
    m_copied(m_storage.construct(t, base, size))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::arg_storage(Tuple&&)
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple>
  copied_ref_storage(Tuple&& t) noexcept
  : copied_ref_storage(std::get<0>(t), std::get<1>(t), std::get<2>(t))
  { }

  copied_ref_storage(copied_ref_storage const&) = delete;
  copied_ref_storage& operator=(copied_ref_storage const&) = delete;

  ~copied_ref_storage(void)
  { (m_copied ? m_storage.destroy() : delete m_p); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base)
  {
    if (m_copied)
      return m_storage.get();
    auto& t       = m_storage.get(base);
    auto* const p = clone(t);
    m_storage.destroy_packed();
    m_p = p;
    return *m_p;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc arg_storage::get(void*const,std::size_t&)
  //////////////////////////////////////////////////////////////////////
  T& get(void* const base, std::size_t& size)
  {
    if (m_copied)
      return m_storage.get();
    auto& t       = m_storage.get(base, size);
    auto* const p = clone(t);
    m_storage.destroy_packed();
    m_p = p;
    return *m_p;
  }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arg_storage for copied objects retrieved
///        by reference.
///
/// @ingroup requestBuildingBlock
////////////////////////////////////////////////////////////////////
template<typename T, typename R>
struct make_arg_storage<copyable<T>, R&,
                        typename std::enable_if<
                          !std::is_const<R>::value
                        >::type>
{
  using type = copied_ref_storage<T>;
};

} // namespace runtime

} // namespace stapl

#endif
