/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_TYPER_FWD_HPP
#define STAPL_RUNTIME_SERIALIZATION_TYPER_FWD_HPP

#include <cstddef>
#include <utility>

namespace stapl {

template<typename T, typename = void>
class typer_traits;

template<typename T, typename = void>
struct typer_traits_specialization;


//////////////////////////////////////////////////////////////////////
/// @brief @ref typer class is used for packing, unpacking and copying objects.
///
/// It operates on user-defined class @c T via its @c T::define_type() function,
/// which must clearly define its members to ensure proper packing/unpacking.
///
/// The body of define_type has four sections:
/// @code
/// void define_type(stapl::typer& t)
/// {
///   // 1) base class packing
///   // 2) preliminary calculations
///   // 3) data member definitions
///   // 4) pointer to member definitions
/// }
/// @endcode
///
/// Section 1 is for packing any base classes of @c T. It is undefined behavior
/// to not pack the base classes before everything else.
///
/// Section 2 is for performing preliminary calculations and storing temporary
/// values that may be necessary in sections 2 or 3 (e.g., offsets, sizes, etc).
/// This is necessary because section 2 may modify the contents of members
/// during packing/unpacking.
///
/// Section 3 is for defining data members: variables that store data, either
/// automatically or via dynamic memory allocation. Section 3 starts with the
/// first call to @ref typer::member(). Upon packing of the object, each data
/// member will be properly packed. If a data member is a user-defined object,
/// it will be recursively packed.
///
/// Section 4 is for defining offset members: pointers used as points of
/// reference within the data members previously defined in section 2 (e.g.,
/// some implementations of @c std::vector store a dynamic array of data, along
/// with an offset pointer referencing one past the array's end). Section 4
/// starts with the first call to @ref typer::pointer_to_member(). Only the
/// relative address within the type is packaged upon packing, not the object
/// pointed to, since that object was already packed in Section 3. Attempting to
/// define an offset member to an object not packed in Section 3 is undefined
/// behavior.
///
/// It is recommended that members are processed in the @c T::define_type() in
/// the order they are declared in the class, although this is not necessary for
/// correctness.
///
/// @warning STL iterators are not directly supported by the primitives for
/// packing/unpacking. They typically come in pairs, one for the beginning and
/// one for the end of a sequence, so they cannot be packed in one pass. This
/// can be overcome by using wrapper classes that explicitly implement a
/// @c define_type() method that does provide the necessary information. For
/// example, the iterators can both be stored in a class as a range with the
/// necessary @c define_type() function.
///
/// @warning If two data members both contain pointers to the same object, that
/// object will be packed twice and after unpacking you will end up with two
/// objects. In some cases, such shared pointers are necessary for the structure
/// of the aggregate object and using offset pointers may still allow for a
/// valid @c define_type(). For example, each node in a doubly-linked list has
/// two pointers, allowing for forward and backward traversals through the list.
/// Defining both pointers to be members is not valid. The correct
/// @c define_type() will define the next pointer to be member, to recursively
/// define the list, and the previous pointer to be a pointer to member, to
/// correctly link the list.
///
/// @warning Some implementations use @c define_type() to pack a class into
///          contiguous memory. This packing may use @c memcpy(), or other
///          low-level mechanisms unaware of C++ classes. The C++ Standard
///          states that only PODs may be safely memcpy'ed. However, the purpose
///          of @c define_type() is to give the typer enough information to
///          still safely use @c memcpy. As such, one should not rely on the
///          copy constructor being invoked during its packing/unpacking.
///
/// @ex
/// A user-defined struct for holding data, demonstrating local members.
/// @code
/// struct simple
/// {
///   int    a;
///   double b;
///
///   void define_type(stapl::typer& t)
///   {
///     t.member(a);
///     t.member(b);
///   }
/// };
/// @endcode
///
/// @ex
/// A user-defined array class, demonstrating dynamic members and pointers to
/// members.
/// @code
/// struct array
/// {
///   int* begin;
///   int* end;
///
///   array(const std::size_t size)
///   : begin(new int[size]), end(begin + size)
///   { }
///
///   ~array()
///   { delete[] begin; }
///
///   void define_type(stapl::typer& t)
///   {
///     const std::ptrdiff_t size = end - begin; // section 2
///     t.member(begin, size);                   // section 3
///     t.pointer_to_member(end, begin, size);   // section 4
///   }
/// };
/// @endcode
///
/// @ex
/// A user-defined inherited class, demonstrating how to pack a base class.
/// @code
/// struct derived
/// : public simple
/// {
///   char c[10];
///   void define_type(stapl::typer& t)
///   {
///     t.base<simple>(*this);
///     t.member(c);
///   }
/// };
/// @endcode
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
class typer
{
public:
  enum pass_type
  {
    /// Calculate size of packed object.
    SIZE,
    /// Check if object can be copied and calculate its packed size.
    COPY,
    /// Check if object can be moved and calculate its packed size.
    MOVE,
    /// Check if object marshaling can be avoided and calculate its packed size.
    NO_MARSHAL,
    /// Pack object.
    PACK,
    /// Unpack object.
    UNPACK,
    /// Destroy unpacked object.
    DESTROY
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper function used to disable implicit conversions.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  struct base_type_wrapper
  {
    using type = T;
  };

private:
  /// Pass being executed.
  const pass_type m_pass;
  /// @brief Packed size of the object or offset into the buffer.
  std::size_t     m_offset;
  /// Base pointer for the buffer being examined (@ref PACK, @ref UNPACK).
  char* const     m_base;

  union
  {
    /// Used when packing is active.
    struct
    {
      /// Address of destination object.
      const char* const m_dest;
      /// Address of source object.
      const char* const m_src;
      /// Size of the object.
      const std::size_t m_sizeof;
    };

    /// @c true if requirements are met.
    bool                m_meets_requirements;
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref typer.
  ///
  /// @param pass If @ref SIZE, it calculates the packed size of the object.
  ///             If @ref DESTROY, it cleanups after an unpacked object.
  ///             If @ref COPY, it determines if it is safe to copy an object
  ///             and calculates its packed size.
  ///             If @ref MOVE, it determines if it is safe to move an object
  ///             and calculates its packed size.
  ///             If @ref NO_MARSHAL, it determines if the object does not have
  ///             to be marshaled and calculates its packed size.
  //////////////////////////////////////////////////////////////////////
  constexpr typer(const pass_type p) noexcept
  : m_pass(p),
    m_offset(0),
    m_base(nullptr),
    m_meets_requirements(true)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref typer for packing an object.
  ///
  /// @param dest   Destination object.
  /// @param src    Source object.
  /// @param base   Pointer to buffer to pack the dynamic part of @p src.
  /// @param offset Offset in @p base.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  typer(T& dest, T const& src, void* base, const std::size_t offset);

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref typer for unpacking an object.
  ///
  /// @param base Pointer to buffer to unpack the dynamic part of the object.
  //////////////////////////////////////////////////////////////////////
  typer(void* base)
  : m_pass(UNPACK),
    m_offset(0),
    m_base(static_cast<char*>(base)),
    m_meets_requirements(false)
  { }

  typer(typer const&) = delete;
  typer& operator=(typer const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current pass of the @ref typer.
  ///
  /// @warning If this function is called, then the object cannot be copied or
  ///          moved, as it is implied that the @c define_type() function is
  ///          performing some operation that has to happen at copying/moving.
  //////////////////////////////////////////////////////////////////////
  pass_type pass(void) noexcept
  {
    switch (m_pass) {
      case COPY:
      case MOVE:
        m_meets_requirements = false;
        break;

      default:
        // nothing
        break;
    }

    return m_pass;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of bytes since the start of the current
  ///        operation.
  //////////////////////////////////////////////////////////////////////
  std::size_t offset(void) const noexcept
  { return m_offset; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if the object meets the requested requirements.
  //////////////////////////////////////////////////////////////////////
  std::pair<bool, std::size_t> meets_requirements(void) const noexcept
  { return std::make_pair(m_meets_requirements, m_offset); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Processes a base class.
  //////////////////////////////////////////////////////////////////////
  template<typename Base>
  void base(typename base_type_wrapper<Base>::type& t)
  { member(t); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Processes member @p t.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void member(T& t);

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::member(T&)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void member(T const& t)
  { member(const_cast<T&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::member(T&)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void member(T volatile& t)
  { member(const_cast<T&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::member(T&)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void member(T const volatile& t)
  { member(const_cast<T&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Processes member @p t with an additional integral type passed to
  ///        the packing functions.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void member(T& t, const std::size_t N);

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::member(T&,const std::size_t)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void member(T const& t, const std::size_t N)
  { member(const_cast<T&>(t), N); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::member(T&,const std::size_t)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void member(T volatile& t, const std::size_t N)
  { member(const_cast<T&>(t), N); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::member(T&,const std::size_t)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void member(T const volatile& t, const std::size_t N)
  { member(const_cast<T&>(t), N); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Default constructs @p t at unpacking.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void transient(T& t);

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::transient(T&)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void transient(T const& t)
  { transient(const_cast<T&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::transient(T&)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void transient(T volatile& t)
  { transient(const_cast<T&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::transient(T&)
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void transient(T const volatile& t)
  { transient(const_cast<T&>(t)); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs @p t using @p u at unpacking.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  void transient(T& t, U&& u);

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::transient(T&,U const&)
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  void transient(T const& t, U&& u)
  { transient(const_cast<T&>(t), std::forward<U>(u)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::transient(T&,U const&)
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  void transient(T volatile& t, U&& u)
  { transient(const_cast<T&>(t), std::forward<U>(u)); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::transient(T&,U const&)
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  void transient(T const volatile& t, U&& u)
  { transient(const_cast<T&>(t), std::forward<U>(u)); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Processes a pointer @p t to other member @p ref at offset
  ///        @p offset.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  void pointer_to_member(T*& t, U* ref, const std::size_t offset = 0) noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::pointer_to_member(T*&,U*,const std::size_t)
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  void pointer_to_member(T* const& t, U* ref,
                         const std::size_t offset = 0) noexcept
  { pointer_to_member(const_cast<T*&>(t), ref, offset); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::pointer_to_member(T*&,U*,const std::size_t)
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  void pointer_to_member(T* volatile& t, U* ref,
                         const std::size_t offset = 0) noexcept
  { pointer_to_member(const_cast<T*&>(t), ref, offset); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc typer::pointer_to_member(T*&,U*,const std::size_t)
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U>
  void pointer_to_member(T* const volatile& t, U* ref,
                         const std::size_t offset = 0) noexcept
  { pointer_to_member(const_cast<T*&>(t), ref, offset); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for a bitwise packed members.
///
/// @related bitwise()
/// @see bitwise
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
struct bitwise_wrapper
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Declares the object @p t as bitwise packable.
///
/// @related bitwise_wrapper
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
constexpr bitwise_wrapper<T> bitwise(T const&)
{
  return bitwise_wrapper<T>{};
}


////////////////////////////////////////////////////////////////////
/// @brief Transforms an object of type @p T to an object of type @c U that has
///        a @c U::define_type(stapl::typer&) function through the @ref apply()
///        function.
///
/// @ingroup serialization
////////////////////////////////////////////////////////////////////
template<typename T>
struct define_type_provider
{
  static constexpr T& apply(T& t) noexcept
  { return t; }
};


namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Transforms a reference of type @p T to a reference of type @c U that
///        has a @c U::define_type(stapl::typer&) function.
///
/// @related define_type_provider
/// @ingroup serializationImpl
////////////////////////////////////////////////////////////////////
template<typename T>
auto define_type_cast(T& t) noexcept
  -> decltype(define_type_provider<T>::apply(t))
{
  return define_type_provider<T>::apply(t);
}


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_cast(T&) for @c const references.
///
/// @related define_type_provider
/// @ingroup serializationImpl
////////////////////////////////////////////////////////////////////
template<typename T>
auto define_type_cast(T const& t) noexcept
  -> decltype(define_type_provider<T>::apply(const_cast<T&>(t)))
{
  return define_type_provider<T>::apply(const_cast<T&>(t));
}


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_cast(T&) for @c volatile
///        references.
///
/// @related define_type_provider
/// @ingroup serializationImpl
////////////////////////////////////////////////////////////////////
template<typename T>
auto define_type_cast(T volatile& t) noexcept
  -> decltype(define_type_provider<T>::apply(const_cast<T&>(t)))
{
  return define_type_provider<T>::apply(const_cast<T&>(t));
}


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_cast(T&) for @c const @c volatile
///        references.
///
/// @related define_type_provider
/// @ingroup serializationImpl
////////////////////////////////////////////////////////////////////
template<typename T>
auto define_type_cast(T const volatile& t) noexcept
  -> decltype(define_type_provider<T>::apply(const_cast<T&>(t)))
{
  return define_type_provider<T>::apply(const_cast<T&>(t));
}

} // namespace runtime

} // namespace stapl

#endif
