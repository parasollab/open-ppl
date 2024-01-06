/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_BOOST_SERIALIZATION_HPP
#define STAPL_RUNTIME_SERIALIZATION_BOOST_SERIALIZATION_HPP

#include "typer_traits.hpp"
#include "packed_object_storage.hpp"
#include "../serialization/typer_traits.hpp"
#include "../type_traits/supports_stapl_packing.hpp"
#include <cstring>
#include <memory>
#include <sstream>
#include <streambuf>
#include <type_traits>
#include <boost/archive/binary_oarchive_impl.hpp>
#include <boost/archive/binary_iarchive_impl.hpp>
#include <boost/archive/detail/register_archive.hpp>
#include <boost/archive/impl/basic_binary_oprimitive.ipp>
#include <boost/archive/impl/basic_binary_iprimitive.ipp>
#include <boost/archive/impl/basic_binary_oarchive.ipp>
#include <boost/archive/impl/basic_binary_iarchive.ipp>

namespace stapl {

namespace runtime {

// --------------------------------------------------------------------------
// STREAMS
// --------------------------------------------------------------------------

namespace buffer_archive_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Calculates the total number of bytes written to the stream.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
class size_stream
: public std::streambuf
{
private:
  std::streamsize m_size;

public:
  size_stream(void)
  : m_size(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Counts how many bytes are written into the stream.
  //////////////////////////////////////////////////////////////////////
  std::streamsize xsputn(const char_type*, std::streamsize n) final
  {
    advance(n*sizeof(char_type));
    return n;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advances the buffer pointer.
  //////////////////////////////////////////////////////////////////////
  void advance(const std::size_t size) noexcept
  { m_size += size; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns how many bytes have been written.
  //////////////////////////////////////////////////////////////////////
  std::streamsize written_bytes(void) const noexcept
  { return m_size; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Writes the bytes to the given buffer.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
class output_stream
: public std::streambuf
{
private:
  char*       m_buffer;
  char* const m_ibuffer;

public:
  explicit output_stream(void* p)
  : m_buffer(static_cast<char*>(p)),
    m_ibuffer(static_cast<char*>(p))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Writes the given bytes to the buffer.
  //////////////////////////////////////////////////////////////////////
  std::streamsize xsputn(const char_type* s, std::streamsize n) final
  {
    std::memcpy(m_buffer, s, n*sizeof(char_type));
    advance(n*sizeof(char_type));
    return n;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns pointer to the buffer.
  //////////////////////////////////////////////////////////////////////
  void const* address(void) const noexcept
  { return m_buffer; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns pointer to the buffer.
  //////////////////////////////////////////////////////////////////////
  void* address(void) noexcept
  { return m_buffer; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advances the buffer pointer.
  //////////////////////////////////////////////////////////////////////
  void advance(const std::size_t size) noexcept
  { m_buffer += size; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns how many bytes have been written.
  //////////////////////////////////////////////////////////////////////
  std::streamsize written_bytes(void) const noexcept
  { return (m_buffer - m_ibuffer); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reads bytes from the given buffer.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
class input_stream
: public std::streambuf
{
private:
  char*       m_buffer;
  char* const m_ibuffer;

public:
  explicit input_stream(void* p)
  : m_buffer(static_cast<char*>(p)),
    m_ibuffer(static_cast<char*>(p))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reads the requested bytes from the buffer.
  //////////////////////////////////////////////////////////////////////
  std::streamsize xsgetn(char_type *s, std::streamsize n) final
  {
    std::memcpy(s, m_buffer, n*sizeof(char_type));
    advance(n*sizeof(char_type));
    return n;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns pointer to the buffer.
  //////////////////////////////////////////////////////////////////////
  void const* address(void) const noexcept
  { return m_buffer; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns pointer to the buffer.
  //////////////////////////////////////////////////////////////////////
  void* address(void) noexcept
  { return m_buffer; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advances the buffer pointer.
  //////////////////////////////////////////////////////////////////////
  void advance(const std::size_t size) noexcept
  { m_buffer += size; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns how many bytes have been read.
  //////////////////////////////////////////////////////////////////////
  std::streamsize read_bytes(void) const noexcept
  { return (m_buffer - m_ibuffer); }
};

} // namespace buffer_archive_impl


// --------------------------------------------------------------------------
// ARCHIVES
// --------------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////
/// @brief Flags required for the @c boost::archive objects.
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
constexpr unsigned int boost_serialization_flags =
  (boost::archive::no_header  |
   boost::archive::no_codecvt |
   boost::archive::no_xml_tag_checking);


//////////////////////////////////////////////////////////////////////
/// @brief Archive to calculate the size an object needs for packing.
///
/// For more information, visit
/// http://www.boost.org/doc/libs/release/libs/serialization/doc/index.html .
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
class size_oarchive
: private buffer_archive_impl::size_stream,
  public boost::archive::binary_oarchive_impl<
           size_oarchive,
           std::ostream::char_type,
           std::ostream::traits_type
         >
{
private:
  using base1_type = buffer_archive_impl::size_stream;
  using base2_type = boost::archive::binary_oarchive_impl<
                       size_oarchive,
                       std::ostream::char_type,
                       std::ostream::traits_type>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Calculates the size required for Boost.Serialization handled types.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void save_override_impl(T& t, std::false_type)
  { base2_type::save_override(t); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Calculates the size required for STAPL serialization handled types.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void save_override_impl(T& t, std::true_type)
  {
    using storage_type = packed_object_storage<T>;
    const std::size_t size =
      (sizeof(storage_type) + storage_type::packed_size(t));
    advance(size);
  }

#define ARCHIVE_IGNORE_IMPLEMENTATION(T) void save_override(T const&) { }
  ARCHIVE_IGNORE_IMPLEMENTATION(boost::archive::version_type)
  ARCHIVE_IGNORE_IMPLEMENTATION(boost::archive::tracking_type)
#undef ARCHIVE_IGNORE_IMPLEMENTATION

  template<typename T>
  void save_override(T& t)
  {
    save_override_impl(t, typename supports_stapl_packing<T>::type{});
  }

  friend class boost::archive::detail::interface_oarchive<size_oarchive>;
  friend class boost::archive::basic_binary_oarchive<size_oarchive>;
  friend class boost::archive::basic_binary_oprimitive<
                 size_oarchive,
                 std::ostream::char_type,
                 std::ostream::traits_type>;
  friend class boost::archive::save_access;

public:
  size_oarchive(void)
  : base2_type(*this, boost_serialization_flags)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of bytes required to pack the object.
  //////////////////////////////////////////////////////////////////////
  std::streamsize size(void) const noexcept
  { return written_bytes(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Archive to determine if the object meets the packing requirements.
///
/// For more information, visit
/// http://www.boost.org/doc/libs/release/libs/serialization/doc/index.html .
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
class meets_requirements_oarchive
: private buffer_archive_impl::size_stream,
  public boost::archive::binary_oarchive_impl<
           meets_requirements_oarchive,
           std::ostream::char_type,
           std::ostream::traits_type
         >
{
private:
  using base1_type = buffer_archive_impl::size_stream;
  using base2_type = boost::archive::binary_oarchive_impl<
                       meets_requirements_oarchive,
                       std::ostream::char_type,
                       std::ostream::traits_type>;

  const typer::pass_type m_pass;
  bool                   m_meets_requirements;

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines the packing requirements for Boost.Serialization handled
  ///        types.
  ///
  /// The packing requirements cannot be changed by any object handled by
  /// Boost.Serialization, as they cannot have references or pointers to
  /// distributed objects without going through the @ref typer.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void save_override_impl(T& t, std::false_type)
  { base2_type::save_override(t); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Calculates the size required for STAPL serialization handled types.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void save_override_impl(T& t, std::true_type)
  {
    using storage_type = packed_object_storage<T>;
    using U            = typename std::remove_cv<T>::type;
    using traits_type  = typer_traits<U>;

    const auto r = traits_type::meets_requirements(m_pass, const_cast<U&>(t));
    if (!r.first)
      m_meets_requirements = false;

    const std::size_t size = (sizeof(storage_type) + aligned_size(r.second));
    advance(size);
  }

#define ARCHIVE_IGNORE_IMPLEMENTATION(T) void save_override(T const&) { }
  ARCHIVE_IGNORE_IMPLEMENTATION(boost::archive::version_type)
  ARCHIVE_IGNORE_IMPLEMENTATION(boost::archive::tracking_type)
#undef ARCHIVE_IGNORE_IMPLEMENTATION

  template<typename T>
  void save_override(T& t)
  {
    save_override_impl(t, typename supports_stapl_packing<T>::type{});
  }

  friend
  class boost::archive::detail::interface_oarchive<meets_requirements_oarchive>;
  friend
  class boost::archive::basic_binary_oarchive<meets_requirements_oarchive>;
  friend class boost::archive::basic_binary_oprimitive<
                 meets_requirements_oarchive,
                 std::ostream::char_type,
                 std::ostream::traits_type>;
  friend class boost::archive::save_access;

public:
  explicit meets_requirements_oarchive(const typer::pass_type p)
  : base2_type(*this, boost_serialization_flags),
    m_pass(p),
    m_meets_requirements(true)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the requirements for the object.
  //////////////////////////////////////////////////////////////////////
  std::pair<bool, std::size_t> meets_requirements(void) const noexcept
  { return std::make_pair(m_meets_requirements, written_bytes()); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Archive to pack an object to a stream.
///
/// For more information, visit
/// http://www.boost.org/doc/libs/release/libs/serialization/doc/index.html .
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
class buffer_oarchive
: private buffer_archive_impl::output_stream,
  public boost::archive::binary_oarchive_impl<
           buffer_oarchive,
           std::ostream::char_type,
           std::ostream::traits_type
         >
{
private:
  using base1_type = buffer_archive_impl::output_stream;
  using base2_type = boost::archive::binary_oarchive_impl<
                       buffer_oarchive,
                       std::ostream::char_type,
                       std::ostream::traits_type>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Packs Boost.Serialization handled types.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void save_override_impl(T const& t, std::false_type)
  { base2_type::save_override(t); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Packs STAPL serialization handled types.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void save_override_impl(T const& t, std::true_type)
  {
    using storage_type = packed_object_storage<T>;
    std::size_t size = sizeof(storage_type);
    ::new(address()) storage_type{t, address(), size};
    advance(size);
  }

#define ARCHIVE_IGNORE_IMPLEMENTATION(T) void save_override(T const&) { }
  ARCHIVE_IGNORE_IMPLEMENTATION(boost::archive::version_type)
  ARCHIVE_IGNORE_IMPLEMENTATION(boost::archive::tracking_type)
#undef ARCHIVE_IGNORE_IMPLEMENTATION

  template<typename T>
  void save_override(T const& t)
  {
    save_override_impl(t, typename supports_stapl_packing<T>::type{});
  }

  friend class boost::archive::detail::interface_oarchive<buffer_oarchive>;
  friend class boost::archive::basic_binary_oarchive<buffer_oarchive>;
  friend class boost::archive::basic_binary_oprimitive<
                 buffer_oarchive,
                 std::ostream::char_type,
                 std::ostream::traits_type>;
  friend class boost::archive::save_access;

public:
  explicit buffer_oarchive(void* const buffer)
  : base1_type(buffer),
    base2_type(*this, boost_serialization_flags)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of bytes used to pack the object.
  //////////////////////////////////////////////////////////////////////
  std::size_t size(void) const noexcept
  { return written_bytes(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Archive to unpack an object from a stream.
///
/// For more information, visit
/// http://www.boost.org/doc/libs/release/libs/serialization/doc/index.html .
///
/// @ingroup serializationImpl
//////////////////////////////////////////////////////////////////////
class buffer_iarchive
: private buffer_archive_impl::input_stream,
  public boost::archive::binary_iarchive_impl<
           buffer_iarchive,
           std::istream::char_type,
           std::istream::traits_type
         >
{
private:
  using base1_type = buffer_archive_impl::input_stream;
  using base2_type = boost::archive::binary_iarchive_impl<
                       buffer_iarchive,
                       std::istream::char_type,
                       std::istream::traits_type>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Unpacks Boost.Serialization handled types.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void load_override_impl(T& t, std::false_type)
  { base2_type::load_override(t); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unpacks STAPL serialization handled types.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void load_override_impl(T& t, std::true_type)
  {
    using storage_type = packed_object_storage<T>;
    t.~T();
    std::size_t size = sizeof(storage_type);
    auto& s = *static_cast<storage_type*>(address());
    ::new(std::addressof(t)) T(s.get(address(), size));
    s.~storage_type();
    advance(size);
  }

#define ARCHIVE_IGNORE_IMPLEMENTATION(T) void load_override(T&) { }
  ARCHIVE_IGNORE_IMPLEMENTATION(boost::archive::version_type)
  ARCHIVE_IGNORE_IMPLEMENTATION(boost::archive::tracking_type)
#undef ARCHIVE_IGNORE_IMPLEMENTATION

  template<typename T>
  void load_override(T& t)
  {
    load_override_impl(t, typename supports_stapl_packing<T>::type{});
  }

  friend class boost::archive::detail::interface_iarchive<buffer_iarchive>;
  friend class boost::archive::basic_binary_iarchive<buffer_iarchive>;
  friend class boost::archive::basic_binary_iprimitive<
                 buffer_iarchive,
                 std::istream::char_type,
                 std::istream::traits_type>;
  friend class boost::archive::load_access;

public:
  explicit buffer_iarchive(void* const buffer)
  : base1_type(buffer),
    base2_type(*this, boost_serialization_flags)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of bytes read while unpacking the object.
  //////////////////////////////////////////////////////////////////////
  std::size_t size(void) const noexcept
  { return read_bytes(); }
};

} // namespace runtime

} // namespace stapl


BOOST_SERIALIZATION_REGISTER_ARCHIVE(stapl::runtime::size_oarchive)
BOOST_SERIALIZATION_REGISTER_ARCHIVE(stapl::runtime::meets_requirements_oarchive)
BOOST_SERIALIZATION_REGISTER_ARCHIVE(stapl::runtime::buffer_oarchive)
BOOST_SERIALIZATION_REGISTER_ARCHIVE(stapl::runtime::buffer_iarchive)
BOOST_SERIALIZATION_USE_ARRAY_OPTIMIZATION(stapl::runtime::size_oarchive)
BOOST_SERIALIZATION_USE_ARRAY_OPTIMIZATION(stapl::runtime::meets_requirements_oarchive)
BOOST_SERIALIZATION_USE_ARRAY_OPTIMIZATION(stapl::runtime::buffer_oarchive)
BOOST_SERIALIZATION_USE_ARRAY_OPTIMIZATION(stapl::runtime::buffer_iarchive)


// --------------------------------------------------------------------------
// TYPER_TRAITS SPECIALIZATION
// --------------------------------------------------------------------------

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for types handled by
///        Boost.Serialization.
///
/// @tparam T Object type to be packed.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
class typer_traits<T,
                   typename std::enable_if<
                     !runtime::supports_stapl_packing<T>::value
                   >::type>
{
private:
  static_assert(sizeof(T)>=sizeof(std::size_t),
                "Insufficient space to store pointer to packed object.");

public:
  using value_type = T;

  static std::size_t packed_size(T const& t) noexcept
  {
    runtime::size_oarchive sa;
    sa << t;
    return sa.size();
  }

  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p, T const& t) noexcept
  {
    runtime::meets_requirements_oarchive mra{p};
    mra << t;
    return mra.meets_requirements();
  }

  static void prepack(T*, T const*, const std::size_t = 1) noexcept
  { }

  static std::size_t pack(T& dest,
                          void* base,
                          const std::size_t offset,
                          T const& src) noexcept
  {
    std::memcpy(static_cast<void*>(std::addressof(dest)),
                &offset,
                sizeof(offset));
    runtime::buffer_oarchive oa(static_cast<char*>(base) + offset);
    oa << src;
    return oa.size();
  }

  static std::size_t unpack(T& t, void* base)
  {
    std::size_t offset = 0;
    std::memcpy(&offset,
                static_cast<void*>(std::addressof(t)),
                sizeof(offset));
    ::new(std::addressof(t)) T;
    runtime::buffer_iarchive ia(static_cast<char*>(base) + offset);
    ia >> t;
    return ia.size();
  }

  static void destroy(T& t) noexcept
  { t.~T(); }
};

} // namespace stapl

#endif
