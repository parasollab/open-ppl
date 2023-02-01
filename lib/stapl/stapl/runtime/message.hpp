/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_MESSAGE_HPP
#define STAPL_RUNTIME_MESSAGE_HPP

#include "config.hpp"
#include "context_id.hpp"
#include "exception.hpp"
#include "full_location.hpp"
#include "message_handle.hpp"
#include "request/header.hpp"
#include "type_traits/aligned_storage.hpp"
#include "utility/option.hpp"
#include "utility/ref_counted.hpp"
#include <memory>
#include <new>
#include <type_traits>
#include <boost/intrusive/slist.hpp>
#include <boost/range/iterator_range_core.hpp>

namespace stapl {

namespace runtime {

class message;


//////////////////////////////////////////////////////////////////////
/// @brief Deleter for @ref message objects.
///
/// @see message
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
struct message_delete
{
  void operator()(message*) const;
};


//////////////////////////////////////////////////////////////////////
/// @brief @ref message unique pointer.
///
/// @see message
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
using message_ptr = std::unique_ptr<message, message_delete>;


//////////////////////////////////////////////////////////////////////
/// @brief Communication buffer.
///
/// A message consists of a standard, fixed header that gives information about
/// the size and the type of the message and the message body. The body contains
/// data relevant to communication, such as additional headers and/or payload.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class message
: public ref_counted<message, message_delete>
{
public:
  using payload_range_type = boost::iterator_range<char*>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Message header.
  ///
  /// The message size must always be the first member so that it can be
  /// transmitted independently of the rest of the header.
  //////////////////////////////////////////////////////////////////////
  struct header_type
  {
    std::size_t  m_size;
    header::type m_type;

    void reset(void) noexcept
    {
      m_size = 0;
      m_type = header::INVALID;
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes the memory for allocating @ref message objects.
  ///
  /// The following options are supported through @p opts:
  /// -# @c STAPL_RUNTIME_MSG_SIZE to change the default capacity.
  /// -# @c STAPL_RUNTIME_MSG_POOL_DISABLE to disable pool @ref message
  ///    allocation.
  /// -# @c STAPL_RUNTIME_MSG_POOL_MIN_NUM Minimum number of allocated
  ///    @ref message objects when pool allocation is enabled.
  /// -# @c STAPL_RUNTIME_MSG_MAX_MEM the maximum fraction of memory to use when
  ///    pool allocation is enabled.
  ///
  /// @param opts Options to pass for initialization.
  /// @param nppn Number of processes per node.
  //////////////////////////////////////////////////////////////////////
  static void initialize(option const& opts, const unsigned int nppn);

  //////////////////////////////////////////////////////////////////////
  /// @brief Finalizes the memory for allocating @ref message objects.
  //////////////////////////////////////////////////////////////////////
  static void finalize(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the default capacity of the internal buffer (header and
  ///        body) in bytes.
  //////////////////////////////////////////////////////////////////////
  static std::size_t default_capacity(void) noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of the header in bytes.
  //////////////////////////////////////////////////////////////////////
  static std::size_t header_size(void) noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the default capacity for the body in bytes.
  //////////////////////////////////////////////////////////////////////
  static void set_default_body_capacity(const std::size_t);

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the default capacity for the body in bytes.
  //////////////////////////////////////////////////////////////////////
  static std::size_t default_body_capacity(void) noexcept
  { return (default_capacity() - header_size()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a @ref message of default capacity.
  //////////////////////////////////////////////////////////////////////
  static message* construct(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a @ref message of with capacity of @p n bytes.
  ///
  /// @param n          Requested capacity in bytes.
  /// @param exact_size @c true if there is no benefit to providing extra
  ///                   capacity.
  //////////////////////////////////////////////////////////////////////
  static message* construct(const std::size_t n, const bool exact_size);

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys @p msg.
  //////////////////////////////////////////////////////////////////////
  static void destroy(message* msg);

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a @ref message that can store at least @p n bytes.
  ///
  /// @param n          Requested capacity in bytes.
  /// @param exact_size @c true if there is no benefit to providing extra
  ///                   capacity.
  //////////////////////////////////////////////////////////////////////
  static message_ptr create(const std::size_t n, const bool exact_size = true)
  { return message_ptr{construct((header_size() + n), exact_size)}; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a @ref message that can store at least @p n bytes.
  ///
  /// @param h          Message type.
  /// @param n          Requested capacity in bytes.
  /// @param exact_size @c true if there is no benefit to providing extra
  ///                   capacity.
  //////////////////////////////////////////////////////////////////////
  static message_ptr create(const header::type h,
                            const std::size_t n,
                            const bool exact_size = true)
  {
    const std::size_t size = (header_size() + n);

    message* const m   = construct(size, exact_size);
    m->header().m_type = h;

    return message_ptr{m};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a @ref message that can store at least @p n bytes.
  ///
  /// @param h          Message type.
  /// @param n          Requested capacity in bytes.
  /// @param exh        Extended header.
  /// @param exact_size @c true if there is no benefit to providing extra
  ///                   capacity.
  //////////////////////////////////////////////////////////////////////
  template<typename ExtendedHeader>
  static message_ptr create(const header::type h,
                            const std::size_t n,
                            ExtendedHeader&& exh,
                            const bool exact_size = true)
  {
    using extended_header_type = typename std::decay<ExtendedHeader>::type;

    const std::size_t ex_hdr_size =
      sizeof(aligned_storage_t<sizeof(extended_header_type)>);
    const std::size_t size = (header_size() + ex_hdr_size + n);

    message* const m    = construct(size, exact_size);
    m->header().m_type  = h;
    m->header().m_size += ex_hdr_size;

    m->m_payload_offset = ex_hdr_size;
    new(m->body()) extended_header_type(std::forward<ExtendedHeader>(exh));

    return message_ptr{m};
  }

private:
  /// Capacity for message body.
  const std::size_t m_body_capacity;
  /// Payload offset in message body, for example due to extended header.
  std::size_t       m_payload_offset;
  /// @c true if the @ref message is using memory managed by the runtime.
  const bool        m_managed;
  /// Handle for the communication layer.
  message_handle    m_handle;
  /// @c true if the @ref message has been forwarded.
  bool              m_forwarded;

public:
  using slist_hook_type = boost::intrusive::slist_member_hook<>;

  /// Hook for @c boost::intrusive::slist
  slist_hook_type   slist_hook;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref message with default capacity.
  ///
  /// @param managed @c true if the @ref message is managed by the runtime.
  //////////////////////////////////////////////////////////////////////
  explicit message(const bool managed) noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref message of arbitrary capacity for the
  ///        payload.
  ///
  /// @param n Capacity for the payload in bytes.
  //////////////////////////////////////////////////////////////////////
  explicit message(const std::size_t n) noexcept;

  ~message(void);

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Resets this @ref message for reuse.
  //////////////////////////////////////////////////////////////////////
  void reset(void) noexcept;

  message_handle const& handle(void) const noexcept
  { return m_handle; }

  message_handle& handle(void) noexcept
  { return m_handle; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer to the internal buffer.
  ///
  /// Flexible array members are not supported in C++. This function returns
  /// a pointer past the end of this @ref message object which is properly
  /// sized and aligned to store requests in it.
  //////////////////////////////////////////////////////////////////////
  char* data(void) noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc data() const noexcept
  //////////////////////////////////////////////////////////////////////
  const char* data(void) const noexcept;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the capacity of the internal buffer in bytes.
  //////////////////////////////////////////////////////////////////////
  std::size_t capacity(void) const noexcept
  { return (header_size() + body_capacity()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of the internal buffer in bytes.
  //////////////////////////////////////////////////////////////////////
  std::size_t size(void) const noexcept
  { return (header_size() + body_size()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the size is bigger than the default capacity.
  //////////////////////////////////////////////////////////////////////
  bool is_long(void) const noexcept
  { return (size() > default_capacity()); }

  header_type const& header(void) const noexcept
  { return *reinterpret_cast<header_type const*>(data()); }

  header_type& header(void) noexcept
  { return *reinterpret_cast<header_type*>(data()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the type of the message.
  //////////////////////////////////////////////////////////////////////
  header::type type(void) const noexcept
  { return header().m_type; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Changes the type of the message.
  //////////////////////////////////////////////////////////////////////
  void retarget(const header::type t) noexcept
  { header().m_type = t; }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer to the body of the message.
  //////////////////////////////////////////////////////////////////////
  char* body(void) noexcept
  { return (data() + header_size()); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc body(void) noexcept
  //////////////////////////////////////////////////////////////////////
  const char* body(void) const noexcept
  { return (data() + header_size()); }

  std::size_t body_capacity(void) const noexcept
  { return m_body_capacity; }

  std::size_t body_size(void) const noexcept
  { return header().m_size; }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if there is no message body.
  //////////////////////////////////////////////////////////////////////
  bool body_empty(void) const noexcept
  { return (body_size()==0); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the offset to the payload and returns the extended header.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  T const& fix_payload_offset(void) noexcept
  {
    m_payload_offset = sizeof(aligned_storage_t<sizeof(T)>);
    return *reinterpret_cast<T const*>(body());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the extended header.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  T const& get_extended_header(void) const noexcept
  { return *reinterpret_cast<T const*>(body()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a range over the payload.
  //////////////////////////////////////////////////////////////////////
  payload_range_type payload(void) noexcept
  {
    char* const first = (body() + m_payload_offset);
    char* const last  = (body() + body_size());
    return boost::make_iterator_range(first, last);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Available space for the payload in bytes.
  //////////////////////////////////////////////////////////////////////
  std::size_t available_space(void) const noexcept
  { return (body_capacity() - body_size()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer to @p n bytes reserved in the internal buffer.
  //////////////////////////////////////////////////////////////////////
  void* reserve(const std::size_t n) noexcept
  {
    STAPL_RUNTIME_ASSERT(n<=available_space());
    char* const p    = (body() + body_size());
    header().m_size += n;
    return p;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer to @p n bytes reserved in the internal buffer or
  ///        @c nullptr if there is not enough space.
  //////////////////////////////////////////////////////////////////////
  void* try_reserve(const std::size_t n) noexcept
  { return (n<=available_space() ? reserve(n) : nullptr); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if this @ref message has been forwarded.
  //////////////////////////////////////////////////////////////////////
  bool is_forwarded(void) const noexcept
  { return m_forwarded; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Marks that this @ref message has been forwarded.
  //////////////////////////////////////////////////////////////////////
  void mark_forwarded(void) noexcept
  {
    STAPL_RUNTIME_ASSERT(!m_forwarded);
    m_forwarded = true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an identical copy of this @ref message.
  //////////////////////////////////////////////////////////////////////
  message_ptr clone(void) const;
};


// Destroys the given message
inline void message_delete::operator()(message* m) const
{
  message::destroy(m);
}


//////////////////////////////////////////////////////////////////////
/// @brief Intrusive @ref message shared pointer.
///
/// @see message
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class message_shared_ptr
: public boost::intrusive_ptr<message>
{
public:
  message_shared_ptr(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref message_shared_ptr from @p m with reference
  ///        count @c 1.
  //////////////////////////////////////////////////////////////////////
  explicit message_shared_ptr(message_ptr&& m)
  : boost::intrusive_ptr<message>(m.release())
  { STAPL_RUNTIME_ASSERT(this->get()->unique()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref message_shared_ptr from @p m with reference
  ///        count @p ref_count.
  //////////////////////////////////////////////////////////////////////
  message_shared_ptr(message_ptr&& m, const long ref_count)
  : boost::intrusive_ptr<message>(m.release(), false)
  {
    STAPL_RUNTIME_ASSERT(this->get()->use_count()==0);
    this->get()->add_ref(ref_count);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref message_shared_ptr from @p m.
  ///
  /// If @p add_ref is @c true, then the reference count is incremented.
  //////////////////////////////////////////////////////////////////////
  explicit message_shared_ptr(message* const m, const bool add_ref = true)
  : boost::intrusive_ptr<message>(m, add_ref)
  { STAPL_RUNTIME_ASSERT(this->get()->use_count()!=0); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a @ref message_ptr from @p m.
///
/// @warning This function checks the reference count and creates a clone of
///          @p m if it is not @c 1. This may cause performance degradation if
//           it is known that the reference count is always @c 1.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
inline message_ptr make_message_ptr(message* const m)
{
  const auto ref_count = m->use_count();

  if (ref_count==0) {
    // never shared, return it immediately
    return message_ptr{m};
  }

  if (ref_count==1) {
    // last owner, reset reference count to convert to message_ptr
    m->remove_ref();
    return message_ptr{m};
  }

  // multiple owners, clone; release() will delete the initial message if ref
  // count reached 0
  message_ptr p{m->clone()};
  m->release();
  return p;
}


//////////////////////////////////////////////////////////////////////
/// @brief Intrusive @ref message list hook.
///
/// @see message, message_slist
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
using message_hook_type = boost::intrusive::member_hook<
                            message,
                            message::slist_hook_type,
                            &message::slist_hook>;


//////////////////////////////////////////////////////////////////////
/// @brief Intrusive @ref message list for @ref message pointers.
///
/// @see message, message_slist_hook
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
using message_raw_ptr_slist = typename boost::intrusive::make_slist<
                                message,
                                message_hook_type,
                                boost::intrusive::cache_last<true>,
                                boost::intrusive::constant_time_size<false>
                              >::type;


//////////////////////////////////////////////////////////////////////
/// @brief Intrusive @ref message list.
///
/// @see message, message_slist_hook
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class message_slist
{
private:
  message_raw_ptr_slist m_list;

public:
  bool empty(void) const noexcept
  { return m_list.empty(); }

  void push_back(message* p) noexcept
  {
    STAPL_RUNTIME_ASSERT(p->use_count()==0);
    m_list.push_back(*p);
  }

  void push_back(message_ptr m) noexcept
  { m_list.push_back(*(m.release())); }

  message& front(void) noexcept
  { return m_list.front(); }

  message_ptr pop_front(void) noexcept
  {
    message_ptr m{&(m_list.front())};
    m_list.pop_front();
    return m;
  }
};

} // namespace runtime

} // namespace stapl

#endif
