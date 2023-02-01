/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_RMI_REQUEST_HPP
#define STAPL_RUNTIME_REQUEST_RMI_REQUEST_HPP

#include <cstddef>

namespace stapl {

namespace runtime {

class context;


//////////////////////////////////////////////////////////////////////
/// @brief Encapsulates an RMI request for subsequent execution via the
///        function operator.
///
/// @ref rmi_request objects package all information for buffering, transfer and
/// execution. The 'header' contains the size of the request. The 'body'
/// (derived class) has the desired function and any arguments required to
/// invoke it.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class rmi_request
{
public:
  typedef std::size_t size_type;

  static constexpr size_type minimum_size(void) noexcept
  { return sizeof(rmi_request); }

private:
  size_type m_size;

public:
  constexpr explicit rmi_request(const size_type size) noexcept
  : m_size(size)
  { }

  rmi_request(rmi_request const&) = delete;
  rmi_request& operator=(rmi_request const&) = delete;

protected:
  ~rmi_request(void) = default;

public:
  constexpr size_type size(void) const noexcept
  { return m_size; }

  size_type& size(void) noexcept
  { return m_size; }

  virtual bool operator()(context&) = 0;
};


#if !defined(STAPL_RUNTIME_DISABLE_COMBINING)

//////////////////////////////////////////////////////////////////////
/// @brief Information about the size of combined requests.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class combined_request_size
{
public:
  typedef std::size_t size_type;

private:
  /// Static size of non-combined request.
  const size_type m_full;
  /// Static size of combined request.
  const size_type m_combined;
  /// Dynamic size of request (i.e. dynamic size of arguments).
  const size_type m_dynamic;

public:
  constexpr combined_request_size(size_type full_sz,
                                  size_type combined_sz,
                                  size_type dynamic_sz) noexcept
  : m_full(full_sz),
    m_combined(combined_sz),
    m_dynamic(dynamic_sz)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the static size of the combined request.
  //////////////////////////////////////////////////////////////////////
  constexpr size_type combined_static_size(void) const noexcept
  { return m_combined; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of the non-combined request.
  //////////////////////////////////////////////////////////////////////
  constexpr size_type non_combined_size(void) const noexcept
  { return (m_full + m_dynamic); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of the combined request.
  //////////////////////////////////////////////////////////////////////
  constexpr size_type combined_size(void) const noexcept
  { return (m_combined + m_dynamic); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Encapsulates an RMI request that supports combining for subsequent
///        execution via the function operator.
///
/// @tparam EmptyArgs Template parameter that if @c true informs that the
///                   request has no arguments or the arguments are empty
///                   classes.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<bool EmptyArgs>
class combinable_rmi_request
: public rmi_request
{
public:
  using rmi_request::size_type;

  /// @c true if the request has a counter for the combined requests.
  static constexpr bool has_counter = false;

  constexpr explicit combinable_rmi_request(const size_type size) noexcept
  : rmi_request(size)
  { }

protected:
  ~combinable_rmi_request(void) = default;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Informs that a request has been combined by increasing the size of
  ///        first request.
  //////////////////////////////////////////////////////////////////////
  void combined(const size_type size) noexcept
  { this->size() += size; }

  constexpr size_type num_requests(void) const noexcept
  { return 0; }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref combinable_rmi_request for requests with no or
///        empty arguments.
///
/// A counter is provided to keep count of the combined requests.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<>
class combinable_rmi_request<true>
: public rmi_request
{
public:
  using rmi_request::size_type;

  static constexpr bool has_counter = true;

private:
  size_type m_requests;

public:
  constexpr explicit combinable_rmi_request(const size_type size) noexcept
  : rmi_request(size),
    m_requests(1)
  { }

protected:
  ~combinable_rmi_request(void) = default;

public:
  void combined(const size_type) noexcept
  { ++m_requests; }

  constexpr size_type num_requests(void) const noexcept
  { return m_requests; }
};

#endif

} // namespace runtime

} // namespace stapl

#endif
