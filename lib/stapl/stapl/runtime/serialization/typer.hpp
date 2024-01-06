/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_TYPER_HPP
#define STAPL_RUNTIME_SERIALIZATION_TYPER_HPP

#include "typer_fwd.hpp"
#include "typer_traits.hpp"
#include "../exception.hpp"
#include "../type_traits/aligned_storage.hpp"
#include "../type_traits/is_basic.hpp"
#include <cstdint>
#include <cstring>
#include <memory>

namespace stapl {

template<typename T>
typer::typer(T& dest, T const& src, void* base, const std::size_t offset)
: m_pass(PACK),
  m_offset(offset),
  m_base(static_cast<char*>(base)),
  m_dest(reinterpret_cast<const char*>(std::addressof(dest))),
  m_src(reinterpret_cast<const char*>(std::addressof(src))),
  m_sizeof(sizeof(T))
{ }


template<typename T>
void typer::member(T& t)
{
  using runtime::aligned_size;

  if (is_basic<T>::value)
    return;

  using traits_type = typer_traits<T>;

  switch (m_pass) {
    case SIZE:
      m_offset += aligned_size(traits_type::packed_size(t));
      break;

    case COPY:
    case MOVE:
    case NO_MARSHAL: {
      const auto r = traits_type::meets_requirements(m_pass, t);
      m_offset    += aligned_size(r.second);
      if (!r.first)
        m_meets_requirements = false;
    } break;

    case PACK: {
      // pre-packing already done
      // find position of t in the object that contains it
      const std::ptrdiff_t dist =
        (reinterpret_cast<const char*>(std::addressof(t)) - m_dest);
      STAPL_RUNTIME_ASSERT(dist>=0);
      // source object for t (inside source parent object)
      const T* const src_t = reinterpret_cast<const T*>(m_src + dist);
      // pack member
      m_offset += aligned_size(traits_type::pack(t, m_base, m_offset, *src_t));
    } break;

    case UNPACK:
      m_offset += aligned_size(traits_type::unpack(t, m_base));
      break;

    case DESTROY:
      traits_type::destroy(t);
      break;

    default:
      STAPL_RUNTIME_ERROR("Incorrect typer state.");
      break;
  }
}


template<typename T>
void typer::member(T& t, const std::size_t N)
{
  using runtime::aligned_size;

  if (is_basic<T>::value)
    return;

  using traits_type = typer_traits<T>;

  switch (m_pass) {
    case SIZE:
      m_offset += aligned_size(traits_type::packed_size(t, N));
      break;

    case COPY:
    case MOVE:
    case NO_MARSHAL: {
      const auto r = traits_type::meets_requirements(m_pass, t, N);
      m_offset    += aligned_size(r.second);
      if (!r.first)
        m_meets_requirements = false;
    } break;

    case PACK: {
      // pre-packing already done
      // find position of t in the object that contains it
      const std::ptrdiff_t dist =
        (reinterpret_cast<const char*>(std::addressof(t)) - m_dest);
      STAPL_RUNTIME_ASSERT(dist>=0);
      // source object for t (inside source parent object)
      const T* const src_t = reinterpret_cast<const T*>(m_src + dist);
      // pack member
      m_offset +=
        aligned_size(traits_type::pack(t, m_base, m_offset, *src_t, N));
    } break;

    case UNPACK:
      m_offset += aligned_size(traits_type::unpack(t, m_base, N));
      break;

    case DESTROY:
      traits_type::destroy(t, N);
      break;

    default:
      STAPL_RUNTIME_ERROR("Incorrect typer state.");
      break;
  }
}


template<typename T>
void typer::transient(T& t)
{
  switch (m_pass) {
    case SIZE:
      // nothing to do
      break;

    case COPY:
    case MOVE:
    case NO_MARSHAL:
      m_meets_requirements = false;
      break;

    case PACK:
      // nothing to do
      break;

    case UNPACK:
      ::new(std::addressof(t)) T{};
      break;

    case DESTROY:
      t.~T();
      break;

    default:
      STAPL_RUNTIME_ERROR("Incorrect typer state.");
      break;
  }
}


template<typename T, typename U>
void typer::transient(T& t, U&& u)
{
  switch (m_pass) {
    case SIZE:
      // nothing to do
      break;

    case COPY:
    case MOVE:
    case NO_MARSHAL:
      m_meets_requirements = false;
      break;

    case PACK:
      // nothing to do
      break;

    case UNPACK:
      ::new(std::addressof(t)) T(std::forward<U>(u));
      break;

    case DESTROY:
      t.~T();
      break;

    default:
      STAPL_RUNTIME_ERROR("Incorrect typer state.");
      break;
  }
}


template<typename T, typename U>
void typer::pointer_to_member(T*& t, U* ref, const std::size_t offset) noexcept
{
  switch (m_pass) {
    case SIZE:
    case COPY:
    case MOVE:
    case NO_MARSHAL:
      // nothing to do
      break;

    case PACK:
      if (!ref) {
        // alias to nullptr
        t = nullptr;
      }
      else if ( (t >= reinterpret_cast<const T*>(m_src)) &&
                (t <  reinterpret_cast<const T*>(m_src + m_sizeof)) ) {
        // pointer to local member
        U* p = (ref + offset);
        const std::intptr_t dist = reinterpret_cast<const char*>(p) - m_base;
        std::memcpy(&t, &dist, sizeof(dist));
      }
      else if (t) {
        // pointer to dynamic member
        t = (ref + offset);
      }
      break;

    case UNPACK:
      if (t) {
        std::intptr_t dist = 0;
        std::memcpy(&dist, &t, sizeof(dist));
        t = reinterpret_cast<T*>(m_base + dist);
      }
      break;

    case DESTROY:
      break;

    default:
      STAPL_RUNTIME_ERROR("Incorrect typer state.");
      break;
  }
}

} // namespace stapl

#endif
