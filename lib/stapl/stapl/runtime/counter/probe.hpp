/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_PROBE_HPP
#define STAPL_RUNTIME_COUNTER_PROBE_HPP

#include "../runtime_fwd.hpp"
#include <map>
#include <mutex>
#include <initializer_list>
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief The @c probe is used to profile sections of code and dump the
///        measurements to @c std::cout.
///
/// You can use the probe like this:
/// @code
/// typedef stapl::probe<stapl::counter<stapl::default_timer>> probe_type;
///
/// void foo(void)
/// {
///   probe_type p = probe_type::get(__func__, "'s probe"); // foo's probe
///   // some code
///   {
///     probe_type p = probe_type::get("goo's probe");
///     goo();
///   }
/// }
/// @endcode
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
template<typename T, typename Tag = void>
class probe
{
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Probe registry.
  ///
  /// Prints all probes to @c std::cout upon finalization.
  //////////////////////////////////////////////////////////////////////
  struct probe_registry
  {
    typedef std::map<
              std::string, T
            > counter_container_type;
    typedef std::map<
              std::thread::id, counter_container_type
            > container_type;

    process_id     pid;
    container_type counters;
    std::mutex     mtx;

    probe_registry(void)
    : pid(get_process_id())
    { }

    ~probe_registry(void)
    {
      if (!counters.empty()) {
        std::ostringstream oss;
        dump(oss);
        std::cout << oss.str();
      }
    }

    void dump(std::ostream& os)
    {
      std::lock_guard<std::mutex> lock{mtx};
      for (auto const& c : counters) {
        os << '[' << pid << ", " << c.first << "]:\n";
        for (auto const& t : c.second)
          os << '\t' << probe{t.first, t.second} << '\n';
      }
    }

    void clear(void)
    {
      std::lock_guard<std::mutex> lock{mtx};
      counters.clear();
    }

    T& get(const std::thread::id tid, std::string const& s)
    {
      counter_container_type* c = nullptr;
      {
        std::lock_guard<std::mutex> lock{mtx};
        if (pid==invalid_process_id)
          pid = get_process_id();
        c = &(counters[tid]);
      }
      return (*c)[s];
    }
  };

  /// Probes of the current process.
  static probe_registry s_probes;

  std::string m_name;
  T*          m_t;
  bool        m_stop;

  probe(std::string const& name, T& t, const bool start)
  : m_name(name),
    m_t(&t),
    m_stop(start)
  {
    if (start)
      m_t->start();
  }

  probe(std::string const& name, T const& t) noexcept
  : m_name(name),
    m_t(&const_cast<T&>(t)),
    m_stop(false)
  { }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an empty @ref probe object that does nothing.
  //////////////////////////////////////////////////////////////////////
  probe(void)
  : m_t(nullptr),
    m_stop(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys the @ref probe object, stopping the corresponding counter.
  //////////////////////////////////////////////////////////////////////
  ~probe(void)
  {
    if (m_stop)
      m_t->stop();
  }

  probe(probe&& other)
  : m_t(nullptr),
    m_stop(false)
  { swap(other); }

  probe& operator=(probe&& other)
  {
    if (this!=&other) {
      if (m_stop)
        m_t->stop();
      swap(other);
      probe{}.swap(other);
    }
    return *this;
  }

  probe(probe const&) = delete;
  probe& operator=(probe const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the probe.
  //////////////////////////////////////////////////////////////////////
  void stop(void)
  {
    m_t->stop();
    m_stop = false;
  }

  void swap(probe& other) noexcept
  {
    using std::swap;
    swap(m_name, other.m_name);
    swap(m_t, other.m_t);
    swap(m_stop, other.m_stop);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a probe with the given name.
  //////////////////////////////////////////////////////////////////////
  static probe get(std::string const& name)
  {
    return probe{name, s_probes.get(std::this_thread::get_id(), name), true};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a probe with the given prefix and name.
  //////////////////////////////////////////////////////////////////////
  template<typename... U>
  static probe get(U const&... u)
  {
    std::ostringstream os;
    (void)std::initializer_list<int>{(os << u, 0)...};
    return get(os.str());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears all probes.
  //////////////////////////////////////////////////////////////////////
  static void clear(void)
  {
    s_probes.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Prints all probes.
  //////////////////////////////////////////////////////////////////////
  static std::ostream& dump(std::ostream& os)
  {
    s_probes.dump(os);
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, probe const& p)
  {
    return os << p.m_name       << ' '
              << p.m_t->calls() << ' '
              << p.m_t->value();
  }
};
template<typename T, typename Tag>
typename probe<T, Tag>::probe_registry probe<T, Tag>::s_probes;


template<typename T, typename Tag>
void swap(probe<T, Tag>& x, probe<T, Tag>& y) noexcept
{
  x.swap(y);
}

} // namespace stapl

#endif
