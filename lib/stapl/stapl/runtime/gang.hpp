/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_GANG_HPP
#define STAPL_RUNTIME_GANG_HPP

#include "context.hpp"
#include "exception.hpp"
#include "yield.hpp"
#include "utility/comparable_proxy.hpp"
#include <functional>
#include <memory>
#include <utility>
#include <boost/optional.hpp>
#include <boost/utility/typed_in_place_factory.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Creates a new gang by partitioning the existing one from which the
///        @ref gang construction is invoked.
///
/// A new @ref gang object has to be created in all locations that want to
/// participate in it. It is the user's responsibility to ensure that.
///
/// The new gang is described by its size and two functions. The first function
/// takes as input the id of a location in the creator gang and returns the id
/// of a location in the created gang. The second function takes as input the id
/// of a function in the created gang and returns the id of a location in the
/// creator gang.
///
/// For example, creating a gang that includes all the even numbered locations
/// of a gang can be done as follows:
/// @code
/// struct even_fun
/// {
///   unsigned int operator()(unsigned int n) const
///   { return (n/2); }
/// };
///
/// struct reverse_even_fun
/// {
///   unsigned int operator()(unsigned int n) const
///   { return (2*n); }
/// };
///
/// int stapl_main(int, char**)
/// {
///   if (stapl::get_location_id()%2==0) {
///    stapl::gang g(std::ceil(stapl::get_num_locations()/2.0),
///                  even_fun(),
///                  reverse_even_fun());
///     // gang created
///     // user code
///   } // gang is destroyed
///   return 0;
/// }
/// @endcode
///
/// @ingroup ARMI
///
/// @bug While you can create a gang with a given id, this will not always work
///      correctly; you have to give a suitable id and you have to make sure
///      that the gang is destroyed before you try to use it again. There is no
///      runtime primitive for the latter.
//////////////////////////////////////////////////////////////////////
class gang
{
public:
  using size_type = runtime::gang_description::size_type;
private:
  enum status_type
  {
    /// No cleanup is needed.
    INACTIVE,
    /// Requires cleanup.
    ACTIVE,
    /// Requires flushing.
    SWITCHED
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new gang without an id hint.
  //////////////////////////////////////////////////////////////////////
  static runtime::location_md& create(const runtime::gang_md::id,
                                      const runtime::location_md::id,
                                      runtime::gang_description,
                                      const runtime::location_md::id,
                                      runtime::comparable_proxy);

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new gang with the given id hint.
  //////////////////////////////////////////////////////////////////////
  static runtime::location_md& create(const std::size_t,
                                      const runtime::gang_md::id,
                                      const runtime::location_md::id,
                                      runtime::gang_description,
                                      const runtime::location_md::id,
                                      runtime::comparable_proxy);


  /// Placeholder for a @ref context object.
  boost::optional<runtime::context> m_context;
  status_type                       m_status;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new gang of one location.
  ///
  /// The creation of the metadata is deferred until any kind of object
  /// registration (@ref rmi_handle, @ref p_object) or communication is
  /// required.
  //////////////////////////////////////////////////////////////////////
  gang(void)
  : m_status(ACTIVE)
  {
    using namespace runtime;

    STAPL_RUNTIME_PROFILE("gang(deferred)", (primitive_traits::blocking |
                                             primitive_traits::environment));
    this_context::push_placeholder(m_context);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new gang.
  ///
  /// The creation of the metadata requires communication to set up the gang id,
  /// therefore the construction will only finish until the id is received.
  ///
  /// @param size Size of the new gang.
  /// @param mf   Mapping function to translate a location id from the creator
  ///             gang to a location id in the created gang.
  /// @param rf   Mapping function to translate a location id from the created
  ///             gang to a location id in the creator gang.
  //////////////////////////////////////////////////////////////////////
  template<typename MappingFunction, typename ResolutionFunction>
  gang(const size_type size,
       MappingFunction&& mf,
       ResolutionFunction&& rf)
  : m_status(ACTIVE)
  {
    using namespace runtime;

    STAPL_RUNTIME_ASSERT_MSG((size>0), "Size cannot be 0");

    auto* const ctx = this_context::try_get();
    if (!ctx) {
      // creating a new subgang from a non-initialized gang - defer creation
      STAPL_RUNTIME_PROFILE("gang(deferred)", (primitive_traits::blocking |
                                               primitive_traits::environment));
      STAPL_RUNTIME_ASSERT_MSG((size==1),
                               "Size cannot exceed that of parent gang");
      STAPL_RUNTIME_ASSERT_MSG((mf(0)==0), "Bad mapping function");
      STAPL_RUNTIME_ASSERT_MSG((rf(0)==0), "Bad resolution function");
      this_context::push_placeholder(m_context);
    }
    else {
      // creating a new subgang
      STAPL_RUNTIME_PROFILE("gang()", (primitive_traits::blocking |
                                       primitive_traits::environment));
      ctx->flush_requests();
      auto const& l   = ctx->get_location_md();
      auto const& g   = l.get_gang_md();
      const auto nlid = mf(l.get_id());
      STAPL_RUNTIME_ASSERT_MSG((size<=g.size()),
                               "Size cannot exceed that of parent gang");
      STAPL_RUNTIME_ASSERT_MSG((nlid<size), "Bad mapping function");
      STAPL_RUNTIME_ASSERT_MSG((rf(nlid)==l.get_id()),
                               "Bad resolution function");
      scheduling_point(*ctx);
      auto& nl = create(g.get_id(),
                        rf(0),
                        gang_description{g.get_description(), rf, size},
                        nlid,
                        comparable_proxy{std::make_pair(mf, rf)});
      m_context = boost::in_place<context>(std::ref(nl));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new gang.
  ///
  /// Since the id is given at construction, there is no communication required
  /// to set-up the gang id. However, the user is responsible for supplying an
  /// unused gang id.
  ///
  /// @param id   Id of the new gang.
  /// @param size Size of the new gang.
  /// @param mf   Mapping function to translate a location id from the creator
  ///             gang to a location id in the created gang.
  /// @param rf   Mapping function to translate a location id from the created
  ///             gang to a location id in the creator gang.
  //////////////////////////////////////////////////////////////////////
  template<typename MappingFunction, typename ResolutionFunction>
  gang(const std::size_t id,
       const size_type size,
       MappingFunction&& mf,
       ResolutionFunction&& rf)
  : m_status(ACTIVE)
  {
    using namespace runtime;

    STAPL_RUNTIME_ASSERT_MSG((size>0), "Size cannot be 0");

    auto* const ctx = this_context::try_get();
    if (!ctx) {
      // creating a new subgang from a non-initialized gang - defer creation
      STAPL_RUNTIME_PROFILE("gang(deferred)", (primitive_traits::blocking |
                                               primitive_traits::environment));
      STAPL_RUNTIME_ASSERT_MSG((size==1),
                               "Size cannot exceed that of parent gang");
      STAPL_RUNTIME_ASSERT_MSG((mf(0)==0), "Bad mapping function");
      STAPL_RUNTIME_ASSERT_MSG((rf(0)==0), "Bad resolution function");
      this_context::push_placeholder(m_context);
    }
    else {
      // creating a new subgang
      STAPL_RUNTIME_PROFILE("gang()", (primitive_traits::blocking |
                                       primitive_traits::environment));
      ctx->flush_requests();
      auto const& l   = ctx->get_location_md();
      auto const& g   = l.get_gang_md();
      const auto nlid = mf(l.get_id());
      STAPL_RUNTIME_ASSERT_MSG((size<=g.size()),
                               "Size cannot exceed that of parent gang");
      STAPL_RUNTIME_ASSERT_MSG((nlid<size), "Bad mapping function");
      STAPL_RUNTIME_ASSERT_MSG((rf(nlid)==l.get_id()),
                               "Bad resolution function");
      auto& nl = create(id,
                        g.get_id(),
                        rf(0),
                        gang_description{g.get_description(), rf, size},
                        nlid,
                        comparable_proxy{std::make_pair(mf, rf)});
      scheduling_point(*ctx);
      m_context = boost::in_place<context>(std::ref(nl));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Switches to the gang of @p t.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  explicit gang(T const& t)
  : m_status(SWITCHED)
  {
    using namespace runtime;

    auto* const ctx = this_context::try_get();
    auto& l         = const_cast<T&>(t).get_location_md();
    if (ctx && ctx->get_location_md()==l && ctx->is_base()) {
      // same gang active, no need to switch
      m_status = INACTIVE;
      return;
    }

    if (ctx) {
      ctx->flush_requests();
      scheduling_point(*ctx);
    }

    STAPL_RUNTIME_PROFILE("gang(switch)", (primitive_traits::blocking |
                                           primitive_traits::environment));
    this_context::switch_to(l, m_context);
  }

  ~gang(void)
  {
    using namespace runtime;

    switch (m_status) {
      case INACTIVE:
        // nothing to do
        break;
      case ACTIVE:
        if (!m_context) {
          // context was not created, remove placeholder
          this_context::pop_placeholder();
        }
        break;
      case SWITCHED:
        if (!m_context) {
          // base context may be inherited, but definitely not created
          this_context::unswitch();
        }
        break;
      default:
        STAPL_RUNTIME_ERROR("Incorrect gang state.");
        break;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Leaves the gang.
  ///
  /// @warning After this call, the section returns to the creator gang,
  ///          therefore one should note that all communication happens in the
  ///          creator gang with everything that this entails (e.g SPMD
  ///          operations have to be called by all locations in the creator
  ///          gang).
  //////////////////////////////////////////////////////////////////////
  void leave(void)
  {
    using namespace runtime;

    switch (m_status) {
      case INACTIVE:
        // nothing to do
        break;
      case ACTIVE:
        if (!m_context) {
          // context was not created, remove placeholder
          this_context::pop_placeholder();
        }
        else {
          // destroy created context
          m_context = boost::none;
        }
        m_status = INACTIVE;
        break;
      case SWITCHED: {
        if (!m_context) {
          // base context may be inherited, but definitely not created
          this_context::unswitch();
        }
        else {
          // destroy created context
          m_context = boost::none;
        }
        m_status = INACTIVE;
      } break;
      default:
        STAPL_RUNTIME_ERROR("Incorrect gang state.");
        break;
    }
  }

  gang(gang const&) = delete;
  gang& operator=(gang const&) = delete;

  void* operator new(std::size_t) = delete;
  void operator delete(void*) = delete;
  void* operator new[](std::size_t) = delete;
  void operator delete[](void*) = delete;

  void* operator new(std::size_t size, void* ptr) noexcept
  { return ::operator new(size, ptr); }

  void operator delete(void* ptr, void* ptr2) noexcept
  { return ::operator delete(ptr, ptr2); }
};


namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Switches to and from the given gang.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class gang_switcher
{
private:
  boost::optional<context> m_context;
  bool                     m_switched;

public:
  gang_switcher(location_md& l)
  : m_switched(false)
  {
    auto* const ctx = this_context::try_get();
    if (!ctx || ctx->get_location_md()!=l) {
      // different gang, must switch
      if (ctx)
        ctx->flush_requests();
      m_switched = true;
      this_context::switch_to(l, m_context);
    }
  }

  ~gang_switcher(void)
  {
    if (m_switched && !m_context)
      this_context::unswitch();
  }
};

} // namespace runtime

} // namespace stapl

#endif
