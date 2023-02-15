/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING
#  ifndef STAPL_SKELETONS_EXPLICIT_NEW_MAP_H
#    define STAPL_SKELETONS_EXPLICIT_NEW_MAP_H
#    ifndef PARAGRAPH_MAX_VIEWS
#      define PARAGRAPH_MAX_VIEWS 5
#    endif

#include <boost/preprocessor/repetition/enum_params_with_a_default.hpp>
#include <boost/preprocessor/arithmetic/inc.hpp>

#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/skeletons/utility/view_index_partition.hpp>
#include <stapl/skeletons/explicit/new_coarse_map_wf.h>
#include <stapl/skeletons/utility/tags.hpp>

#include <boost/utility/result_of.hpp>

namespace stapl {

namespace composition {

//////////////////////////////////////////////////////////////////////
/// @brief Implements required interface of the old views to provide a
/// components class with the method @p get_id.
///
/// @tparam Cid The index type for the view.
///
/// @deprecated  Old views are removed from trunk.  @ref new_map needs to be
///   updated to use the new view interfaces.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Cid>
struct component_holder
{
  /// @brief Index value used to reference an element in the view.
  Cid m_val;

  component_holder(Cid const& val)
    : m_val(val)
  { }

  Cid get_id(void) const
  {
    return m_val;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object which returns the application of @p operator[]
/// on the input parameter, passing a data member as the index argument.
///
/// This functor is used by the @ref map_view class to redirect requests for
/// elements not on this location to the sibling structure on a remote
/// location.
///
/// @param T The type of the parameter to the function operator.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename T>
struct get_element_func
{
  typedef typename T::value_type result_type;

  std::size_t m_idx;

  void define_type(typer& t)
  {
    t.member(m_idx);
  }

  get_element_func(std::size_t idx)
    : m_idx(idx)
  { }

  result_type
  operator()(T const& t) const
  {
    return t[m_idx];
  }
};


template<typename View>
struct repeated_subview;


//////////////////////////////////////////////////////////////////////
/// @brief Defines a one dimensional view where every element is a light
/// wrapper around a copy of the underlying view of type @p View.
///
/// Implemented to prototype the replicated computation concept. Used
/// in the function @ref map_factory to support efficient matrix vector
/// multiplication.
///
/// @todo This implementation needs to be removed and the additional
///       functionality it prototypes wrapped into the class of the
///       same name in the view framework.
/// @todo The deferred_evaluation_view_ enabled behavior and associated
///       methods available and request_notify should be conditionally
///       included based on the parameter @p View to avoid unnecessary
///       runtime overhead.
/// @todo Restrict visibility of available and request_notify to PARAGRAPH only.
///       Maybe employ view_core_access idiom.
/// @todo Externally guard calling of get_num_copies method to avoid needing
///       empty definition.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename View>
struct repeat_view
  : p_object
{
  typedef View base_view_type;

  /// @brief The underlying view wrapped and returned as each element of
  /// @p repeat_view.
  View m_view;

  void define_type(typer&)
  {
    abort("repeat_view::define_type called");
  }

  // Factory Required Typedefs...
  typedef repeated_subview<View>             subview_type;
  typedef subview_type                       reference;
  typedef size_t                             cid_type;

  repeat_view(View const& view)
    : m_view(view)
  { }

  size_t version(void) const
  {
    return find_accessor<View>()(m_view).version();
  }

  bool validate(void) const
  {
    return find_accessor<View>()(m_view).validate();
  }

  /// @brief Informs PARAGRAPH that values the view refers to may not be
  /// available yet so that it can create notifications to defer execution
  /// as needed.
  typedef void deferred_evaluation_view_;


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the underlying view has its data available
  /// for consumption.
  //////////////////////////////////////////////////////////////////////
  bool available(void) const
  {
    return find_accessor<View>()(m_view).available();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Allows users to request the given @p notifier be invoked
  /// when the values of the view are available for consumption.
  ///
  /// Requests redirected to underlying view.
  //////////////////////////////////////////////////////////////////////
  template<typename Notifier>
  void request_notify(Notifier notifier) const
  {
    find_accessor<View>()(m_view).request_notify(notifier);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Required method when computation is replicated in the system.
  /// Replicated views don't drive computation, so this method should never
  /// be called.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_copies(void) const
  {
    abort("repeat_view::get_num_copies called");

    return m_view.get_num_copies();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief The required value for repeated views by @ref partition_id_set.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_local_subviews(size_t = 0) const
  {
    return 1;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief The required value for repeated views by @ref partition_id_set.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_subviews(void) const
  {
    abort("detail::repeat_view::get_num_subviews() called");

    return 0;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Old view interface returning component wrapper of index value.
  //////////////////////////////////////////////////////////////////////
  component_holder<size_t>
  get_local_component(size_t idx) const
  {
    stapl_assert(idx == 0, "repeat_view::get_elemet_component: idx != 0");

    return component_holder<size_t>(0);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a @ref stapl::composition::repeated_subview wrapping
  /// the underlying view.
  ///
  /// @param idx An index into this view.  Unused as all elements are the same.
  //////////////////////////////////////////////////////////////////////
  subview_type get_subview(cid_type idx) const
  {
    return reference(this->m_view);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return locality information as required by the task placement
  /// interface of a PARAGRAPH scheduler.
  ///
  /// @param idx The task identifier locality information is requested for.
  ////////////////////////////////////////////////////////////////////////
  locality_info locality(cid_type idx)
  {
    return locality_info(
      LQ_CERTAIN, get_affinity(),
      this->get_rmi_handle(),
      this->get_location_id()
   );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returned by @ref map_view as index into a coarsened portion of
/// return value it represents.  Typically is all the elements on a given
/// location.  Used to facilitate inter-PARAGRAPH data flow.
///
/// @tparam BaseView The underlying view type.  In current usage, always
///                  an instance of class template @ref result_view.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename BaseView>
struct map_subview
{
private:
  typedef typename BaseView::cid_type    idx_t;
  typedef typename BaseView::value_type  base_sv_t;

  /// @brief The underlying view @ref result_view.
  BaseView   m_view;

  /// @brief An index to an element of @ref result_view.
  idx_t      m_idx;

public:
  map_subview(void)
  { }

  map_subview(BaseView const& view, idx_t idx)
    : m_view(view), m_idx(idx)
  { }

  /// @brief Informs PARAGRAPH that values the view refers to may not be
  /// available yet so that it can create notifications to defer execution
  /// as needed.
  typedef void deferred_evaluation_view_;

  typedef typename base_sv_t::const_reference   reference;
  typedef typename base_sv_t::const_iterator    iterator;
  typedef typename base_sv_t::value_type  value_type;

  typedef map_subview                     fast_view_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the underlying view has its data available
  /// for consumption.
  //////////////////////////////////////////////////////////////////////
  bool available(void) const
  {
    return m_view.available(m_idx);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the underlying container.
  ///
  /// Used by @ref find_result_storage if @p reusable returns true.
  //////////////////////////////////////////////////////////////////////
  base_sv_t const& container(void) const
  {
    stapl_assert(this->available(),
      "map_subview trying to access non available values");

    return m_view.container()[m_idx];
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Interface used by @ref find_result_storage to detect if the
  /// underlying storage can be transparently reclaimed and mutated (if the
  /// task holding the view is the only remaining consumer of the underlying
  /// data container).
  //////////////////////////////////////////////////////////////////////
  bool reusable(void) const
  {
    return m_view.container().reusable();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Required interface for PARAGRAPH localization.  Unconditionally
  /// returns true as view is created on consuming PARAGRAPH location.
  //////////////////////////////////////////////////////////////////////
  bool is_local(void) const
  {
    return true;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Required PARAGRAPH callback for views to perform optional
  /// pre-execution operations.  A noop in this class.
  //////////////////////////////////////////////////////////////////////
  void pre_execute(void) const
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Required PARAGRAPH callback for views to perform optional
  /// post-execution operations.  A noop in this class.
  //////////////////////////////////////////////////////////////////////
  void post_execute(void) const
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the first element of the view.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void) const
  {
    return this->container().begin();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the end (one past the last element)
  /// of the view.
  //////////////////////////////////////////////////////////////////////
  iterator end(void) const
  {
    return this->container().end();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the value in the underlying container
  /// at index @p idx.
  //////////////////////////////////////////////////////////////////////
  value_type&
  operator[](size_t idx) const
  {
    return this->container()[idx];
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief The number of elements this view references.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->container().size();
  }

  void define_type(typer&)
  {
    abort("map_subview::define_type called.");
  }
}; // struct map_subview


//////////////////////////////////////////////////////////////////////
/// @brief A view that is the result of the functional map operation
/// @p map_func.
///
/// Unique features of the base @p map_func are results backed by the
/// PARAGRAPH's @ref result_view and support for transparent computation
/// replication.
///
/// @tparam T The result type of the PARAGRAPH for each location.
///
/// @todo available method and operator[] needs generalization for skeletons
/// of off location consumption, in which cases we need global termination
/// detection.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename T>
struct map_view
  : p_object
{
private:
  typedef detail::result_view<T>              base_view_t;
  typedef typename base_view_t::reference     ref_t;

  /// @brief A view of the result of the PARAGRAPH computation this
  /// map_view refers to.
  base_view_t                                 m_base_view;

  /// @brief The number of local subviews this map_view holds.
  size_t                                      m_n_subviews;

  /// @brief The number of replicated versions that exist of the computation
  /// whose results this view represents.
  size_t                                      m_n_copies;


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the underlying view.
  //////////////////////////////////////////////////////////////////////
  T subview(void) const
  {
    // return m_base_view.get_element(0);

    return m_base_view[0];
  };

public:
  /// @brief Informs PARAGRAPH that values the view refers to may not be
  /// available yet, so that it can create notifications to defer execution
  /// as needed.
  typedef void deferred_evaluation_view_;

  map_view(map_view const& other)
    : m_base_view(other.m_base_view),
      m_n_subviews(other.m_n_subviews),
      m_n_copies(other.m_n_copies)
  { }

  map_view(base_view_t const& view, size_t n_sv, size_t n_copies)
    : m_base_view(view), m_n_subviews(n_sv), m_n_copies(n_copies)
  { }

  map_view& operator=(map_view const& rhs)
  {
    if (&rhs == this)
      return *this;

    m_base_view    = rhs.m_base_view;
    m_n_subviews   = rhs.m_n_subviews;

    return *this;
  }

  size_t version() const
  {
    return 0;
  }

  bool validate() const
  {
    return true;
  }

  void define_type(typer&)
  {
    abort("map_view::define_type called");
  }

  // Factory Required Typedefs...
  typedef map_subview<base_view_t>         subview_type;

  typedef subview_type                     reference;
  typedef typename T::value_type           value_type;
  typedef typename T::iterator             iterator;
  typedef replicated_cid                   cid_type;


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the first element of the local
  /// portion of data represented by the view.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void) const
  {
    return this->subview().begin();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the end (one past the last element)
  /// of the local portion of data represented by the view.
  //////////////////////////////////////////////////////////////////////
  iterator end(void) const
  {
    return this->subview().begin();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the underlying view has its data available
  /// for consumption.
  //////////////////////////////////////////////////////////////////////
  bool available(void) const
  {
    return local_available();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Allows users to request the given @p notifier be invoked
  /// when the values of the view are available for consumption.
  ///
  /// Requests redirected to underlying view.
  //////////////////////////////////////////////////////////////////////
  template<typename Notifier>
  void request_notify(Notifier notifier) const
  {
    request_local_notify(notifier);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the local portion of the underlying view has
  /// its data available for consumption.
  //////////////////////////////////////////////////////////////////////
  bool local_available(void) const
  {
    return m_base_view.available(0);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Allows users to request the given @p notifier be invoked
  /// when the local values of the view are available for consumption.
  ///
  /// Requests redirected to underlying view.
  //////////////////////////////////////////////////////////////////////
  template<typename Notifier>
  void request_local_notify(Notifier notifier) const
  {
    return m_base_view.request_notify(0, notifier);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the element in this view at index @p idx.
  //////////////////////////////////////////////////////////////////////
  value_type
  operator[](std::size_t idx) const
  {
    std::size_t block_size = this->subview().size();
    std::size_t local_idx  = idx % block_size;

    stapl_assert(get_location_id() == idx / block_size,  "errors..");

    return this->subview()[local_idx];
  }

  //
  // Factory Required Methods
  //
  size_t get_num_copies(void) const
  {
    return m_n_copies;
  }


  size_t get_num_subviews(void) const
  {
    return m_n_subviews;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief The required value for repeated views by @ref partition_id_set.
  /// @param replicate_set the instance of replicated computation this view
  ///   is associated with.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_local_subviews(size_t replicate_set = UINT_MAX) const
  {
    // assumes block and cyclic replicated to fill all processors..
    if (replicate_set == UINT_MAX)
    {
      if (this->get_location_id() < m_n_subviews)
        return 1;

      // else
      return 0;
    }

    // else
    return 1;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Old view interface returning component wrapper of index value.
  //////////////////////////////////////////////////////////////////////
  component_holder<cid_type>
  get_local_component(size_t idx) const
  {
    stapl_assert(idx == 0, "map_view: Invalid local_component id");

    const location_type loc = get_location_id();

    stapl_assert(m_n_copies != 1 || loc < m_n_subviews,
      "map_view::get_local_component: called on empty location");

    return component_holder<cid_type>(
      cid_type(loc % m_n_subviews, loc / m_n_subviews)
    );
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a @ref map_subview of a coarsened piece of this view.
  /// @param idx An index into this view.
  //////////////////////////////////////////////////////////////////////
  subview_type get_subview(cid_type idx) const
  {
    stapl_assert(
      (m_n_subviews * idx.replicate_set() + idx.index())
        == get_location_id(),
      "map_view Invalid cid"
    );

    return subview_type(m_base_view, idx);
  }



  //////////////////////////////////////////////////////////////////////
  /// @brief Return locality information as required by the task placement
  /// interface of a PARAGRAPH scheduler.
  ///
  /// @param idx The task identifier locality information is requested for.
  ////////////////////////////////////////////////////////////////////////
  locality_info locality(cid_type idx)
  {
    return locality_info(
      LQ_CERTAIN, get_affinity(),
      this->get_rmi_handle(),
      this->get_location_id()
   );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number elements in all locations this view refers to.
  //////////////////////////////////////////////////////////////////////
  std::size_t
  size(void) const
  {
    return this->subview().size() * this->get_num_subviews();
  }
}; // struct map_view


//////////////////////////////////////////////////////////////////////
/// @brief Factory implementing functional version of @ref map_func which
/// returns a view over the result of the mapping function applied to all
/// elements.  The input views are not mutated.
///
/// @tparam MapWF    Type of the user provided workfunction.
/// @tparam b_coarse Boolean computed internally signifying whether
///                  @p MapWF is an elemental workfunction or one
///                  pre-coarsened by the user.
///
/// @todo Right now, we are assuming a block distribution among the
///       set of locations 0..n_subviews-1. Handles global block
///       distribution and first row distribution of 2D processor
///       layout uniformly. Generalize workfunction and custom key
///       mapper.
/// @todo This class needs to be integrated with (perhaps replacing) the
///       standard @p map_func that allows mutation of input views and
///       has a void return value.
/// @todo Use empty base optimization on stored workfunction member.
/// @todo Fix visibility of @p m_wf, @p coarse_map_wf_t.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template <typename MapWF, bool b_coarse = false>
struct map_factory
  : public task_factory_base
{
// private:
public:
  typedef typename boost::mpl::if_c<
    b_coarse,
    MapWF,
    composition::coarse_map_wf<MapWF>
  >::type                                                   coarse_map_wf_t;
  using coarsener_type = default_coarsener;
  /// @brief The user specified workfunction.
  MapWF m_wf;

public:
  typedef skeletons::tags::composition_map                   tag_type;
  typedef stapl::identity<size_t>                           task_id_mapper_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a task to location mapping function used as a custom
  /// key mapper for the associated PARAGRAPH's @ref directory.
  ///
  /// @param v Variadic list of coarsened view inputs to the factory.
  ///          Unused, included to match expected signature.
  //////////////////////////////////////////////////////////////////////
# define STAPL_MAPPER_OP_DECL(z, n, none) \
  \
  template<BOOST_PP_ENUM_PARAMS(n, typename V)> \
  task_id_mapper_type \
  get_task_id_mapper(BOOST_PP_ENUM_BINARY_PARAMS(n, V, & v)) const \
  { \
    return task_id_mapper_type(); \
  }
# define BOOST_PP_LOCAL_MACRO(n) STAPL_MAPPER_OP_DECL(~, n, BOOST_PP_EMPTY())
# define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_MAX_VIEWS)
# include BOOST_PP_LOCAL_ITERATE()
# undef STAPL_MAPPER_OP_DECL

  ///////////////////////////////////////////////////////////////////////
  /// @brief Reflect return type of @p map_factory. Passed view list in
  /// @p Signature which includes the arguments to function operator.
  //////////////////////////////////////////////////////////////////////
  template<typename Signature>
  struct result;

  map_factory(MapWF const& wf)
    : m_wf(wf)
  { }

  default_coarsener get_coarsener() const { return default_coarsener(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator of factory which receives @ref paragraph_view
  ///  as well as coarsened input views. Populates the PARAGRAPH with tasks.
  ///
  /// @param tgv A @ref paragraph_view over the associated PARAGRAPH.
  /// @param v   Variadic list of coarsened input views to the PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
# define STAPL_MAP_FUNC_OP_DECL(z, n, none) \
  \
  template<typename TGV, BOOST_PP_ENUM_PARAMS(n, typename V)> \
  void operator()(TGV const& tgv, BOOST_PP_ENUM_BINARY_PARAMS(n, V, & v));

# define BOOST_PP_LOCAL_MACRO(n) STAPL_MAP_FUNC_OP_DECL(~, n, BOOST_PP_EMPTY())
# define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_MAX_VIEWS)
# include BOOST_PP_LOCAL_ITERATE()
# undef STAPL_MAP_FUNC_OP_DECL

}; // class map_factory


namespace result_of {

///////////////////////////////////////////////////////////////////////
/// @brief Reflect return type of @p map_func passed workfunction @p MapWF
/// and a variable number of views.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename MapWF,
BOOST_PP_ENUM_PARAMS_WITH_A_DEFAULT(
  BOOST_PP_INC(PARAGRAPH_MAX_VIEWS), typename V, void
)>
struct map_func;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that receives a view and returns it.  Effectively
///  an identify functor.
///
/// Used in @ref map_func to handle edge case of a location without any
/// elements to process (i.e., return an empty view).
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
struct simple_wf
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Nested struct that reflects return type of function
  /// operator invocation.
  //////////////////////////////////////////////////////////////////////
  template<typename Sig>
  struct result;

  template<typename T>
  struct result<simple_wf(T)>
  {
    typedef T type;
  };

  template<typename T>
  T operator()(T& v)
  {
    return v;
  }
};


#    define BOOST_PP_ITERATION_LIMITS (1, PARAGRAPH_MAX_VIEWS)
#    define BOOST_PP_FILENAME_1 "stapl/skeletons/explicit/new_map.h"
#    include BOOST_PP_ITERATE()
#  endif // STAPL_SKELETONS_EXPLICIT_MAKE_SKELETONS_H

#else // BOOST_PP_IS_ITERATING

#define i BOOST_PP_ITERATION()

#define STAPL_SKELETONS_MAP_RESULT(z, n, nothing) \
  typename V ## n::reference::reference

#define STAPL_SKELETONS_MAP_RESULT2(z, n, nothing) \
  typename V ## n::reference

template<typename WF, bool b_coarse>
template<BOOST_PP_ENUM_PARAMS(i, typename V)>
struct
map_factory<WF, b_coarse>::
result<map_factory<WF, b_coarse>(BOOST_PP_ENUM_PARAMS(i, V))>
{
private:
  typedef typename boost::mpl::if_c<
    b_coarse, WF, composition::coarse_map_wf<WF>
  >::type                                                  coarse_map_wf_t;

  typedef typename boost::result_of<
    coarse_map_wf_t(BOOST_PP_ENUM(i, STAPL_SKELETONS_MAP_RESULT2, ~))
  >::type                                                  wf_ret_t;

public:
  typedef map_view<wf_ret_t>                               type;
};

# undef STAPL_SKELETONS_MAP_RESULT
# undef STAPL_SKELETONS_MAP_RESULT2

template <typename WF, bool b_coarse>
template <typename TGV, BOOST_PP_ENUM_PARAMS(i, typename V)>
void map_factory<WF, b_coarse>::
operator()(TGV const& tgv, BOOST_PP_ENUM_BINARY_PARAMS(i, V, & view))
{
  // typedefs for view_index_iterator types
 # define STAPL_SKELETONS_MAP_ID_TYPEDEF(z, n, nothing) \
    typedef view_index_iterator<V ## n> id_iter_type ## n;
   BOOST_PP_REPEAT(i, STAPL_SKELETONS_MAP_ID_TYPEDEF, none)
 # undef STAPL_SKELETONS_MAP_ID_TYPEDEF

   tuple<BOOST_PP_ENUM_PARAMS(i, id_iter_type)> id_it =
     partition_id_set<true>(BOOST_PP_ENUM_PARAMS(i, view));

 # define STAPL_SKELETONS_MAP_ID_INIT(z, n, nothing) \
     id_iter_type ## n iter ## n = get<n>(id_it);
   BOOST_PP_REPEAT(i, STAPL_SKELETONS_MAP_ID_INIT, none)
 # undef STAPL_SKELETONS_MAP_ID_INIT

  stapl_assert(iter0.size() <= 1,
    "new map_factory: unsupported subview configuration detected.");

  std::size_t result_tid = get_location_id();

  using std::make_pair;

  const std::size_t num_succs = 1;

  if (iter0.size() == 0)
  {
#   define STAPL_SKELETONS_MAP_RESULT2(z, n, nothing) \
    typename V ## n::reference

    typedef typename boost::result_of<
      coarse_map_wf_t(BOOST_PP_ENUM(i, STAPL_SKELETONS_MAP_RESULT2, ~))
    >::type                                                      wf_ret_t;

    tgv.add_task(
      result_tid, simple_wf(), num_succs, localize_object(wf_ret_t(0))
    );

#   undef STAPL_SKELETONS_MAP_RESULT2
  }
  else
  {
    // create the set of map tasks
    for (; !iter0.at_end();)
    {
#      define STAPL_SKELETONS_MAP_ID_MAKE_PAIR(z, n, nothing) \
         make_pair(&view ## n, *iter ## n)

       tgv.add_task(result_tid, coarse_map_wf_t(m_wf), num_succs,
                    BOOST_PP_ENUM(i, STAPL_SKELETONS_MAP_ID_MAKE_PAIR, none));

#      undef STAPL_SKELETONS_MAP_ID_MAKE_PAIR

#      define STAPL_SKELETONS_MAP_ID_INCREMENT(z, n, nothing) \
         ++iter ## n;
       BOOST_PP_REPEAT(i, STAPL_SKELETONS_MAP_ID_INCREMENT, none)
#      undef STAPL_SKELETONS_MAP_ID_INCREMENT

       stapl_assert(iter0.at_end(),
         "currently only 1 element per loc (post coarsening)");
    }
  }

  tgv.set_result(result_tid);
} // map_factory::operator()


namespace result_of {

///////////////////////////////////////////////////////////////////////
/// @brief Reflect return type of @p map_func when passed the elemental
/// workfunction.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename MapWF, BOOST_PP_ENUM_PARAMS(i, typename V)>
struct map_func<MapWF, BOOST_PP_ENUM_PARAMS(i, V)>
  : public boost::result_of<
      paragraph<default_scheduler, composition::map_factory<MapWF, false>,
             BOOST_PP_ENUM_PARAMS(i,V)>()
    >
{ };


///////////////////////////////////////////////////////////////////////
/// @brief Reflect return type of @p map_func when passed a user
/// coarsened workfunction.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename MapWF, BOOST_PP_ENUM_PARAMS(i, typename V)>
struct map_func<coarsened_wf<MapWF>, BOOST_PP_ENUM_PARAMS(i, V)>
  : public boost::result_of<
      paragraph<default_scheduler, composition::map_factory<MapWF, true>,
             BOOST_PP_ENUM_PARAMS(i,V)>()
    >
{ };

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Signature for functional version of @p map_func accepting
/// the elemental workfunction.
///
/// @param fmap Workfunction to apply to each set of elements defined
///             container in the input views.
/// @param v    Variadic list of views that should be coarsened and
///             passed to the @ref map_factory invocation for task
///             creation.
///
/// @return A view representing the application of the user defined
///         operation on each element of the input view.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename MapWF, BOOST_PP_ENUM_PARAMS(i, typename V)>
inline
typename result_of::map_func<MapWF, BOOST_PP_ENUM_PARAMS(i, V)>::type
map_func(MapWF const& fmap,
         BOOST_PP_ENUM_BINARY_PARAMS(i, V, const& v))
{
  typedef composition::map_factory<MapWF, false>             factory_t;
  typedef paragraph<
    default_scheduler, factory_t, BOOST_PP_ENUM_PARAMS(i, V)
  >                                                          tg_t;

  return (*new tg_t(factory_t(fmap),
                    BOOST_PP_ENUM_PARAMS(i, v)))
          ((int) 0);
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for functional version of @p map_func accepting
/// a user coarsened workfunction.
///
/// @param fmap User coarsened workfunction accepting coarsened data
///             in the input view.
/// @param v    Variadic list of views that should be coarsened and
///             passed to the @ref map_factory invocation for task
///             creation.
///
/// @return A view representing the application of the user defined
///         operation on each element of the input view.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename MapWF, BOOST_PP_ENUM_PARAMS(i, typename V)>
inline
typename result_of::map_func<
  coarsened_wf<MapWF>,
  BOOST_PP_ENUM_PARAMS(i, V)
>::type
map_func(coarsened_wf<MapWF> const& fmap,
         BOOST_PP_ENUM_BINARY_PARAMS(i, V, const& v))
{
  typedef composition::map_factory<MapWF, true>                factory_t;
  typedef paragraph<
    default_scheduler, factory_t, BOOST_PP_ENUM_PARAMS(i, V)
  >                                                            tg_t;

  return (*new tg_t(factory_t(fmap),
                    BOOST_PP_ENUM_PARAMS(i, v)))
          ((int) 0);
}

# undef i

#endif // ifndef BOOST_PP_IS_ITERATING

#ifndef BOOST_PP_IS_ITERATING

#  ifndef STAPL_SKELETONS_EXPLICIT_NEW_MAP_FOOTER_H
#    define STAPL_SKELETONS_EXPLICIT_NEW_MAP_FOOTER_H

} // namespace composition

} // namespace stapl

#  endif

#endif // ifndef BOOST_PP_IS_ITERATING
