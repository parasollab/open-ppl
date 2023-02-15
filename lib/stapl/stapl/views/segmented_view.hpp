/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_SEGMENTED_VIEW_HPP
#define STAPL_VIEWS_SEGMENTED_VIEW_HPP

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/views/array_ro_view.hpp>
#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/views/type_traits/upcast.hpp>
#include <stapl/views/operations/paragraph_requirements.hpp>
#include <stapl/views/segmented_view_base.hpp>
#include <stapl/views/metadata/extraction/segmented_view.hpp>
#include <stapl/utility/use_default.hpp>

#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/views/multiarray_view.hpp>

#include <stapl/views/common_view.hpp>
#include <stapl/views/store_in_frame.hpp>
#include <stapl/views/type_traits/upcast.hpp>

#include <boost/utility/in_place_factory.hpp>

#include <iostream>
#include <type_traits>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Base class of @ref segmented_view, which optionally
/// stores the underlying view/container, allowing it to implement the
/// same ownership capabilities of other views.
///
/// @todo Implement small object optimization strategy, dropping to a
/// shared pointer for large objects.
//////////////////////////////////////////////////////////////////////
template <typename Container>
class container_storage
{
private:
  boost::optional<Container> m_ct;

public:
  container_storage(void) = default;

  template<typename ...Args>
  container_storage(Args&&... args)
    : m_ct(boost::in_place(std::forward<Args>(args)...))
  { }

  void define_type(typer& t)
  { t.member(m_ct); }

  Container& managed_container(void)
  {
    if (!m_ct)
      abort("Attempted to access empty container storage.");
    stapl_assert(m_ct, "Attempted to access empty container storage.");
    return *m_ct;
  }

  Container const& managed_container(void) const
  {
    if (!m_ct)
      abort("Attempted to access empty container storage.");
    stapl_assert(m_ct, "Attempted to access empty container storage.");
    return *m_ct;
  }

}; // struct container_storage


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to create a subview for a given index in segmented_view.
///
/// @tparam SV The subview type that will be created
//////////////////////////////////////////////////////////////////////
template<typename SV>
struct default_subview_creator
{
  using type = SV;

  default_subview_creator()
  { }

  // Metafunction to recast this subview creator with a new subview type
  template<typename NewSV>
  struct with_subview
  {
    using type = default_subview_creator<NewSV>;
  };

  template<typename Container, typename ViewDom, typename ViewMF,
           typename Index, typename Partitioner, typename MFG>
  type operator()(Container* cnt, ViewDom const& view_dom,
                  ViewMF const& view_mf, Index const& index,
                  Partitioner const& part, MFG const& mfg) const
  {
    using mf_creator = compose_func<ViewMF, typename MFG::mapfunc_type>;

    auto const& mf    = mf_creator::apply(view_mf, mfg[index]);
    auto const domain = part[index];

    return type{ *cnt, domain, mf };
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Define a container of views, specified by a partition function
///        and a mapping function generator.
///
/// The partition function defines the domain of each view in the
/// container and the mapping function generator, the mapping function
/// to use by each view.
///
/// @tparam View which every view in the container, reference as
///         underneath container
/// @tparam P partition functor type
/// @tparam MFG Mapping function generator type
/// @tparam SVC Functor that creates subviews
/// @todo Determine proper type for const_reference.  This requires
/// determining how to make a view's access methods return const references.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P, typename MFG, typename SVC>
class view_container
  : public common_view,
    public p_object,
    public view_impl::container_storage<View>,
    public view_container_base
{
private:
  using traits_t             = container_traits<view_container>;
  using map_fun_gen_t        = typename traits_t::map_fun_gen_t;
  using subview_creator_type = typename traits_t::subview_creator_type;
  using storage_base_t       = view_impl::container_storage<View>;

  size_t m_version;

public:
  using index_type               = typename P::index_type;
  using view_domain_type         = typename View::domain_type;
  using view_map_func_type       = typename View::map_func_type;
  using view_view_container_type = typename View::view_container_type;
  using container_type           = typename View::view_container_type;

  using gid_type                 = size_t;
  using view_container_type      = View;
  using value_type               = typename traits_t::value_type;

  using loc_dist_metadata        =
    metadata::segmented_view_extractor<view_container>;

  using reference                = value_type;
  using const_reference          = const value_type;
  using partition_type           = P;
  using mapfunc_generator_type   = map_fun_gen_t;
  using domain_type              = typename P::domain_type;
  using traversal_type           = void;

  /// the type to repesent the size in each dimension
  using dimensions_type = typename P::index_type;

  /// integral constant representing the number of dimensions
  using dimension_type  = typename dimension_traits<dimensions_type>::type;

  // Members
  view_view_container_type*       m_container;
  view_map_func_type              m_view_map_func;
  view_domain_type                m_view_domain;
  View const*                     m_view;
  bool                            m_owns_view;
  partition_type                  m_partitioner;
  map_fun_gen_t                   m_map_func_gen;
  bool                            m_is_store_in_frame;

  view_container(view_container const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor that replaces the container for a new container.
  /// @note Constructor used during the fast view transformation.
  //////////////////////////////////////////////////////////////////////
  template<typename Other>
  view_container(view_container_type const* view, Other const& other)
    : p_object(),
      m_version(0),
      m_container(view->get_container()),
      m_view_map_func(view->mapfunc()),
      m_view_domain(view->domain()),
      m_view(view),
      m_owns_view(true),
      m_partitioner(other.partition()),
      m_map_func_gen(other.mapfunc_generator()),
      m_is_store_in_frame(false)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Default constructor, where the underneath container, the
  ///        partition functor and mapping function generator are
  ///        specified.
  ///
  /// @param view is the view from which the subviews are created.
  /// @param part domain partition.
  /// @param mfg mapping function generator.
  //////////////////////////////////////////////////////////////////////
  view_container(view_container_type const& view,
                 partition_type const& part,
                 mapfunc_generator_type const& mfg)
    : p_object(),
      m_version(0),
      m_container(view.get_container()),
      m_view_map_func(view.mapfunc()),
      m_view_domain(view.domain()),
      m_view(&view),
      m_owns_view(false),
      m_partitioner(part),
      m_map_func_gen(mfg),
      m_is_store_in_frame(false)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor where the underlying @c view is constructed in
  /// place and whose lifetime is managed by the @ref view_container.
  ///
  /// @param view is the view from which the subviews are created.
  /// @param part domain partition.
  /// @param mfg mapping function generator.
  //////////////////////////////////////////////////////////////////////
  view_container(store_in_frame,
                 view_container_type const& view,
                 partition_type const& part,
                 mapfunc_generator_type const& mfg)
    : storage_base_t(view),
      m_version(0),
      m_container(view.get_container()),
      m_view_map_func(view.mapfunc()),
      m_view_domain(view.domain()),
      m_view(&view),
      m_owns_view(false),
      m_partitioner(part),
      m_map_func_gen(mfg),
      m_is_store_in_frame(true)
  { }

  ~view_container()
  {
    if (m_owns_view)
      delete m_view;
  }

  view_container_type const* get_view(void) const
  {
    if (!m_is_store_in_frame)
      return m_view;

    return &this->managed_container();
  }

  view_container_type* get_view(void)
  {
    if (!m_is_store_in_frame)
      return const_cast<view_container_type*>(m_view);

    return &this->managed_container();
  }

  view_container_type const& view(void) const
  {
    stapl_assert(m_view != nullptr,"view_container has null view.");

    if (!m_is_store_in_frame)
      return *m_view;

    return this->managed_container();
  }

  view_view_container_type* get_container(void)
  {
    if (!m_is_store_in_frame)
      return m_container;

    return &this->managed_container().container();
  }

  view_view_container_type const* get_container(void) const
  {
    if (!m_is_store_in_frame)
      return m_container;

    return &this->managed_container().container();
  }

  view_view_container_type& container(void)
  {
    if (!m_is_store_in_frame)
      return *m_container;

    return this->managed_container().container();
  }

  view_view_container_type const& container(void) const
  {
    if (!m_is_store_in_frame)
      return *m_container;

    return this->managed_container().container();
  }

  template<typename Slices, typename Fixed>
  struct slice_type
  {
    using type =
      typename view_container_type::template slice_type<Slices, Fixed>::type;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a deep slice by slicing off dimensions specified
  ///        in Slices.
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed  The type of element that will be used to specify the
  ///                fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template <typename Slices, typename Fixed>
  typename slice_type<Slices, Fixed>::type
  slice(Fixed const& fixed)
  {
    return this->get_view()-> template slice<Slices>(fixed);
  }

  partition_type const& partition(void) const
  {
    return m_partitioner;
  }

  mapfunc_generator_type const& mapfunc_generator(void) const
  {
    return m_map_func_gen;
  }

  view_domain_type view_domain(void) const
  {
    return m_view_domain;
  }

  value_type operator[](index_type index) const
  {
    return this->get_element(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief  increment the version number
  //////////////////////////////////////////////////////////////////////
  void incr_version(void)
  {
    ++m_version;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current version number
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return m_version;
  }

 //////////////////////////////////////////////////////////////////////
  /// @brief Return the view associated with the position index (linear
  ///        position) The returned view mapping function is the
  ///        function composition of top view mapping function and the
  ///        mapping function generated for the mapping function
  ///        generator.
  ///
  /// @param index to an element
  /// @return the generated subview at position index
  //////////////////////////////////////////////////////////////////////
  value_type get_element(index_type index) const
  {
    const subview_creator_type svc;
    return svc(
      m_container, m_view_domain, m_view_map_func,
      index, m_partitioner, m_map_func_gen
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the locality information of the segment containing the
  /// index provided.
  /// @param idx GID of a view element whose locality is being requested
  /// @return The locality information of the segment containing the GID.
  ///
  /// @todo Extend the computation to return the locality of all elements
  /// in the segment when locality information allows for multiple locations
  /// @todo Eliminate the gang construction and replace the call to get_element
  /// with a call to retrieve the metadata from the location that holds it. This
  /// will eliminate the construction of a p_object and blocking communication
  /// in cases where the segement is not local.
  //////////////////////////////////////////////////////////////////////
  locality_info locality(index_type const& idx) const
  {
    gang g;
    auto subvw = get_element(idx);
    auto first = subvw.domain().first();

    return subvw.locality(first);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided function the the value referenced
  ///        for the given index returning the resulting value.
  ///
  /// @param index of element to apply the function
  /// @param f function to apply
  /// @return result of evaluating the function f on the value
  ///         referenced for the index
  //////////////////////////////////////////////////////////////////////
  template<class Functor>
  typename Functor::result_type
  apply_get(index_type index, Functor const& f) const
  {
    return f(get_element(index));
  }

  template<class Functor>
  typename Functor::result_type
  apply_get(index_type index, Functor const& f)
  {
    return f(get_element(index));
  }

  size_t size(void) const
  {
    return this->m_partitioner.size();
  }

  domain_type domain(void) const
  {
    return this->m_partitioner.domain();
  }

  dimensions_type dimensions(void) const
  {
    return this->m_partitioner.dimensions();
  }

  bool is_in_frame(void) const
  {
    return m_is_store_in_frame;
  }

  void define_type(typer& t)
  {
    t.base<storage_base_t>(*this);
    t.member(m_container);
    t.member(m_view_map_func);
    t.member(m_view_domain);
    t.member(m_view);
    t.member(m_owns_view);
    t.member(m_partitioner);
    t.member(m_map_func_gen);
    t.member(m_is_store_in_frame);
  }
}; // class view_container


//////////////////////////////////////////////////////////////////////
/// @brief Helper static polymorphic function to compute view container
///   size based on either size() or dimensions() call, based on the
///   the dimensionality of the partitioned data it represents.
//////////////////////////////////////////////////////////////////////
template<typename ViewContainer, int = ViewContainer::dimension_type::value>
struct view_container_size
{
  static
  typename ViewContainer::dimensions_type
  apply(ViewContainer const& vc)
  { return vc.dimensions(); }
};


template<typename ViewContainer>
struct view_container_size<ViewContainer, 1>
{
  static
  size_t apply(ViewContainer const& vc)
  { return vc.size(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to select underlying view type used to
/// implement @ref segmented_view.  Partial specialization in place of
/// std::conditional avoids both view templates being instantiated.
//////////////////////////////////////////////////////////////////////
template<typename Container, bool HasSingleDimension>
struct select_sv_base_view_type
{ using type = multiarray_view<Container>; };


template<typename Container>
struct select_sv_base_view_type<Container, true>
{ using type = array_ro_view<Container>; };

} // namespace view_impl


template<typename View, typename P, typename MFG, typename SVC>
struct container_traits<view_impl::view_container<View, P, MFG, SVC>>
{
  using gid_type = size_t;

  using map_fun_gen_t =
    typename select_parameter<
      MFG,
      map_fun_gen<f_ident<typename View::domain_type::index_type>>
    >::type;

  using composed_mf_t =
    typename compose_func<
      typename View::map_func_type,
      typename map_fun_gen_t::mapfunc_type
    >::type;

  using default_subview_type =
    typename upcast_view<View, typename P::value_type, composed_mf_t>::type;

  using subview_creator_type =
    typename select_parameter<
      SVC, view_impl::default_subview_creator<default_subview_type>
    >::type;

  using subview_type    = typename subview_creator_type::type;
  using value_type      = subview_type;
  using domain_type     = typename P::domain_type;
  using reference       = value_type;
  using const_reference = const value_type;
};

template <typename C, typename P, typename MFG, typename SVC>
class segmented_view;

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of upcast_view (used to change mapping function
/// and domain type of a view) for segmented_view.  This has non standard
/// template parameters (i.e., not C, D, MF, Derived) and furthermore is
/// assumed to inherit domain from partition and have an identity mapping
/// function.  Assert that these have not been changed and reflect the
/// original type passed to it.
//////////////////////////////////////////////////////////////////////
template <typename C, typename P, typename MFG, typename SVC, typename NewDom,
          typename NewMF>
struct upcast_view<segmented_view<C, P, MFG, SVC>, NewDom,
                   NewMF>
{
  using type = segmented_view<C, P, MFG, SVC>;

  static_assert(std::is_same<NewDom, typename type::domain_type>::value,
                "Invalid change of domain type in upcast_view");

  static_assert(std::is_same<NewMF, typename type::map_func_type>::value,
                "Invalid change of mapping function type in upcast_view");
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for view_traits to expose the types provided
///        for segmented_view.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename Partition, typename MFG, typename SVC>
struct view_traits<
  segmented_view<Container, Partition, MFG, SVC>>
  : container_traits<view_impl::view_container<Container, Partition, MFG, SVC>>
{
  using index_type     = typename Partition::index_type;

  using container      =
     view_impl::view_container<Container, Partition, MFG, SVC>;

  using map_function   = f_ident<typename Partition::index_type>;

  using base_view_type =
    typename view_impl::select_sv_base_view_type<
      container, dimension_traits<index_type>::type::value == 1>::type;

  using domain_type    = typename base_view_type::domain_type;

  using underlying_view_type = Container;
};


//////////////////////////////////////////////////////////////////////
/// @brief Define a view over a virtual container of views
///        (view_container).
///
/// @tparam Container type of view to be segmented
/// @tparam Partition Partition functor type
/// @tparam MFG Mapping function generator type
/// @tparam SVC Type of the functor used to create subviews
/// @ingroup segmented_view
///
/// @todo Rename the Container template parameter to View.
//////////////////////////////////////////////////////////////////////
template<typename Container,
         typename Partition,
         typename MFG = use_default,
         typename SVC = use_default>
class segmented_view
  : public segmented_view_base,
    public view_traits<
      segmented_view<Container, Partition, MFG, SVC>>::base_view_type
{
private:
  using traits_t            = view_traits<segmented_view>;
  using part_container_t    = typename traits_t::container;
  using base_type           = typename traits_t::base_view_type;
  using map_fun_gen_t       = typename traits_t::map_fun_gen_t;

public:
  using view_type           = base_type;
  using value_type          = typename traits_t::value_type;
  using domain_type         = typename traits_t::domain_type;
  using index_type          = typename traits_t::index_type;

  // Types required for every view
  using view_container_type = part_container_t;
  using map_function        = typename traits_t::map_function;
  using map_func_type       = map_function;
  using partition_type      = Partition;
  using dimension_type = typename dimension_traits<view_container_type>::type;

  segmented_view(void) = delete;

private:
  using multidim = std::integral_constant<bool, dimension_type::value!=1>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the domain of a multi-dimensional segmented view
  ///        from the partitioner dimensions.
  //////////////////////////////////////////////////////////////////////
  template<typename Part>
  constexpr domain_type
  get_domain(Part const& part, std::true_type /* multidim */) const
  {
    static_assert(dimension_traits<typename Part::index_type>::type::value ==
      dimension_type::value,
      "Incompatible partition and view_container dimensions");

    return { part.dimensions() };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the domain of a 1-dimensional segmented view
  ///        from the partitioner size.
  //////////////////////////////////////////////////////////////////////
  template<typename Part>
  constexpr domain_type
  get_domain(Part const& part, std::false_type /* multidim */) const
  {
    static_assert(dimension_traits<typename Part::index_type>::type::value==1,
      "Incompatible partition and view_container dimensions");

    return { part.size() };
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to
  ///        the view.
  ///
  /// @param c pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mf mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  segmented_view(view_container_type* c,
                 domain_type const& dom,
                 map_func_type const& mf = map_func_type())
    : base_type(c, dom, mf)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  segmented_view(view_container_type const& c,
                 domain_type const& dom,
                 map_func_type const& mf = map_func_type())
    : base_type(c, dom, mf)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  segmented_view(view_container_type const& c,
                 domain_type const& dom,
                 map_func_type const& mf,
                 segmented_view const&)
    : base_type(c, dom, mf)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container.
  ///
  /// @param c reference to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  segmented_view(view_container_type const& c)
    : base_type(c,
                domain_type(view_impl::view_container_size<
                              view_container_type>::apply(c)),
                map_func_type())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a segmented_view over the container c using
  ///        the partition part.
  ///
  /// @param c container to partition
  /// @param part partitioner
  /// @param mfg mapping function generator
  //////////////////////////////////////////////////////////////////////
  template<typename ContainerParam, typename Part>
  segmented_view(ContainerParam const& c,
                 Part const& part,
                 map_fun_gen_t const& mfg = map_fun_gen_t())
    : base_type(new view_container_type(c, part, mfg),
                get_domain(part, multidim()),
                map_func_type())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor where the underlying container is constructed in
  /// place and whose lifetime is managed by the @ref view_container.
  ///
  /// The partitioner is constructed using the container's domain.
  ///
  /// @param ct_args  arguments forwarded to the container's constructor
  //////////////////////////////////////////////////////////////////////
  template <typename Cont>
  segmented_view(view_impl::store_in_frame tag, Cont&& cont)
    : base_type(
        new view_container_type(
          tag,
          std::forward<Cont>(cont),
          partition_type(cont.domain()),
          map_fun_gen_t()),
        get_domain(
          partition_type(cont.domain()), multidim()),
        map_func_type())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor where the underlying container is constructed in
  /// place and whose lifetime is managed by the @ref view_container.
  ///
  /// @param part     partitioner
  /// @param ct_args  arguments forwarded to the container's constructor
  //////////////////////////////////////////////////////////////////////
  template <typename Part, typename Cont>
  segmented_view(Part&& part,
                 view_impl::store_in_frame tag,
                 Cont&& cont)
    : base_type(
        new view_container_type(
          tag,
          std::forward<Cont>(cont),
          std::forward<Part>(part),
          map_fun_gen_t()),
        get_domain(std::forward<Part>(part), multidim()),
        map_func_type())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a segmented_view using a view_container that takes
  ///   ownership of the provided view and copies remaining members from
  ///   the view_container of another segmented view.
  ///
  /// This constructor is invoked during localization of the segmented view
  /// where the provided view is the cast view reconstructed on the heap with
  /// the corresponding base container at its base and the other segmented_view
  /// is the un-cast segmented_view with the original container at its base.
  ///
  /// @tparam Other segmented_view from which to construct this instance
  //////////////////////////////////////////////////////////////////////
  template<typename Other>
  segmented_view(Other const& other, Container* view)
    : base_type(
        new view_container_type(view, other.container()),
        other.domain(), other.mapfunc())
  {
    static_assert(is_segmented_view<Other>::value,
      "Invalid view type passed to segmented_view(Other const&, Container*) "
      "constructor: view derived from segmented_view expected.");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief segmented_view needs to redefine the nested_locality as using
  ///        the one in it's base type(core_view) creates a
  ///        p_object(view_container) when creating a reference to an element
  ///        of segmented_view which could cause unbalanced epochs and hanging.
  ///
  /// @index the index of the element we asked for it's locality
  //////////////////////////////////////////////////////////////////////
  rmi_handle::reference
  nested_locality(index_type const& index)
  {
    static constexpr size_t subview_ndims =
      tuple_size<
        typename tuple_ops::result_of::ensure_tuple<
          typename map_fun_gen_t::mapfunc_type::index_type>::type
      >::value;

    auto&& first_sub_idx =
      tuple_ops::extract_1D(homogeneous_tuple<subview_ndims, size_t>(0));

    return this->container().container().locality(
             map_fun_gen_t()[index](first_sub_idx)).handle();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
}; //class segmented_view

///////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a segmented view.
/// @tparam Container The container which is segmented.
/// @tparam Paritition The partition used to segment the container.
/// @tparam OptionalParams Optional mapping function generator and
///   subview creator parameters.
//////////////////////////////////////////////////////////////////////
template<typename Container, typename Partition, typename... OptionalParams>
segmented_view<Container, Partition, OptionalParams...>
make_segmented_view(Container const& container,
                    Partition const& partition,
                    OptionalParams const&... params)
{
  return segmented_view<Container, Partition, OptionalParams...>(
    container, partition, params...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for changing the container used for a
///        segmented view
//////////////////////////////////////////////////////////////////////
template <typename C, typename P, typename MFG, typename SVC, typename NewC>
struct cast_container_view<segmented_view<C, P, MFG, SVC>, NewC>
{
  using subview_creator_type =
    typename container_traits<typename view_traits<segmented_view<
      C, P, MFG, SVC>>::container>::subview_creator_type;

  using original_subview_type = typename subview_creator_type::type;

  using type = segmented_view<
    typename cast_container_view<C, NewC>::type,
    P,
    MFG,
    typename subview_creator_type::template with_subview<
      typename cast_container_view<original_subview_type, NewC>::type
    >::type
  >;
};

} // namespace stapl

#endif // STAPL_VIEWS_SEGMENTED_VIEW_HPP
