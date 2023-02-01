/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING
#  ifndef STAPL_PARAGRAPH_NEW_COARSE_MAP_WF_H
#    define STAPL_PARAGRAPH_NEW_COARSE_MAP_WF_H

#include <vector>

#include <boost/intrusive/slist.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

namespace composition {

//////////////////////////////////////////////////////////////////////
/// @brief Used in conjunction with @ref mpi_messager to implement RDMA
/// style transmission of @ref edge_container values.  Used to back edge
/// values of type @ref cm_res_view.
///
/// @todo Rename this class to something descriptive.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename T>
struct my_container
{
private:
  /// @brief The amount of space allocated for the header prepended to the mpi
  /// buffer: paragraph id, task id, and size.
  static const size_t      mpi_hdr_size = 3;

  /// @brief Pointer to the buffer used for storing elements of the container
  /// and the mpi buffer.
  T*                       m_storage_ptr;

  /// @brief The number of elements stored in the container.
  size_t                   m_size;

  /// @brief The number of elements storage is allocated for.
  size_t                   m_capacity;

public:
  typedef T                value_type;
  typedef T&               reference;
  typedef T const&         const_reference;
  typedef T*               iterator;
  typedef T const *        const_iterator;


  //////////////////////////////////////////////////////////////////////
  /// @brief Helper function used by constructors to initialize
  /// @p m_storage_ptr to be large enough to hold the mpi header plus
  /// @p n elements of type @p T.
  ///
  /// @return Pointer to initialized storage.
  //////////////////////////////////////////////////////////////////////
  T* allocate(size_t n)
  {
    /// stapl_assert(sizeof(T) == sizeof(size_t),
    stapl_assert(sizeof(T) <= sizeof(size_t),
      "my_container::allocate size problems");

    return new T[n + mpi_hdr_size];
  }


  my_container(size_t n)
    : m_storage_ptr(allocate(n)), m_size(n), m_capacity(n)
  { }


  my_container(size_t n, T const& init)
    : m_storage_ptr(allocate(n)), m_size(n), m_capacity(n)
  {
    std::fill(this->begin(), this->end(), init);
  }


  my_container(my_container const& other)
    : m_storage_ptr(allocate(other.m_capacity)),
      m_size(other.m_size),
      m_capacity(other.m_capacity)
  {
    stapl_assert(other.m_storage_ptr != NULL,
      "my_container::copy_constructor other null ptr");

    std::copy(other.begin(), other.end(), this->begin());
  }


  ~my_container(void)
  {
    delete[] m_storage_ptr;
  }


  ////////////////////////////////////////////////////////////////////////
  /// @brief Return the size of the buffer that should be reported to
  /// MPI commands (i.e., include the MPI header used by this object).
  //////////////////////////////////////////////////////////////////////
  size_t mpi_size(void) const
  {
    return (m_capacity + mpi_hdr_size) * sizeof(size_t);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return pointer to raw buffer that @ref mpi_messager will pass
  /// to MPI_Irecv to receive the inbound message.
  //////////////////////////////////////////////////////////////////////
  void* mpi_buffer(void)
  {
    return reinterpret_cast<void*>(m_storage_ptr);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Called by @ref mpi_messager to initialize the mpi header of
  /// prior to transmission via MPI_Isend.
  ///
  /// @param tg_id The identifier of the associated PARAGRAPH.
  /// @param tid The identifier of the task that produced this value.
  /// @param size The number of elements in this value.
  //////////////////////////////////////////////////////////////////////
  void set_mpi_header(size_t tg_id, size_t tid, size_t size)
  {
    size_t* hdr_ptr = reinterpret_cast<size_t*>(m_storage_ptr);

    hdr_ptr[0] = tg_id;
    hdr_ptr[1] = tid;
    hdr_ptr[2] = size;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the source location and PARAGRAPH identifier
  /// fields stores in the class to the caller (@ref mpi_messager).
  //////////////////////////////////////////////////////////////////////
  std::pair<size_t, size_t>
  get_mpi_header(void) // const
  {
    size_t* hdr_ptr = reinterpret_cast<size_t*>(m_storage_ptr);
    return std::make_pair(hdr_ptr[0], hdr_ptr[1]);
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Called in runtime in @ref mpi_messager upon arrival of
  /// MPI message. Sets logical size based on the size set in the message
  /// header.  Called prior to notify @ref edge_container of value arrival.
  //////////////////////////////////////////////////////////////////////
  void sync_size(void)
  {
    size_t* hdr_ptr = reinterpret_cast<size_t*>(m_storage_ptr);

    const std::size_t actual_size = hdr_ptr[2];

    stapl_assert(actual_size <= m_size, "error in sync_size");

    m_size = actual_size;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Set size of valid elements in buffer to @p n.
  //////////////////////////////////////////////////////////////////////
  void resize(size_t n)
  {
    stapl_assert(n <= m_capacity, "resize failed");

    m_size = n;
  }


  my_container& operator=(my_container const& other)
  {
    if (&other != this)
    {
      stapl_assert(m_capacity == other.m_capacity,
        "my_container::operator= capacities differ");

      stapl_assert(m_size == other.m_size,
        "my_container::operator= different sizes");

      stapl_assert(m_storage_ptr != NULL,
        "my_container::operator= null ptr");

      stapl_assert(other.m_storage_ptr != NULL,
        "my_container::operator= other null ptr");

      std::copy(other.begin(), other.end(), this->begin());
    }
    return *this;
  }


  size_t size(void) const
  {
    return m_size;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an const_iterator to the beginning of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return m_storage_ptr + mpi_hdr_size;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the beginning of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return m_storage_ptr + mpi_hdr_size;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an const_iterator to the end (one past the last element) of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return this->begin() + m_size;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the end (one past the last element) of
  /// the sequence of elements this container stores.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return this->begin() + m_size;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return reference to element in buffer at index @p idx.
  //////////////////////////////////////////////////////////////////////
  reference operator[](std::size_t idx)
  {
    return this->begin()[idx];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return reference to element in buffer at index @p idx.
  //////////////////////////////////////////////////////////////////////
  const_reference operator[](std::size_t idx) const
  {
    return this->begin()[idx];
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Reset the size to the maximum capacity this buffer can hold.
  //////////////////////////////////////////////////////////////////////
  void reset(void)
  {
    m_size = m_capacity;
  }


  //////////////////////////////////////////////////////////////////////
  /// @todo See if this method is needed anymore
  //////////////////////////////////////////////////////////////////////
  //  size_t capacity_and_reset(void)
  //  {
  //    m_size = m_capacity;
  //
  //    return m_capacity;
  //  }


  void define_type(typer& t)
  {
    t.member(m_storage_ptr, m_size + mpi_hdr_size);
    t.member(m_size);
    t.member(m_capacity);
  }
};


template<typename BaseView>
struct map_subview;

  // typename boost::remove_reference<V ## n>::type::reference
# define STAPL_PARAGRAPH_MAP_RESULT(z, n, nothing) \
  typename V ## n::reference

template<typename Container>
struct ref_cnt_elem_holder;


//////////////////////////////////////////////////////////////////////
/// @brief Function object implementing the disposer concept defined by
/// Boost.Intrusive. Function operator signatures restricted to
///
/// @ref ref_cnt_elem_holder to guard against inadvertent use in other
/// contexts.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
struct delete_disposer
{
   template<typename T>
   void operator()(ref_cnt_elem_holder<T> *delete_this)
   {
     delete delete_this;
   }
};


//////////////////////////////////////////////////////////////////////
/// @brief List of buffers previously used by instances of
/// @ref ref_cnt_elem_holder maintained in place of deletion to allow
/// reuse by future @ref ref_cnt_elem_holder objects.
///
/// @tparam T The type of buffer stored in this list.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename T>
struct holder_recycler
  : public boost::intrusive::slist<T>
{
private:
  typedef boost::intrusive::slist<T> base_t;

public:
  holder_recycler(void)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor implements move semantics with @p other.
  //////////////////////////////////////////////////////////////////////
  holder_recycler(holder_recycler const& other)
  {
    this->swap(
      static_cast<base_t&>(const_cast<holder_recycler&>(other))
    );
  }

private:
  holder_recycler& operator=(holder_recycler const&);

public:
  ~holder_recycler(void)
  {
    this->clear_and_dispose(delete_disposer());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class used to manage underlying storage for views of
/// @ref coarse_map_reduce.
///
/// Maintains reference count and list of storage from destroyed instances
/// to reduce allocation and increase locality.
///
/// @tparam Container The container type held as the the underlying storage.
///
/// @todo Use @ref per_location_storage instead of static members to
///       avoid allocation problems in the presence of sub gangs.
///       Reenable STAPL_USE_MANAGED_ALLOC. Figure out how to order with the
///       static variable (pool frees before recycler).
/// @todo Buffer size usage is commented out.  Verify support for more than one
///       size, and also set a max size on the recycle bin.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct ref_cnt_elem_holder
  : public boost::intrusive::slist_base_hook<
      // NOTE - safe mode right now is traversing recycled list & cleaning up.
      // boost::intrusive::link_mode<
      //   boost::intrusive::normal_link
      // >
    >
{
private:
  typedef holder_recycler<ref_cnt_elem_holder> recycle_list_t;

  typedef recycle_list_t                       recycle_bin_t;

  /// @brief Maintains a single instance of recycle list to be used by
  /// static methods and shared across all object of this type.
  static recycle_bin_t                         recycle_bin;

public:
  /// @brief The container size this class template is serving.
  static size_t                                buffer_size;

private:
  /// @brief The container this instance is managing.
  Container  m_ct;

  /// @brief The reference count to the container this instance is managing.
  size_t     m_ref_cnt;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return a pointer to an available instance of the class for
  /// a container of size @p n, reusing a container from the recycle bin if
  /// possible.
  //////////////////////////////////////////////////////////////////////
  static ref_cnt_elem_holder*
  allocate(size_t n)
  {
    /// stapl_assert((n == buffer_size) || ((n+1) == buffer_size),
    ///   "allocate(n) size error");

    if (recycle_bin.empty())
    {
      ref_cnt_elem_holder* ptr = new ref_cnt_elem_holder(n);

      /// ref_cnt_elem_holder* ptr = new ref_cnt_elem_holder(buffer_size);

      ptr->container().resize(n);

      return ptr;
    }

    ref_cnt_elem_holder& holder_ref = recycle_bin.front();

    recycle_bin.pop_front();

    holder_ref.container().resize(n);

    return &holder_ref;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a pointer to an available instance of the class for
  /// a container of size @p n, reusing a container from the recycle bin if
  /// possible.  Initialize all elements with value @p init.
  //////////////////////////////////////////////////////////////////////
  static ref_cnt_elem_holder*
  allocate(size_t n, typename Container::value_type const& init)
  {
    /// stapl_assert((n == buffer_size) || ((n+1) == buffer_size),
    ///   "allocate(n, init) size error");

    if (recycle_bin.empty())
    {
      ref_cnt_elem_holder* ptr = new ref_cnt_elem_holder(n, init);

      /// ref_cnt_elem_holder* ptr = new ref_cnt_elem_holder(buffer_size, init);

      ptr->container().resize(n);

      return ptr;
    }

    ref_cnt_elem_holder& holder_ref = recycle_bin.front();

    recycle_bin.pop_front();

    holder_ref.container().resize(n);

    std::fill(
      holder_ref.container().begin(), holder_ref.container().end(), init
    );

    return &holder_ref;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a pointer to an available instance of the class for
  /// a container of size @p n, initializing the held container with the
  /// same values as @p other.  Reuse a container from the recycle bin
  /// if possible.
  ///
  /// @todo Investigate method is not expected to be called.  If not
  /// needed, refactor code to avoid need of method signature.
  //////////////////////////////////////////////////////////////////////
  static ref_cnt_elem_holder*
  allocate(Container const& other)
  {
    stapl_assert(0, "shouldn't be called");

    const size_t n = other.size();
    ///
    /// stapl_assert((n == buffer_size) || ((n+1) == buffer_size),
    ///   "allocate(Container) size error");

    if (recycle_bin.empty())
    {
      ref_cnt_elem_holder* ptr = new ref_cnt_elem_holder(other);

      ptr->container().resize(n);

      return ptr;
    }

    ref_cnt_elem_holder& holder_ref = recycle_bin.front();

    recycle_bin.pop_front();

    holder_ref.container().resize(n);

    holder_ref = other;

    return &holder_ref;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Reclaim buffer in @p holder that is no longer in use and place it
  /// in the recycle bin.
  //////////////////////////////////////////////////////////////////////
  static void
  deallocate_holder(ref_cnt_elem_holder* holder)
  {
    holder->container().reset();

    recycle_bin.push_front(*holder);
  }

private:
  ref_cnt_elem_holder(size_t n)
    : m_ct(n), m_ref_cnt(1)
  { }

  ref_cnt_elem_holder(size_t n, typename Container::value_type const& init)
    : m_ct(n, init), m_ref_cnt(1)
  { }

  ref_cnt_elem_holder(Container const& ct)
    : m_ct(ct), m_ref_cnt(1)
  { }

public:
  ref_cnt_elem_holder(ref_cnt_elem_holder const&) = delete;

  void increment(void)
  {
    ++m_ref_cnt;
  }

  void decrement(void)
  {
    if (m_ref_cnt == 1)
    {
      deallocate_holder(this);
      return;
    }

    --m_ref_cnt;
  }

  Container& container(void)
  {
    return m_ct;
  }

  void define_type(typer& t)
  {
    t.member(m_ct);
    t.member(m_ref_cnt);
  }
};


template<typename Container>
typename ref_cnt_elem_holder<Container>::recycle_bin_t
ref_cnt_elem_holder<Container>::recycle_bin;


template<typename Container>
size_t ref_cnt_elem_holder<Container>::buffer_size;


//////////////////////////////////////////////////////////////////////
/// @brief Used as the return value for @ref coarse_map_wf.
/// Implements concept of @ref array_view.
///
/// @tparam Container The type of underlying storage employed to store
///                   elements for this view.
///
/// @todo Fix visibility of m_elem_ptr (i.e., make it private).
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Container>
class cm_res_view
{
public:
  typedef void is_view_;

  typedef Container                       container_type;
  typedef typename Container::reference   reference;
  typedef typename Container::const_reference const_reference;
  typedef typename Container::iterator    iterator;
  typedef typename Container::const_iterator const_iterator;
  typedef typename Container::value_type  value_type;
  typedef cm_res_view                     fast_view_type;

// private:
public:
  typedef ref_cnt_elem_holder<Container>  holder_t;

  /// @brief Pointer to underlying elements container held in a reference
  /// counting structure that also attempts to recycle previously allocated
  /// buffers.
  holder_t*                               m_elem_ptr;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Default construction allocates no storage.
  //////////////////////////////////////////////////////////////////////
  cm_res_view(void)
    : m_elem_ptr(NULL)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Allocates uninitialized underlying storage for @p n elements.
  //////////////////////////////////////////////////////////////////////
  explicit cm_res_view(size_t n)
    : m_elem_ptr(holder_t::allocate(n))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Allocates underlying storage for @p n elements initialized
  /// to @p init.
  //////////////////////////////////////////////////////////////////////
  cm_res_view(size_t n, value_type const& init)
    : m_elem_ptr(holder_t::allocate(n, init))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Increment reference count in element holder during copy
  /// construction.
  //////////////////////////////////////////////////////////////////////
  cm_res_view(cm_res_view const& other)
    : m_elem_ptr(other.m_elem_ptr)
  {
    if (m_elem_ptr != NULL)
      m_elem_ptr->increment();
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor called in @ref deep_copy to allocate new
  /// underlying storage and initialize it with the values of @p elems.
  //////////////////////////////////////////////////////////////////////
  explicit cm_res_view(Container const& elems)
    : m_elem_ptr(holder_t::allocate(elems))
  { }


public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Decrement reference count in element holder during destruction.
  //////////////////////////////////////////////////////////////////////
  ~cm_res_view(void)
  {
    if (m_elem_ptr != NULL)
      m_elem_ptr->decrement();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Updates reference counting in lhs and rhs element holders
  /// during assignment operation.
  //////////////////////////////////////////////////////////////////////
  cm_res_view const&
  operator=(cm_res_view const& rhs)
  {
    if (&rhs != this)
    {
      stapl_assert(rhs.m_elem_ptr != 0,
        "cm_res_view::operator= called with null m_elem_ptr");

      if (this->m_elem_ptr != NULL)
        this->m_elem_ptr->decrement();

      this->m_elem_ptr = rhs.m_elem_ptr;

      if (this->m_elem_ptr != NULL)
        this->m_elem_ptr->increment();
    }

    return *this;
  }

public:
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


  container_type& container(void) const
  {
    stapl_assert(m_elem_ptr != NULL,
      "m_res_view::container() called on null view");

    return m_elem_ptr->container();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in the underlying container.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->container().size();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the value at index @p idx in the
  /// underlying container.
  //////////////////////////////////////////////////////////////////////
  reference operator[](std::size_t idx)
  {
    return this->container()[idx];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the value at index @p idx in the
  /// underlying container.
  //////////////////////////////////////////////////////////////////////
  const_reference operator[](std::size_t idx) const
  {
    return this->container()[idx];
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the first element of the container.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return this->container().begin();
  }



  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the end (one past the last element)
  /// of the container.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return this->container().end();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Interface used by the @ref edge_container to force the
  /// container underlying a @p cm_res_view to be copied out of the runtime
  /// buffer during an RMI invocation.
  //////////////////////////////////////////////////////////////////////
  cm_res_view
  deep_copy(void) const
  {
    stapl_assert(m_elem_ptr != NULL,
      "cm_res_view::deep_copy() called on null view");

    return cm_res_view(m_elem_ptr->container());
  }


  void define_type(typer& t)
  {
    stapl_assert(m_elem_ptr != NULL,
      "cm_res_view::define_type: trying to pack null view");

    t.member(m_elem_ptr);
  }
}; // struct cm_res_view


//////////////////////////////////////////////////////////////////////
/// @brief The default coarsened workfunction uses iterators to apply
/// the elementary workfunction to each element in coarsened view passed to it.
///
/// @tparam MapWF The elementary workfunction type.
///
/// @todo Employ empty base class optimization for workfunction.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename MapWF>
class coarse_map_wf
{
private:
  /// @brief The elementary workfunction.
  MapWF m_wf;

public:
  explicit coarse_map_wf(MapWF const& wf)
    : m_wf(wf)
  { }

  void define_type(typer& t)
  {
    t.member(m_wf);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Reflect the return type of the function operator, a view
  /// whose type is an instance of @ref cm_res_view, backed by an instance
  /// of class template @ref my_container.
  ///
  /// @tparam Signature The signature of the result type request. Used
  ///                   to partially specialize based on workfunction
  ///                   arity (i.e., number of views).
  //////////////////////////////////////////////////////////////////////
  template<typename Signature>
  struct result;

# define STAPL_COARSE_MAP_RESULT_DECL(z, n, none) \
  template<BOOST_PP_ENUM_PARAMS(n, typename V)> \
  struct result<coarse_map_wf(BOOST_PP_ENUM_PARAMS(n, V))> \
  { \
  private: \
    typedef my_container< \
      typename boost::result_of< \
        MapWF(BOOST_PP_ENUM(n, STAPL_PARAGRAPH_MAP_RESULT, ~)) \
      >::type \
    > container_t; \
    \
  public: \
    typedef cm_res_view<container_t> type; \
  };
# define BOOST_PP_LOCAL_MACRO(n) \
    STAPL_COARSE_MAP_RESULT_DECL(~, n, BOOST_PP_EMPTY())
# define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_MAX_VIEWS)
# include BOOST_PP_LOCAL_ITERATE()
# undef STAPL_COARSE_MAP_RESULT_DECL

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator receives coarsened views and returns a view
  /// over the results of application of the elementary workfunction on all
  /// elements.
  ///
  /// @param v Variadic list of coarsened views which are inputs to the
  ///          elementary workfunction.
  //////////////////////////////////////////////////////////////////////
# define STAPL_COARSE_MAP_FUNC_OP_DECL(z, n, none) \
  template<BOOST_PP_ENUM_PARAMS(n, typename V)> \
  typename result<coarse_map_wf(BOOST_PP_ENUM_PARAMS(n, V))>::type \
  operator()(BOOST_PP_ENUM_BINARY_PARAMS(n, V, &v)); \
                                                \
  template<BOOST_PP_ENUM_PARAMS(n, typename V)> \
  typename result<coarse_map_wf(BOOST_PP_ENUM_PARAMS(n, V))>::type \
  operator()(BOOST_PP_ENUM_BINARY_PARAMS(n, V, const&v));


# define BOOST_PP_LOCAL_MACRO(n) \
  STAPL_COARSE_MAP_FUNC_OP_DECL(~, n, BOOST_PP_EMPTY())
# define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_MAX_VIEWS)
# include BOOST_PP_LOCAL_ITERATE()
# undef STAPL_COARSE_MAP_FUNC_OP_DECL

}; // class coarse_map_wf


//////////////////////////////////////////////////////////////////////
/// @brief Attempts to transparently reuse the underlying storage of a
/// workfunction's input views (when they are not mutated).  Enables
/// a functional programming model, while avoiding the heap allocation
/// and locality loss drawbacks often associated with this model.
///
/// @tparam Result The return type from a PARAGRAPH, which is then
///                wrapped in a @ref map_subview by the @p map
///                PARAGRAPH this technique is employed in.
///
/// @todo Generalize this for use by other views besides @ref map_subview.
///       Views should probably reflect a reusable type flag.
/// @todo Develop heuristic to decide whether to use lhs or rhs if they
///       are both reusable.
/// @todo The current function signatures catch opportunities for reuse
///       in my current use cases.  Use preprocessing to generate a more
///       exhaustive list of function signatures.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Result>
struct find_result_storage
{
  typedef composition::map_subview<
    detail::result_view<Result>
  > reusable_view_t;


  //////////////////////////////////////////////////////////////////////
  /// @brief Function signature used when there is one view passed to the
  /// workfunction and it is not a candidate for reuse.
  //////////////////////////////////////////////////////////////////////
  template<typename V0>
  Result operator()(V0& v0) const
  {
    return Result(v0.size());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Function signature used when there are two views passed to the
  /// workfunction and neither is a candidate for reuse.
  //////////////////////////////////////////////////////////////////////
  template<typename V0, typename V1>
  Result operator()(V0& v0, V1& v1) const
  {
    return Result(v0.size());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Function signature used when there are two views passed to the
  /// workfunction and the first is a candidate for reuse.
  //////////////////////////////////////////////////////////////////////
  template<typename V1>
  Result operator()(reusable_view_t& v0, V1& v1) const
  {
    if (v0.reusable())
      return v0.container();

    return Result(v0.size());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Function signature used when there are two views passed to the
  /// workfunction and the last is a candidate for reuse.
  //////////////////////////////////////////////////////////////////////
  template<typename V0>
  Result operator()(V0& v0, reusable_view_t& v1) const
  {
    return Result(v0.size());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Function signature used when there are two views passed to the
  /// workfunction and both are candidates for reuse.
  //////////////////////////////////////////////////////////////////////
  Result operator()(reusable_view_t& v0, reusable_view_t& v1) const
  {
    if (v0.reusable())
      return v0.container();

    if (v1.reusable())
      return v1.container();

    return Result(v0.size());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Function signature used when there are three views passed to the
  /// workfunction and the first and last are candidates for reuse.
  //////////////////////////////////////////////////////////////////////
  template<typename V1>
  Result operator()(reusable_view_t& v0, V1& v1, reusable_view_t& v2) const
  {
    if (v0.reusable())
      return v0.container();

    if (v2.reusable())
      return v2.container();

    return Result(v0.size());
  }

}; // struct find_result_storage

#      define BOOST_PP_ITERATION_LIMITS (1, PARAGRAPH_MAX_VIEWS)
#      define BOOST_PP_FILENAME_1 "stapl/skeletons/explicit/new_coarse_map_wf.h"
#      include BOOST_PP_ITERATE()

#  endif // ifndef STAPL_PARAGRAPH_NEW_COARSE_MAP_WF_H

#else // ifndef BOOST_PP_IS_ITERATING

# define i BOOST_PP_ITERATION()

# define PARAGRAPH_VIEW_decl_begin_iter(z, n, data) \
    typename V ## n::iterator iter ## n = BOOST_PP_CAT(v,n).begin();

# define PARAGRAPH_VIEW_decl_begin_const_iter(z, n, data) \
    typename V ## n::const_iterator iter ## n = BOOST_PP_CAT(v,n).begin();

# define PARAGRAPH_VIEW_increment_iter(z, n, data) \
     ++iter ## n;

template<typename MapWF>
template<BOOST_PP_ENUM_PARAMS(i, typename V)>
inline
typename coarse_map_wf<MapWF>::template result<
  coarse_map_wf<MapWF>(BOOST_PP_ENUM_PARAMS(i, V))
>::type
composition::coarse_map_wf<MapWF>::
operator()(BOOST_PP_ENUM_BINARY_PARAMS(i, V, &v))
{
  typedef result<coarse_map_wf<MapWF>(BOOST_PP_ENUM_PARAMS(i, V))> result_mf_t;
  typedef typename result_mf_t::type                               result_type;

  stapl_assert(v0.size() != 0, "coarse_map_wf::operator: v0 has no elements");

  result_type result_vw =
    find_result_storage<result_type>()(BOOST_PP_ENUM_PARAMS(i, v));

  /// result_type result_vw(v0.size());

  BOOST_PP_REPEAT(i, PARAGRAPH_VIEW_decl_begin_iter, ~)

  typename V0::iterator end_iter = v0.end();

  size_t idx = 0;

  for ( ; iter0 != end_iter; )
  {
    result_vw[idx] = m_wf(BOOST_PP_ENUM_PARAMS(i, *iter));

    ++idx;

    BOOST_PP_REPEAT(i, PARAGRAPH_VIEW_increment_iter, ~)
  }

  return result_vw;
}

template<typename MapWF>
template<BOOST_PP_ENUM_PARAMS(i, typename V)>
inline
typename coarse_map_wf<MapWF>::template result<
  coarse_map_wf<MapWF>(BOOST_PP_ENUM_PARAMS(i, V))
>::type
composition::coarse_map_wf<MapWF>::
operator()(BOOST_PP_ENUM_BINARY_PARAMS(i, V, const&v))
{
  typedef result<coarse_map_wf<MapWF>(BOOST_PP_ENUM_PARAMS(i, V))> result_mf_t;
  typedef typename result_mf_t::type                               result_type;

  stapl_assert(v0.size() != 0, "coarse_map_wf::operator: v0 has no elements");

  result_type result_vw =
    find_result_storage<result_type>()(BOOST_PP_ENUM_PARAMS(i, v));

  /// result_type result_vw(v0.size());

  BOOST_PP_REPEAT(i, PARAGRAPH_VIEW_decl_begin_const_iter, ~)

  typename V0::const_iterator end_iter = v0.end();

  size_t idx = 0;

  for ( ; iter0 != end_iter; )
  {
    result_vw[idx] = m_wf(BOOST_PP_ENUM_PARAMS(i, *iter));

    ++idx;

    BOOST_PP_REPEAT(i, PARAGRAPH_VIEW_increment_iter, ~)
  }

  return result_vw;
}

# undef i
# undef PARAGRAPH_VIEW_increment_iter
# undef PARAGRAPH_VIEW_decl_begin_iter

#endif // ifndef BOOST_PP_IS_ITERATING

#ifndef BOOST_PP_IS_ITERATING
#  ifndef STAPL_PARAGRAPH_NEW_COARSE_MAP_WF_FOOTER_H
#    define STAPL_PARAGRAPH_NEW_COARSE_MAP_WF_FOOTER_H

#undef STAPL_PARAGRAPH_MAP_RESULT

} // namespace composition

} // namespace stapl

#  endif //infdef STAPL_PARAGRAPH_NEW_COARSE_MAP_WF_FOOTER_H
#endif // ifndef BOOST_PP_IS_ITERATING
