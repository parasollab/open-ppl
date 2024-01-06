/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef MATRIX_VIEW_HPP
#define MATRIX_VIEW_HPP

namespace stapl {

template<typename Container>
struct my_matrix_view;

namespace detail {

struct subview_base
{
  void pre_execute() const
  { }

  void post_execute() const
  { }

  bool is_local() const
  {
    return true;
  }
};


// matrix view passed to coarse_map_reduce_wf of the nested inner_product
template<typename MTLRowSlice>
struct row_slice_subview
  : public subview_base
{
  MTLRowSlice m_row;

  typedef decltype(m_row.begin())     iterator;
  typedef row_slice_subview           fast_view_type;

  // FIXME - do I like this? coarse_map_reduce_wf wants it
  typedef double& reference;

  row_slice_subview(MTLRowSlice const& row)
    : m_row(row)
  { }

  auto
  begin(void) -> decltype(m_row.begin())
  {
    return m_row.begin();
  }

  auto
  end(void) -> decltype(m_row.end())
  {
    return m_row.end();
  }
};


// matrix view passed to the inner matvec tg (e.g., inner_product)
template <typename MTLRowSlice>
struct row_slice_view
{
  MTLRowSlice m_row;

  //FIXME - iterator ops just for sparse_filter_sub_sub_view usage
  typedef decltype(m_row.begin())     iterator;

  auto
  begin(void) -> decltype(m_row.begin())
  {
    return m_row.begin();
  }

  auto
  end(void) -> decltype(m_row.end())
  {
    return m_row.end();
  }

  // FIXME - generalize;
  typedef row_slice_subview<MTLRowSlice>       subview_type;
  typedef subview_type                         reference;
  typedef size_t                               cid_type;
  typedef double                               value_type;

  row_slice_view(MTLRowSlice const& row)
    : m_row(row)
  { }

  void define_type(typer&)
  {
    stapl_assert(0, "You can ship row_slice_view yet...");
  }

  // Factory Required Methods
  size_t get_num_local_subviews() const
  {
    return 1;
  }

  size_t get_num_subviews() const
  {
    // FIXME - not correct for parallel, instead, num of pcolumns?
    return 1;
  }

  detail::component_holder
  get_local_component(size_t idx) const
  {
    stapl_assert(idx == 0, "row_view::get_elemet_component: idx != 0");

    return detail::component_holder(0);
  }

  subview_type get_subview(cid_type idx) const
  {
    return subview_type(m_row /*, idx*/);
  }

  size_t size() const
  {
    return m_row.end() - m_row.begin();
  }

  // Task Graph (i.e., add_task) Required Methods
  std::pair<location_type, loc_qual>
  get_preferred_location(cid_type idx)
  {
    // FIXME broken on nested
    // row_view shouldn't be called in nested context, so ok
    //
    return std::pair<location_type, loc_qual>(get_location_id(), LQ_CERTAIN);
  }
}; // struct row_slice_view


// view passed to coarse_map_wf of map_func
template<typename Container>
struct coarse_row_view
  : public subview_base
{
  Container&   m_ct;
  std::size_t  m_idx;

  typedef coarse_row_view                                        fast_view_type;

  //typedef typename Container::b_container_t::/*eonst_*/iterator  iterator;
  //typedef typename Container::b_container_t::reference           reference;

  // typedef typename Container::b_container_t::reference           base_ref_t;

  typedef decltype(*(m_ct.bcontainer().begin()))                 base_ref_t;

  typedef row_slice_view<base_ref_t>                             reference;

  struct iterator
  {
    typedef typename Container::b_container_t::iterator base_iter_t;

    base_iter_t m_base_iter;

    iterator(base_iter_t const& iter)
      : m_base_iter(iter)
    { }

    iterator&
    operator++(void)
    {
      ++m_base_iter;
      return *this;
    }

    row_slice_view<base_ref_t>
    operator*(void) const
    {
      return reference(*m_base_iter);
    }

    bool operator!=(iterator const& rhs) const
    {
      return m_base_iter != rhs.m_base_iter;
    }
  };

  coarse_row_view(Container& ct, std::size_t idx)
    : m_ct(ct), m_idx(idx)
  {
    stapl_assert(m_idx == m_ct.my_prow(), "coarse_row_view: invalid idx");
  }

  iterator begin() const
  {
    return iterator(m_ct.bcontainer().begin());
  }

  iterator end() const
  {
    return m_ct.bcontainer().end();
  }

  size_t size() const
  {
    return m_ct.bcontainer().nrows();
  }
}; // struct coarse_row_view


// view passed to outer matvec tg (e.g., map_func);
template<typename Container>
struct row_view
{
  typedef my_matrix_view<Container> base_view_type;

  Container& m_ct;

  void define_type(typer&)
  {
    stapl_assert(0, "Why are you packing row_view?");
  }

  // Factory Required Typedefs...
  typedef detail::coarse_row_view<Container>   subview_type;
  typedef subview_type                         reference;
  typedef size_t                               cid_type;
  typedef double                               value_type;

  row_view(Container& ct)
    : m_ct(ct)
  { }

  //
  // Factory Required Methods
  //
  size_t get_num_local_subviews() const
  {
    if (m_ct.my_pcol() == 0)
      return 1;

    return 0;
  }

  size_t get_num_subviews() const
  {
    return m_ct.nprows();
  }

  detail::component_holder
  get_local_component(size_t idx) const
  {
    stapl_assert(idx == 0, "row_view::get_elemet_component: idx != 0");

    return detail::component_holder(m_ct.my_prow());
  }

  subview_type get_subview(cid_type idx) const
  {
    return subview_type(m_ct, idx);
  }

  //
  // Task Graph (i.e., add_task) Required Methods
  //
  std::pair<location_type, loc_qual>
  get_preferred_location(cid_type idx)
  {
    stapl_assert(idx == m_ct.my_prow() && m_ct.my_pcol() == 0,
      "row_view::get_preferred_location doesn't like that.");

    // FIXME broken on nested
    // row_view shouldn't be called in nested context, so ok
    //
    return std::pair<location_type, loc_qual>(get_location_id(), LQ_CERTAIN);
  }
}; // struct row_view


// view passed to coarse_map_wf of transformed map_func
template<typename Container>
struct coarse_column_view
  : public subview_base
{
  Container&   m_ct;
  std::size_t  m_idx;

  typedef coarse_column_view    fast_view_type;

  coarse_column_view(Container& ct, std::size_t idx)
    : m_ct(ct), m_idx(idx)
  {
    stapl_assert(m_idx == m_ct.my_pcol(), "coarse_column_view: invalid idx");
  }
}; // struct coarse_column_view


//
template<typename Container>
struct column_view
{
  Container& m_ct;

  void define_type(typer&)
  {
    stapl_assert(0, "Why are you packing column_view?");
  }

  // Factory Required Typedefs...
  typedef detail::coarse_column_view<Container> subview_type;
  typedef subview_type                          reference;
  typedef size_t                                cid_type;
  typedef double                                value_type;

  column_view(Container& ct)
    : m_ct(ct)
  { }

  //
  // Factory Required Methods
  //
  size_t get_num_local_subviews() const
  {
    if (m_ct.my_prow() == 0)
      return 1;

    return 0;
  }

  size_t get_num_subviews() const
  {
    return m_ct.npcols();
  }

  detail::component_holder
  get_local_component(size_t idx) const
  {
    stapl_assert(idx == 0, "column_view::get_elemet_component: idx != 0");

    return detail::component_holder(m_ct.my_pcol());
  }

  subview_type get_subview(cid_type idx) const
  {
    return subview_type(m_ct, idx);
  }

  //
  // Task Graph (i.e., add_task) Required Methods
  //
  std::pair<location_type, loc_qual>
  get_preferred_location(cid_type idx)
  {
    stapl_assert(idx == m_ct.my_pcol() && m_ct.my_prow() == 0,
      "column_view::get_preferred_location doesn't like that.");

    // FIXME broken on nested
    // column_view shouldn't be called in nested context, so ok
    //
    return std::pair<location_type, loc_qual>(get_location_id(), LQ_CERTAIN);
  }
};


template<typename Container>
struct coarse_proc_col_row_view
  : public subview_base
{
  Container&   m_ct;

  typedef coarse_proc_col_row_view                                        fast_view_type;

  //typedef typename Container::b_container_t::/*eonst_*/iterator  iterator;
  //typedef typename Container::b_container_t::reference           reference;

  // typedef typename Container::b_container_t::reference           base_ref_t;

/*
  typedef decltype(*(m_ct.bcontainer().begin()))                 base_ref_t;

  typedef row_slice_view<base_ref_t>                             reference;

  struct iterator
  {
    typedef typename Container::b_container_t::iterator base_iter_t;

    base_iter_t m_base_iter;

    iterator(base_iter_t const& iter)
      : m_base_iter(iter)
    { }

    iterator&
    operator++(void)
    {
      ++m_base_iter;
      return *this;
    }

    row_slice_view<base_ref_t>
    operator*(void) const
    {
      return reference(*m_base_iter);
    }

    bool operator!=(iterator const& rhs) const
    {
      return m_base_iter != rhs.m_base_iter;
    }
  };
*/

  coarse_proc_col_row_view(Container& ct, std::size_t my_row, std::size_t my_col)
    : m_ct(ct)
  {
    stapl_assert(my_row == m_ct.my_prow(), "coarse_proc_col_row_view: invalid prow");
    stapl_assert(my_col == m_ct.my_pcol(), "coarse_proc_col_row_view: invalid pcol");
  }

/*  iterator begin() const
  {
    return iterator(m_ct.bcontainer().begin());
  }

  iterator end() const
  {
    return m_ct.bcontainer().end();
  }

  size_t size() const
  {
    return m_ct.bcontainer().nrows();
  }
*/
}; // struct coarse_proc_col_row_view


template<typename Container>
struct proc_col_row_view
{
  Container*    m_ct_ptr;
  std::size_t   m_pcol; // Column subview ID

  void define_type(typer& t)
  {
    std::cout << "proc_col_row_view define_type called\n";

    t.member(m_ct_ptr);
    t.member(m_pcol);
  }

  // Factory Required Typedefs...
  typedef detail::coarse_proc_col_row_view<Container>   subview_type;
  typedef subview_type                                  reference;
  typedef size_t                                        cid_type;
  typedef double                                        value_type;

  proc_col_row_view(Container& ct, std::size_t idx)
    : m_ct_ptr(&ct), m_pcol(idx)
  { }

  //
  // Factory Required Methods
  //
  size_t get_num_local_subviews() const
  {
    if (m_ct_ptr->my_prow() == 0)
      return m_ct_ptr->nprows();

    return 0;
  }

  size_t get_num_subviews() const
  {
    return m_ct_ptr->nprows();
  }

  detail::component_holder
  get_local_component(size_t idx) const
  {
    stapl_assert(idx < m_ct_ptr->nprows(),
      "proc_col_row_view::get_elemet_component: idx >= nprows");

    return detail::component_holder(idx);
  }

  subview_type get_subview(cid_type idx) const
  {
    return subview_type(*m_ct_ptr, idx, m_pcol); // idx is prow.
  }

  //
  // Task Graph (i.e., add_task) Required Methods
  //
  std::pair<location_type, loc_qual>
  get_preferred_location(cid_type idx)
  {
    stapl_assert(m_ct_ptr->my_prow() == 0,
      "proc_col_row_view::get_preferred_location doesn't like that.");

    // compute linearized location id
    location_type loc = m_ct_ptr->npcols() * idx + m_pcol;

    // FIXME broken on nested
    // row_view shouldn't be called in nested context, so ok
    //
    return std::pair<location_type, loc_qual>(loc, LQ_CERTAIN);
  }
}; // struct proc_col_row_view

//
// Take in a view of matrix rows and return a view of that matrix's columns.
//
template<typename Container>
column_view<Container>
transpose(row_view<Container> const& rv)
{
  return column_view<Container>(rv.m_ct);
}

//
// Given a view of matrix columns, transpose to give a view of
// all the partial rows represented by them.
//
template<typename Container>
proc_col_row_view<Container>
transpose(coarse_column_view<Container>& cv)
{
  return proc_col_row_view<Container>(cv.m_ct, cv.m_idx);
}


// vector view passed to coarse_map_reduce_wf of the nested inner_product
template<typename Dense1DView, typename Sparse1DView>
struct sparse_filter_sub_sub_view
  : public subview_base
{
  Dense1DView  m_base_view;
  Sparse1DView m_sparse_view;

  sparse_filter_sub_sub_view(Dense1DView const& view1,
                             Sparse1DView const& view2)
    : m_base_view(view1), m_sparse_view(view2)
  { }

  typedef sparse_filter_sub_sub_view fast_view_type;

  // FIXME - do I like this? coarse_map_reduce_wf wants it
  typedef double& reference;

  struct iterator
  {
  private:
    typedef typename Dense1DView::iterator  dense_iter_t;
    typedef typename Sparse1DView::iterator sparse_iter_t;

    dense_iter_t    m_base_iter;
    sparse_iter_t   m_sparse_iter;

  public:
    iterator(dense_iter_t const& base_iter,
             sparse_iter_t  const& sparse_iter)
      : m_base_iter(base_iter),
        m_sparse_iter(sparse_iter)
    { }

    // FIXME double& to something else
    double& operator*()
    {
      return *(m_base_iter + m_sparse_iter.column());
    }

    iterator& operator++()
    {
      ++m_sparse_iter;
      return *this;
    }
  }; // struct iterator

  iterator begin()
  {
    return iterator(m_base_view.begin(), m_sparse_view.begin());
  }
};


// vector view passed to the inner matvec tg (e.g., inner_product)
template<typename Dense1DView, typename Sparse1DView>
struct sparse_filter_sub_view
{
  Dense1DView  m_base_view;
  Sparse1DView m_sparse_view;

  sparse_filter_sub_view(Dense1DView const& view1,
                         Sparse1DView const& view2)
    : m_base_view(view1), m_sparse_view(view2)
  { }

  void define_type(typer&)
  {
    stapl_assert(0, "You can ship sparse_filter_sub_view yet...");
  }

  typedef sparse_filter_sub_sub_view<Dense1DView, Sparse1DView>  subview_type;
  typedef subview_type                                           reference;
  typedef size_t                                                 cid_type;
  typedef double                                                 value_type;

  // Factory Required Methods
  size_t get_num_local_subviews() const
  {
    return 1;
  }

  size_t get_num_subviews() const
  {
    // FIXME - no correcto for parallelo
    return 1;
  }

  detail::component_holder
  get_local_component(size_t idx) const
  {
    stapl_assert(idx == 0,
      "sparse_filter_view::get_elemet_component: idx != 0");

    return detail::component_holder(idx);
  }

  subview_type
  get_subview(cid_type idx) const
  {
    return subview_type(m_base_view, m_sparse_view);
  }

  size_t size() const
  {
    return m_sparse_view.size();
  }

  // Task Graph (i.e., add_task) Required Methods
  std::pair<location_type, loc_qual>
  get_preferred_location(cid_type idx)
  {
    return std::pair<location_type, loc_qual>(get_location_id(), LQ_CERTAIN);
  }
}; // struct sparse_filter_sub_view


// The View Passed to coarse_map_wf
template<typename Dense1DView, typename Sparse1DView>
struct coarse_sparse_filter_view
  : public subview_base
{
private:
  Dense1DView   m_base_view;
  Sparse1DView  m_sparse_view;

public:
  typedef coarse_sparse_filter_view                                  fast_view_type;
  typedef typename Dense1DView::reference                            reference;

  coarse_sparse_filter_view(Dense1DView const& base_view,
                            Sparse1DView const& sparse_view)
    : m_base_view(base_view),
      m_sparse_view(sparse_view)
  { }

  size_t size() const
  {
    return m_sparse_view.size();
  }

  struct iterator
  {
  private:
    typedef typename Dense1DView::iterator  base_iter_t;
    typedef typename Sparse1DView::iterator sparse_iter_t;

    base_iter_t   m_base_iter;
    sparse_iter_t m_sparse_iter;

  public:
    iterator(base_iter_t const& base_iter,
             sparse_iter_t const& sparse_iter)
      : m_base_iter(base_iter),
        m_sparse_iter(sparse_iter)
    { }

    // FIXME - why is this different...
    typedef sparse_filter_sub_view<decltype(*m_base_iter), decltype(*m_sparse_iter)> reference;

    reference operator*()
    {
      return reference(*m_base_iter, *m_sparse_iter);
    }

    iterator& operator++()
    {
      ++m_base_iter;
      ++m_sparse_iter;
      return *this;
    }
  }; // struct iterator

  iterator begin()
  {
    return iterator(m_base_view.begin(), m_sparse_view.begin());
  }

  iterator end()
  {
    return iterator(m_base_view.end(), m_sparse_view.end());
  }
}; // struct coarse_sparse_filter_view


// The Coarsened Repeated Vector View
template<typename Dense1DView, typename Sparse1DView>
struct sparse_filter_view
{
  Dense1DView   m_base_view;
  Sparse1DView  m_sparse_view;

  void define_type(typer&)
  {
    stapl_assert(0, "Why are you packing sparse_filter_view?");
  }

  // Factory Required Typedefs...
  typedef typename Dense1DView::subview_type   d_sv_t;
  typedef typename Sparse1DView::subview_type  s_sv_t;

  typedef detail::coarse_sparse_filter_view<d_sv_t, s_sv_t>  subview_type;
  typedef subview_type                                       reference;
  typedef size_t                                             cid_type;
  typedef double                                             value_type;

  sparse_filter_view(Dense1DView const& base_view,
                     Sparse1DView const& sparse_view)
    : m_base_view(base_view), m_sparse_view(sparse_view)
  { }

  //
  // Factory Required Methods
  //
  size_t get_num_local_subviews() const
  {
    return m_base_view.get_num_local_subviews();
  }

  size_t get_num_subviews() const
  {
    return m_base_view.get_num_subviews();
  }

  detail::component_holder
  get_local_component(size_t idx) const
  {
    stapl_assert(idx == 0,
      "sparse_filter_view::get_elemet_component: idx != 0");

    return detail::component_holder(idx);
  }

  subview_type
  get_subview(cid_type idx) const
  {
    return subview_type(m_base_view.get_subview(idx),
                        m_sparse_view.get_subview(idx));
  }

  //
  // Task Graph (i.e., add_task) Required Methods
  //
  std::pair<location_type, loc_qual>
  get_preferred_location(cid_type idx)
  {
    return std::pair<location_type, loc_qual>(get_location_id(), LQ_CERTAIN);
  }

}; // struct sparse_filter_view

} // namespace detail


template<typename Container>
struct my_matrix_subview
  : public detail::subview_base
{
  typedef Container container_t;

  Container* m_ct;

  typedef my_matrix_subview fast_view_type;

  my_matrix_subview()
    : m_ct(0)
  { }

  my_matrix_subview(Container& ct)
    : m_ct(&ct)
  { }
};


template<typename Container>
struct my_matrix_view
{
  Container& m_ct;

  typedef my_matrix_subview<Container> subview_type;
  typedef size_t                       cid_type;

  my_matrix_view(Container& ct)
    : m_ct(ct)
  { }

  void define_type(typer&)
  {
    stapl_assert(0, "trying to pack my_matrix_view");
  }

  detail::row_view<Container>
  rows() const
  {
    return detail::row_view<Container>(m_ct);
  }

  detail::column_view<Container>
  columns() const
  {
    return detail::column_view<Container>(m_ct);
  }

  subview_type
  get_subview(cid_type) const
  {
    return subview_type(m_ct);
  }

  std::pair<location_type, loc_qual>
  get_preferred_location(cid_type idx)
  {
    return std::pair<location_type, loc_qual>(get_location_id(), LQ_CERTAIN);
  }

  ~my_matrix_view()
  {
    // FIXME - need to refcount and delete m_ct (it's heap allocated,
    // and I effectively take ownership...
  }
}; // struct matrix_view

} // namespace stapl

#endif // ifndef MATRIX_VIEW_HPP
