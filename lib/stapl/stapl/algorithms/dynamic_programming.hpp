/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_DYNAMIC_PROGRAMMING_HPP
#define STAPL_ALGORITHMS_DYNAMIC_PROGRAMMING_HPP

#include <stapl/paragraph/paragraph_fwd.h>
#include <stapl/paragraph/task_factory_base.hpp>
#include <stapl/views/cross_view.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Represents the coarsening of a cross_view into elements of a
///   specified block size.
/// @tparam CrossView Tuple containing the fine-grained cross_view type.
///
/// An instance of the coarsened cross view is passed to sequence_comp_factory
/// to allow tasks that process multiple points in the input of a dynamic
/// programming algorithm.
///
/// @bug The performance of dynamic programming algorithms in STAPL is poor.  It
///   is suspected that the on-the-fly coarsening may be contributing to this.
/// @remark See Bug #833 in the STAPL gForge project.
//////////////////////////////////////////////////////////////////////
template <typename CrossView>
struct coarsened_cross_view
  : public p_object
{
private:
  typedef typename tuple_element<0, CrossView>::type fine_view_type;

  /// Fine-grained view passed as input of a dynamic programming algorithm.
  fine_view_type m_fine_view;

  /// Factor by which the fine grained view is coarsened.
  std::size_t    m_blk_sz;

public:
  typedef typename fine_view_type::index_type      index_type;
  typedef typename fine_view_type::size_type       size_type;
  typedef typename fine_view_type::dimensions_type dimensions_type;
  typedef fine_view_type                           reference;
  typedef fine_view_type                           value_type;
  typedef typename fine_view_type::index_type      gid_type;
  typedef typename fine_view_type::index_type      cid_type;
  typedef typename fine_view_type::domain_type     domain_type;
  typedef identity<index_type>                     map_function;
  typedef fine_view_type                           subview_type;

public:
  coarsened_cross_view(fine_view_type const& view, std::size_t blk_sz)
    : m_fine_view(view), m_blk_sz(blk_sz)
  {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the coarsened view.
  /// @return tuple of two elements, each containing the number of coarsened
  ///   view elements in a dimension.
  //////////////////////////////////////////////////////////////////////
  dimensions_type dimensions() const
  {
    dimensions_type fine_sz = m_fine_view.dimensions();
    return dimensions_type(get<0>(fine_sz)/m_blk_sz, get<1>(fine_sz)/m_blk_sz);
  }

  locality_info locality(index_type)
  {
    return locality_info(
      LQ_CERTAIN, get_affinity(),
      this->get_rmi_handle(), this->get_location_id()
    );
  }

  bool is_local(void)
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief PARAGRAPH tasks call pre_execute on each view before invoking the
  ///   work function operator.  This view doesn't require any operation.
  //////////////////////////////////////////////////////////////////////
  void pre_execute(void)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief PARAGRAPH tasks call post_execute on each view after the task's
  ///   work function operator returns.  This view doesn't require any
  ///   operation.
  //////////////////////////////////////////////////////////////////////
  void post_execute(void)
  {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the coarsened subview requested.
  /// @param i two-dimensional index of the requested coarse view element.
  /// @return coarse view element of m_blk_sz*m_blk_sz fine-grain elements.
  //////////////////////////////////////////////////////////////////////
  subview_type get_subview(cid_type& i) const
  {
    // Construct a cross_view over the subset of the container.
    typedef typename domain_type::index_type index_t;
    return subview_type(m_fine_view.container(),
             domain_type(index_t(m_blk_sz*get<0>(i), m_blk_sz*get<1>(i)),
               index_t(m_blk_sz*(get<0>(i)+1)-1, m_blk_sz*(get<1>(i)+1)-1)));
  }

  void define_type(typer& t)
  {
    t.member(m_fine_view);
    t.member(m_blk_sz);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to construct a coarse_cross_view during PARAGRAPH
///   construction.
///
/// @todo Extend to allow sequence_comp_factory to partition the id set of the
///   resulting view to parallelize task creation.
//////////////////////////////////////////////////////////////////////
struct cross_view_coarsener
{
private:
  /// Factor by which the fine grained view is coarsened.
  std::size_t m_blk_sz;

public:
  cross_view_coarsener(std::size_t blk_sz)
    : m_blk_sz(blk_sz)
  { }

  template <typename View>
  tuple<coarsened_cross_view<View>>
  operator()(View const& view) const
  {
    return tuple<coarsened_cross_view<View>>(
             coarsened_cross_view<View>(get<0>(view), m_blk_sz));
  }
};


/*
 This code can be used to filter task results when the edge_view support for
 filtering is generalized.  The support required by this factory is:

 1. Support for local filters.  We're overpartitioning a row, and would like
    to filter the result to extract the right face.

 2. Support for multiple filters from the same location.  Two tasks on the row
    below the current task will request that its result be filtered.  One task
    wants to extract the bottom face while the other wants the corner element.

template <typename Result>
struct filter_dp_result;

template <typename Result>
inline bool operator==(filter_dp_result<Result> const&,
                       filter_dp_result<Result> const&);

template <typename Result>
struct filter_dp_result
{
protected:
  std::size_t m_element;

  template <typename T>
  friend bool operator==(filter_dp_result<T> const&,
                         filter_dp_result<T> const&);
public:
  typedef Result result_type;

  filter_dp_result(std::size_t element)
    : m_element(element)
  {
    stapl_assert(m_element != 0,
                 "Filtering the left task result would create a local filter.");
  }

  void define_type(typer& t)
  {
    t.member(m_element);
  }

  Result operator()(Result const& result) const
  {
    Result res(3);
    res[m_element] = result[m_element];
    return res;
  }
};


template <typename Result>
inline bool operator==(filter_dp_result<Result> const& lhs,
                       filter_dp_result<Result> const& rhs)
{
  return lhs.m_element == rhs.m_element;
}
*/

//////////////////////////////////////////////////////////////////////
/// @brief Functor that implements a coarse-grained work function in a dynamic
///   programming algorithm by wrapping the fine-grained operation in a loop
///   over the block of elements that make up a coarsened cross view element.
//////////////////////////////////////////////////////////////////////
template <typename FineWF>
struct coarse_dp_wf
{
private:
  FineWF m_fine_wf;

public:
  typedef std::vector<std::vector<typename FineWF::result_type> > result_type;

  coarse_dp_wf(FineWF const& wf)
    : m_fine_wf(wf)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Combines the results of the fine-grained operations to form the
  ///   result of the coarse-grained task.
  /// @param fine_results results of the fine-grained operations on the elements
  ///   of the coarse cross view element that was processed.
  /// @param dom indices of the rows and columns, processed by the task.
  /// @param dims size of the coarse-grained element processed by the task.
  /// @return a vector of fine-grained results representing the right face,
  ///   bottom face, and corner element of the coarsened task.
  //////////////////////////////////////////////////////////////////////
  template <typename FineResults, typename Domain, typename DomainSize>
  result_type construct_return(FineResults const& fine_results,
                               Domain const& dom, DomainSize const& dims) const
  {
    // Prepare return of vec<vec<res>> where order is (right, bottom, corner).
    result_type res(3);

    typedef typename std::tuple_element<0, typename Domain::index_type>::type
      index_type;

    // Right face
    int i_idx(0);
    int j_idx(get<1>(dims)-1);
    for (index_type i = get<0>(dom.first());
         i != get<0>(dom.last())+1; ++i, ++i_idx)
    {
      res[0].push_back(fine_results[i_idx][j_idx]);
    }

    // Bottom face
    i_idx = get<0>(dims)-1;
    j_idx = 0;
    for (index_type j = get<1>(dom.first());
         j != get<1>(dom.last())+1; ++j, ++j_idx)
    {
      res[1].push_back(fine_results[i_idx][j_idx]);
    }

    // Corner element
    res[2].push_back(fine_results.back().back());

    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the fine-grained operation on each element contained in a
  ///   coarsened cross view element.
  /// @param e Element of a coarsened cross view.
  /// @return vector representing the results of the fine-grained operations
  ///   across the bottom face, right face, and lower right corner element of
  ///   the coarsened view element.
  //////////////////////////////////////////////////////////////////////
  template <typename Element>
  result_type operator()(Element e) const
  {
    typename Element::dimensions_type dims = e.dimensions();

    typedef typename Element::domain_type dom_type;
    typedef typename std::tuple_element<0, typename dom_type::index_type>::type
      index_type;

    dom_type dom = e.domain();

    std::vector<std::vector<typename FineWF::result_type> >
      fine_results(get<0>(dims),
        std::vector<typename FineWF::result_type>(get<1>(dims),
          typename FineWF::result_type()));

    int i_idx(0);
    for (index_type i = get<0>(dom.first()); i != get<0>(dom.last())+1;
         ++i, ++i_idx)
    {
      int j_idx(0);
      for (index_type j = get<1>(dom.first());
           j != get<1>(dom.last())+1; ++j, ++j_idx)
      {
        if (i_idx!=0 && j_idx!=0) {
          // Element is in the middle of the view.
          fine_results[i_idx][j_idx] =
            m_fine_wf(e[typename Element::index_type(i,j)],
                      fine_results[i_idx][j_idx-1],
                      fine_results[i_idx-1][j_idx],
                      fine_results[i_idx-1][j_idx-1]);
        } else if (i_idx==0 && j_idx!=0) {
          // Element is on the first row and only has one predecessor.
          fine_results[i_idx][j_idx] =
            m_fine_wf(e[typename Element::index_type(i,j)],
                      fine_results[i_idx][j_idx-1]);
        } else if (i_idx!=0 && j_idx==0) {
          // Element is on the first column and only has one predecessor.
          fine_results[i_idx][j_idx] =
            m_fine_wf(e[typename Element::index_type(i,j)],
                      fine_results[i_idx-1][j_idx]);
        } else {
          // process the first element.
          fine_results[i_idx][j_idx] =
            m_fine_wf(e[typename Element::index_type(i,j)]);
        }
      }
    }

    return construct_return(fine_results, dom, dims);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the fine-grained operation on each element contained in a
  ///   coarse cross view element using the result received from a preceding
  ///   coarse task.
  /// @param e Element of a coarsened cross view.
  /// @param r1 Result of a preceding coarse task representing input along
  ///   either the left or top faces of the coarse view to process.
  /// @return vector representing the results of the fine-grained operations
  ///   across the bottom face, right face, and lower right corner element of
  ///   the coarsened view element.
  //////////////////////////////////////////////////////////////////////
  template <typename Element, typename Res1>
  result_type operator()(Element e, Res1 r1) const
  {
    typename Element::dimensions_type dims = e.dimensions();

    typedef typename Element::domain_type dom_type;
    typedef typename std::tuple_element<0, typename dom_type::index_type>::type
      index_type;

    dom_type dom = e.domain();

    std::vector<std::vector<typename FineWF::result_type> >
      fine_results(get<0>(dims),
        std::vector<typename FineWF::result_type>(get<1>(dims),
          typename FineWF::result_type()));

    int i_idx(0), j_idx(0);

    // determine if the result is on the left or top face.
    if (get<0>(dom.first()) == 0) {
      // Task is on the top block of rows.  Result is on the left face.
      index_type i = get<0>(dom.first());
      index_type j = get<1>(dom.first());

      // First element
      fine_results[i_idx][j_idx] =
        m_fine_wf(e[typename Element::index_type(i,j)],
                  r1[0][i_idx]);
      // The rest of the column
      ++i;
      ++i_idx;
      for (; i != get<0>(dom.last())+1; ++i, ++i_idx)
      {
        fine_results[i_idx][j_idx] =
          m_fine_wf(e[typename Element::index_type(i,j)],
                    r1[0][i_idx], fine_results[i_idx-1][j_idx], r1[0][i_idx-1]);
      }
      for (i = get<0>(dom.first()), i_idx = 0;
           i != get<0>(dom.last())+1; ++i, ++i_idx)
      {
        // The first column has already been computed.
        for (j = get<1>(dom.first())+1, j_idx = 1;
             j != get<1>(dom.last())+1; ++j, ++j_idx)
        {
          if (i_idx!=0 && j_idx!=0) {
            // Element is in the middle of the view.
            fine_results[i_idx][j_idx] =
              m_fine_wf(e[typename Element::index_type(i,j)],
                        fine_results[i_idx][j_idx-1],
                        fine_results[i_idx-1][j_idx],
                        fine_results[i_idx-1][j_idx-1]);
          } else {
            // (i_idx==0 && j_idx!=0)
            // Element is on the first row and only has one predecessor.
            fine_results[i_idx][j_idx] =
              m_fine_wf(e[typename Element::index_type(i,j)],
                        fine_results[i_idx][j_idx-1]);
          }
        }
      }
    } else {
      // Task is on the left block of columns.  Result is on the top face.
      index_type i = get<0>(dom.first());
      index_type j = get<1>(dom.first());

      // First element
      fine_results[i_idx][j_idx] =
        m_fine_wf(e[typename Element::index_type(i,j)],
                  r1[1][j_idx]);
      // The rest of the row
      ++j;
      ++j_idx;
      for (; j != get<1>(dom.last())+1; ++j, ++j_idx)
      {
        fine_results[i_idx][j_idx] =
          m_fine_wf(e[typename Element::index_type(i,j)],
                    fine_results[i_idx][j_idx-1], r1[1][j_idx], r1[1][j_idx-1]);
      }
      // The first row has already been computed.
      for (i = get<0>(dom.first())+1, i_idx = 1;
           i != get<0>(dom.last())+1; ++i, ++i_idx)
      {
        for (j = get<1>(dom.first()), j_idx = 0;
             j != get<1>(dom.last())+1; ++j, ++j_idx)
        {
          if (i_idx!=0 && j_idx!=0) {
            // Element is in the middle of the view.
            fine_results[i_idx][j_idx] =
              m_fine_wf(e[typename Element::index_type(i,j)],
                        fine_results[i_idx][j_idx-1],
                        fine_results[i_idx-1][j_idx],
                        fine_results[i_idx-1][j_idx-1]);
          } else {
            // (i_idx!=0 && j_idx==0)
            // Element is on the first column and only has one predecessor.
            fine_results[i_idx][j_idx] =
              m_fine_wf(e[typename Element::index_type(i,j)],
                        fine_results[i_idx-1][j_idx]);
          }
        }
      }
    }

    return construct_return(fine_results, dom, dims);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the fine-grained operation on each element contained in a
  ///   coarse cross view element using the result received from a preceding
  ///   coarse task.
  /// @param e Element of a coarsened cross view.
  /// @param r1,r2,r3 Result of a preceding coarse task representing input
  ///   along the left face, top face, and corner, respectively.
  /// @return vector representing the results of the fine-grained operations
  ///   across the bottom face, right face, and lower right corner element of
  ///   the coarsened view element.
  //////////////////////////////////////////////////////////////////////
  template <typename Element, typename Res1, typename Res2, typename Res3>
  result_type operator()(Element e, Res1 r1, Res2 r2, Res3 r3) const
  {
    typedef typename Element::domain_type dom_t;

    typename Element::dimensions_type dims = e.dimensions();

    dom_t dom = e.domain();

    std::vector<std::vector<typename FineWF::result_type> >
      fine_results(get<0>(dims),
        std::vector<typename FineWF::result_type>(get<1>(dims),
          typename FineWF::result_type()));

    int i_idx(0), j_idx(0);
    typename std::tuple_element<0, typename dom_t::index_type>::type i =
      get<0>(dom.first());
    typename std::tuple_element<1, typename dom_t::index_type>::type j =
      get<1>(dom.first());

    // Compute the top corner.
    fine_results[i_idx][j_idx] =
      m_fine_wf(e[typename Element::index_type(i,j)],
                  r1[0][i_idx], r2[1][j_idx], r3[2][0]);

    // Compute the first row.
    for (j  = get<1>(dom.first())+1, j_idx = 1;
         j != get<1>(dom.last())+1; ++j, ++j_idx)
    {
      fine_results[i_idx][j_idx] =
        m_fine_wf(e[typename Element::index_type(i,j)],
                  fine_results[i_idx][j_idx-1], r2[1][j_idx], r2[1][j_idx-1]);
    }

    // Compute the first column.
    j_idx = 0;
    j = get<1>(dom.first());
    for (i  = get<0>(dom.first())+1, i_idx = 1;
         i != get<0>(dom.last())+1; ++i, ++i_idx)
    {
      fine_results[i_idx][j_idx] =
        m_fine_wf(e[typename Element::index_type(i,j)],
                  r1[0][i_idx], fine_results[i_idx-1][j_idx], r1[0][i_idx-1]);
    }

    // Compute the rest of the table.
    i_idx = 1;
    i = get<0>(dom.first()) + 1;
    for (; i != get<0>(dom.last())+1; ++i, ++i_idx)
    {
      j_idx = 1;
      j = get<1>(dom.first()) + 1;
      for (; j != get<1>(dom.last())+1; ++j, ++j_idx)
      {
        fine_results[i_idx][j_idx] =
          m_fine_wf(e[typename Element::index_type(i,j)],
                    fine_results[i_idx][j_idx-1], fine_results[i_idx-1][j_idx],
                    fine_results[i_idx-1][j_idx-1]);
      }
    }

    return construct_return(fine_results, dom, dims);
  }

  void define_type(typer& t)
  { t.member(m_fine_wf); }
};


template<typename T>
struct dp_result_wf
{
  typedef T result_type;

  template<typename Ref>
  T operator()(Ref x) const
  {
    return x;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Factory functor that generates the tasks to perform a dynamic
///   programming algorithm over data represented by a cross_view.
///
/// The task graph generated is a 2D mesh with each task depending on the tasks
/// that processed the elements to the left, top, and upper left corner of the
/// view element to be processed.
///
/// @todo Determine if filtering support has been generalized to the point that
///   it can be used on the results of the tasks generated.
//////////////////////////////////////////////////////////////////////
template <typename WF>
struct sequence_comp_factory
  : public task_factory_base,
    private coarse_dp_wf<WF>
{
  typedef cross_view_coarsener                   coarsener_type;
  typedef typename coarse_dp_wf<WF>::result_type result_type;

private:
  // See @todo.
  // typedef filter_dp_result<result_type>          filter_type;

  /// @brief Coarsener employed by PARAGRAPH using this factory.  Requires the
  ///   input view be a cross_view.
  cross_view_coarsener                           m_coarsener;

public:
  sequence_comp_factory(WF const& wf, size_t blk_sz)
    : coarse_dp_wf<WF>(wf),
      m_coarsener(blk_sz)
  { }

  coarsener_type get_coarsener() const
  { return m_coarsener; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs the task graph for a dynamic programming computation.
  /// @param tgv The PARAGRAPH view.
  /// @param view Coarsened cross_view of the input elements.
  ///
  /// Task ids are assigned in row major order.  For example given two strings
  /// of length 16 and a block size of 4 the following task graph is produced.
  /// Data flow is from top left to bottom right.
  /// @verbatim
  ///  0- 1- 2- 3
  ///  |\ |\ |\ |
  ///  4- 5- 6- 7
  ///  |\ |\ |\ |
  ///  8- 9-10-11
  ///  |\ |\ |\ |
  /// 12-13-14-15
  /// @endverbatim
  //////////////////////////////////////////////////////////////////////
  template <typename TaskGraphView, typename View2D>
  void operator()(TaskGraphView const& tgv, View2D& view)
  {
    // coarsening the view should enable the ability to partition the
    // id set of the view.  Until we're coarsening the view we'll do a cyclic
    // assignment of the first dimension across locations.
    std::size_t loc_id = get_location_id();
    std::size_t nlocs  = get_num_locations();

    tuple<std::size_t, std::size_t> size = view.dimensions();
    typedef tuple<std::size_t, std::size_t> view_index_type;

    coarse_dp_wf<WF> wf(static_cast<coarse_dp_wf<WF> const&>(*this));

    if (get<0>(size) > loc_id)
    {
      // For each row of view
      for (std::size_t row = loc_id; row < get<0>(size); row += nlocs)
      {
        std::size_t task_id = row * get<1>(size);

        // Generate a task for all elements of seq1.
        for (std::size_t i = 0; i != get<1>(size); ++i, ++task_id)
        {
          std::size_t num_successors = 3;
          if (i == get<1>(size)-1 || row == get<0>(size)-1)
            num_successors -= 2; // we're on the right or bottom boundary

          if (i == get<1>(size)-1 && row == get<0>(size)-1)
            num_successors = 1; // we're on the bottom right corner

          // Note: we don't filter the results because we can't be sure which
          // predecessors are on the same location (e.g., np == 1), and the
          // edge_view doesn't currently support filtering local results.
          if (row != 0 && i != 0)
          {
            // Middle of subproblem space.  Task consumes three results.
            tgv.add_task(task_id, wf, num_successors,
              std::make_pair(&view, view_index_type(row, i)),
              consume<result_type>(tgv, task_id - 1),
              // filter_type(0)
              consume<result_type>(tgv, task_id - get<1>(size)),
              // filter_type(1)
              consume<result_type>(tgv, task_id - get<1>(size)-1));
              //filter_type(2)
          }
          else if (row == 0 && i != 0)
          {
            // Top row of subproblems.  Task consumes from left only.
            tgv.add_task(task_id, wf, num_successors,
              std::make_pair(&view, view_index_type(row, i)),
              consume<result_type>(tgv, task_id - 1)); // filter_type(0)
          }
          else if (row != 0 && i == 0)
          {
            // Left column of subproblems.  Task consumes from above only.
            tgv.add_task(task_id, wf, num_successors,
              std::make_pair(&view, view_index_type(row, i)),
              consume<result_type>(tgv, task_id - get<1>(size)));
              // filter_type(1)
          }
          else
          {
            // First subproblem.  Task doesn't consume any previous result.
            tgv.add_task(task_id, wf, num_successors,
              std::make_pair(&view, view_index_type(row, i)));
          }
        }
      }
    }

    // FIXME: We need a smarter broadcast of the result.
    // For now we send it to location 0 and broadcast from there.

    // Task that computes the final result
    std::size_t comp_result = get<0>(size) * get<1>(size) - 1;

    std::size_t broadcast_id = comp_result + 1 + loc_id;

    std::size_t broadcast_pred;
    if (loc_id == 0)
      broadcast_pred = comp_result;
    else
      broadcast_pred = comp_result + 1 + ((loc_id-1)/2);

    // The local return value will consume the result of the broadcast task.
    std::size_t num_succs = 1;

    // Add additional consumers if I'm a non-leaf on the broadcast tree.
    std::size_t nleaves = (nlocs+1)/2;
    if ((loc_id < (nlocs - nleaves)) && (nlocs != 1))
    {
      if (2*loc_id+1 == nlocs-1)
        ++num_succs;
      else
        num_succs += 2;
    }

    tgv.add_task(broadcast_id, dp_result_wf<result_type>(),
                 num_succs, consume<result_type>(tgv, broadcast_pred));

    tgv.set_result(broadcast_id);
  }
};

} // namespace stapl

#endif
