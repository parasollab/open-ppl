/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXPLICIT_NEW_MAP_TRANSFORMATIONS_HPP
#define STAPL_SKELETONS_EXPLICIT_NEW_MAP_TRANSFORMATIONS_HPP

#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/views/metadata/coarseners/default.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

namespace prototype {

namespace functional {

struct inner_product;

} } // namespace prototype::functional


namespace composition {

//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that is one possible algorithmic coarsening of the
/// nested skeleton map_func(functional::map_reduce(), ...).
///
/// Transforms the fine grain specification into a map_reduce operating
/// on a transposed version of the input two dimensional array view, with
/// custom coarsened map and reduce workfunctions.
///
/// Implemented with matrix vector multiplication as driving example.
///
/// @todo This needs to be cleaned and become a default transformation
///       for the case as it is likely always better than the original
///       specification. Generalize naming and assumptions from current
///       use case where the @p map_reduce is an @p inner_product call.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
struct m_mr_transform_wf
{
  typedef std::vector<double>* result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Coarsened binary workfunction which is map operator for
  /// @ref m_mr_transform_wf.
  //////////////////////////////////////////////////////////////////////
  struct map_wf
  {
    typedef std::vector<double>* result_type;

    //////////////////////////////////////////////////////////////////////
    /// @param row_pieces        A row from the transposed two dimensional
    ///                          array view input.
    /// @param rep_vector_piece The row input to the @ref m_mr_transform_wf
    ///                         workfunction.
    ///
    /// @return A heap allocated one dimensional container with result of the
    ///         operator (currently fixed to @p inner_product).
    //////////////////////////////////////////////////////////////////////
    template<typename V0, typename V1>
    result_type
    operator()(V0& row_pieces, V1& rep_vector_piece)
    {
      const size_t num_rows = rep_vector_piece.size();
      const size_t num_cols = (*rep_vector_piece.begin()).size();

      stapl_assert(num_rows > 0 && num_cols > 0,
        "m_mr_transform_wf: rows or cols equal 0");

      result_type tmp = new std::vector<double>(num_rows);

      typename V0::iterator v0_iter          = row_pieces.begin();
      typename V1::iterator v1_iter          = rep_vector_piece.begin();
      std::vector<double>::iterator res_iter = tmp->begin();

      for (std::size_t i = 0; i < num_rows;
           ++i, ++v0_iter, ++v1_iter, ++res_iter)
        *res_iter = std::inner_product(
                      v0_iter->begin(), v0_iter->end(),
                      v1_iter->begin(),
                      std::multiplies<double>(), std::plus<double>()
                    );

      return tmp;
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Coarsened binary reduction workfunction defined for result type
  /// of @ref map_wf.
  ///
  /// @return Mutates storage of lhs with result of reduction and returns it.
  ///
  /// Frees underlying storage of rhs.
  //////////////////////////////////////////////////////////////////////
  struct reduce_wf
  {
    typedef std::vector<double>* result_type;

    result_type
    operator()(result_type lhs, result_type rhs)
    {
      stapl_assert(lhs->size() == rhs->size(),
        "ip_transform::reduce_wf::operator(), lhs / rhs sizes differ");

      std::transform(lhs->begin(), lhs->end(), rhs->begin(),
                     lhs->begin(), std::plus<int>());

      delete rhs;

      return lhs;
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator.
  ///
  /// @param mat_columns A two dimensional array view.
  /// @param vector_piece A one dimensional array view.
  ///
  /// @return A one dimensional array view containing result of the
  /// @p map_reduce operation.
  //////////////////////////////////////////////////////////////////////
  template<typename V0, typename V1>
  result_type operator()(V0& mat_columns, V1& vector_piece) const
  {
    return map_reduce(is_coarse_wf(map_wf()), reduce_wf(),
                      transpose(mat_columns),
                      repeat_view<V1>(vector_piece));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A coarsened workfunction for matrix vector multiplication that
/// operates on a block of a two dimensional matrix and the vector portion
/// corresponding to it.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
struct block_matvec_wf
{
  // typedef cm_res_view<std::vector<double> > res_t;

  typedef cm_res_view<my_container<double> > res_t;


  //////////////////////////////////////////////////////////////////////
  /// @brief Nested struct which reflects the return type of the function
  /// operator based on the types of the input views.
  //////////////////////////////////////////////////////////////////////
  template<typename Signature>
  struct result;


  template<typename SubMatrix, typename SubVector>
  struct result<block_matvec_wf(SubMatrix, SubVector)>
  {
    // Using SubVector would imply map_subview usage.
    // May want make that cm_res_view instead.
    // typedef SubVector type;

    typedef res_t type;
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator.
  ///
  /// @param mv A view of a sub-block of the input matrix.
  /// @param sv_view A view of a piece of the input vector.
  /// @return A view over a container with the result of matrix
  ///   vector multiplication.
  //////////////////////////////////////////////////////////////////////
  template<typename SubMatrix, typename SubVector>
  res_t operator()(SubMatrix& mv, SubVector& sv_view) const
  {
    typedef typename SubMatrix::container_t::b_container_t SequentialMatrix;

    SequentialMatrix* mtl_a = &mv.m_ct->bcontainer();

    res_t result_vw(mtl_a->nrows(), 0);

    typename SequentialMatrix::iterator i        = mtl_a->begin();
    typename SequentialMatrix::iterator i_end    = mtl_a->end();
    typename SubVector::iterator        res_iter = result_vw.begin();

    for (; i != i_end; ++i, ++res_iter)
    {
      double sum = 0.0l;

      typename SequentialMatrix::OneD::iterator j;
      typename SequentialMatrix::OneD::iterator j_end;

      j_end = (*i).end();

      for (j = (*i).begin(); j != j_end; ++j)
        sum += *j * sv_view[j.column()];

      *res_iter = sum;
    }

    return result_vw;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Custom key mapper passed to the @ref directory when using the
/// @ref nest_map_mr_factory factory within a PARAGRAPH.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
struct nest_map_mr_task_mapper
{
private:
  /// @brief The number of processor columns. (nest_map_mr_factory uses
  /// a two dimensional logical processor layout).
  std::size_t m_n_pcols;

  /// @brief True if transparent computation replication has enabled.
  bool        m_b_replicated;

public:
  nest_map_mr_task_mapper(std::size_t n_pcols, bool b_replicated)
    : m_n_pcols(n_pcols), m_b_replicated(b_replicated)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Maps the task identifier @p tid to the location where it will
  /// execute.
  //////////////////////////////////////////////////////////////////////
  size_t operator()(size_t tid) const
  {
    const size_t n_locs = stapl::get_num_locations();

    if (m_b_replicated)
      return tid % n_locs;

    // phase 1 tids...
    if (tid < m_n_pcols)
      return tid;

    // phase 2 tids...
    tid -= m_n_pcols;

    if (tid < n_locs)
      return tid;

    // phase 3 tids...
    tid -= n_locs;

    if (tid < n_locs)
      return tid;

    // phase 4 tids
    tid -= n_locs;

    if (tid < n_locs)
      return tid;

    // unknown tid
    stapl_assert(0, "nest_map_mr_task_mapper: unexpected tid lookup request");

    return 0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Customized implementation of a map_func(map_reduce()...)
/// skeleton that is used for matrix vector multiplication.
///
/// The key implementation features are the use of logical two dimension
/// processor layout to reduce communication and the replication of computation
/// across subportions of processors to further reduce communication caused by
/// vector updates.
///
/// @todo Generalize and extend usage beyond matrix vector multiplication.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
struct nest_map_mr_factory
  : public task_factory_base
{
private:
  typedef composition::coarse_map_wf<identity<double> >    coarse_map_wf_t;
  typedef composition::coarse_map_wf<plus<double> >        coarse_map_plus_wf_t;
  typedef skeletons::tags::nested_map_reduce                tag_type;
public:
//  typedef cm_res_view<std::vector<double> >                sv_t;
  typedef cm_res_view<my_container<double> >  sv_t;
  using coarsener_type = default_coarsener;

  default_coarsener get_coarsener() const { return default_coarsener(); }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Factory implementation called by function operator if computation
  /// replication is disabled.
  ///
  /// @param tgv The @ref paragraph_view.
  /// @param mv A view over the input matrix.
  /// @param vv A view over the input vector.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV, typename Matrix, typename Vector>
  void matvec_no_replication(TGV const& tgv, Matrix& mv, Vector& vv)
  {
    stapl_assert(0, "don't expect to get here");

    const std::size_t nprows     =  mv.m_ct.nprows();
    const std::size_t npcols     =  mv.m_ct.npcols();

    stapl_assert(nprows == npcols,
      "Double sanity check of nprows==npcols in nest_map_mr_factory failed");

    const std::size_t my_prow    =  mv.m_ct.my_prow();
    const std::size_t my_pcol    =  mv.m_ct.my_pcol();

    const size_t n_locs          = vv.get_num_subviews();
    const std::size_t num_succs  = 1;

    //
    // (1) Identity add_task to allow consumption of col subvector piece by
    //     allows locations in col
    //
    const std::size_t phase_one_tid = my_pcol;
////
////    if (my_prow == 0)
////    {
////      // std::cout << get_location_id() << ": Phase 1 - " << phase_one_tid
////                   << "\n";
////      //
////      tgv.add_task(phase_one_tid,
////                       coarse_map_wf_t(identity<double>(), n_locs),
////                       nprows,
////                       std::make_pair(&vv, get_location_id()));
////    }

    //
    // (2) Local Matmult.
    //
    const std::size_t phase_two_tid = npcols + get_location_id();

    // std::cout << get_location_id() << ": Phase 2 - << " << phase_two_tid
    //           << " consuming " << phase_one_tid << "\n";

    tgv.add_task(phase_two_tid,
                 block_matvec_wf(),
                 std::vector<std::size_t>(),
                 1,
                 std::make_pair(&mv, 0),
                 consume<sv_t>(tgv, phase_one_tid));

    //
    // (3) Reduction across location row tasks
    //
    const std::size_t phase_three_tid   =
      npcols + get_num_locations() + get_location_id();

    const std::size_t phase_3_num_succs =
      vv.get_num_local_subviews() == 0 ? 2 : 2;

    if (my_pcol == 0)
    {
      // std::cout << get_location_id() << ": Phase 3 -
      //           << " << phase_three_tid << " consuming "
      //           << phase_two_tid << "\n";

      tgv.add_task(phase_three_tid,
                   coarse_map_wf_t(identity<double>()),
                   std::vector<std::size_t>(),
                   phase_3_num_succs,
                   consume<sv_t>(tgv, phase_two_tid));
    }
    else
    {
      const std::size_t left_neighbor = phase_three_tid - 1;

      // std::cout << get_location_id() << ": Phase 3 - << " << phase_three_tid
      //           << " consuming " << phase_two_tid << " and "
      //           << left_neighbor << "\n";

      tgv.add_task(phase_three_tid,
                   coarse_map_plus_wf_t(plus<double>()),
                   std::vector<std::size_t>(),
                   phase_3_num_succs,
                   consume<sv_t>(tgv, phase_two_tid),
                   consume<sv_t>(tgv, left_neighbor));
    }

    //
    // (4) Transpose value back add_task..
    //
    const std::size_t phase_four_pred_tid =
      npcols + get_num_locations() + my_pcol * npcols + npcols - 1;

    const std::size_t result_tid =
      npcols + 2 * get_num_locations() + get_location_id();

    if (vv.get_num_local_subviews() == 0)
    {
      // NOTE - we make this task depend on phase 3, so that we safely say local
      // computation is done when the result_tid task is done.  That way we can
      // use result based TD and avoid the general terminator algorithm.
      //
      tgv.add_task(result_tid, simple_wf(), num_succs,
                   localize_object(sv_t(0, n_locs)),
                   consume<sv_t>(tgv, phase_three_tid));


      //std::cout << get_location_id() << ": Phase 4 - << "
      //          << result_tid << " consuming " << phase_four_pred_tid << "\n";
    }
    else
    {
      stapl_assert(my_prow == 0, "Error in nested_map_mr_factory");

      //std::cout << get_location_id() << ": Phase 4 - << result_tid consuming "
      //          << phase_four_pred_tid << "\n";

      // NOTE - setting the result_tid as commented out below is sufficient.
      // The only reason we create a local task is for the dependence on the
      // phase 3 task so that we can guaranteed all local tasks have finished
      // when result fires, so that we can use result based TD.  A better TD
      // would relax this unnecssarily strict ordering.
      //
      // result_tid = phase_four_pred_tid;
      //
      tgv.add_task(result_tid,
                   coarse_map_wf_t(identity<double>()),
                   std::vector<std::size_t>(1, phase_three_tid),
                   num_succs,
                   consume<sv_t>(tgv, phase_four_pred_tid));
    }

    // std::cout << get_location_id() << ": Result Tid is "
    //           << result_tid << "\n";

    tgv.set_result(result_tid);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Factory implementation called by function operator if computation
  /// replication is enabled.
  ///
  /// @param tgv The @ref paragraph_view.
  /// @param mv A view over the input matrix.
  /// @param vv A view over the input vector.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV, typename Matrix, typename Vector>
  void matvec_replication(TGV const& tgv, Matrix& mv, Vector& vv)
  {
    const std::size_t nprows  = mv.m_ct.nprows();
    const std::size_t npcols  = mv.m_ct.npcols();

    const std::size_t my_prow = mv.m_ct.my_prow();
    const std::size_t my_pcol = mv.m_ct.my_pcol();

    const size_t n_locs       = get_num_locations();
    const size_t my_loc       = get_location_id();

    stapl_assert(nprows == npcols,
      "Double sanity check of nprows==npcols in nest_map_mr_factory failed");

    // (1) Local Matmult.
    //
    const std::size_t phase_one_tid = my_loc;

    tgv.add_task(phase_one_tid,
                 block_matvec_wf(),
                 std::vector<std::size_t>(),
                 2,
                 std::make_pair(&mv, 0),
                 std::make_pair(&vv, vv.get_local_component(0).get_id()));

    // (2) Reduction across location rows.
    //
    size_t l2_npcols = 0;

    for (size_t i = npcols; i > 1; i /= 2)
      ++l2_npcols;

    plus<double> plus_wf;

    coarse_map_plus_wf_t phase2_wf(plus_wf);

    size_t phase2_base_tid  = n_locs;
    size_t div_factor       = 2;

    // size_t level = 0;

    for (; div_factor <= npcols; phase2_base_tid += n_locs)
    {
      size_t new_div_factor = div_factor * 2;

      size_t j = (my_pcol + div_factor/2) % div_factor
        + my_pcol / div_factor * div_factor;

      const size_t exchange_location = my_prow * npcols + j;

      const size_t my_tid     = phase2_base_tid + my_loc;
      //const size_t n_succs    = div_factor == npcols ? 1 : 2;
      const size_t n_succs    = div_factor == npcols ? 2 : 2;
      const size_t left_pred  = my_tid - n_locs;
      const size_t right_pred = phase2_base_tid + exchange_location - n_locs;

      // ++level;

      tgv.add_task(my_tid, phase2_wf, std::vector<std::size_t>(), n_succs,
                   consume<sv_t>(tgv, left_pred),
                   consume<sv_t>(tgv, right_pred));

      div_factor = new_div_factor;
    }

    // (3) Transpose value back add_task..
    //
    size_t phase3_base_tid   = n_locs * (l2_npcols);
    size_t exchange_location = my_loc % nprows * nprows + my_loc / nprows;
    size_t predecessor_tid   = phase3_base_tid + exchange_location;

    tgv.set_result(predecessor_tid);
  }

public:
  typedef map_view<sv_t>                                   result_type;
  typedef nest_map_mr_task_mapper                          task_id_mapper_type;


  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the custom task to location mapper used to
  /// specialize the key distribution of the PARAGRAPH's @ref directory.
  ///
  /// @return A key mapper which knows how tasks for this PARAGRAPH will be
  /// distributed among locations.
  ///
  /// @param mv A view over the input matrix.
  /// @param vv A view over the input vector.
  //////////////////////////////////////////////////////////////////////
  template<typename Matrix, typename Vector>
  task_id_mapper_type
  get_task_id_mapper(Matrix& mv, Vector& vv) const
  {
    return task_id_mapper_type(mv.m_ct.npcols(), true);
//    return task_id_mapper_type(mv.m_ct.npcols(), vv.get_num_copies() > 1);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator a factory called to populate the PARAGRAPH
  /// with tasks.
  ///
  /// @param tgv The @ref paragraph_view.
  /// @param mv A view over the input matrix.
  /// @param vv A view over the input vector.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV, typename Matrix, typename Vector>
  void operator()(TGV const& tgv, Matrix& mv, Vector& vv)
  {
//    if (vv.get_num_copies() > 1)
      matvec_replication(tgv, mv, vv);
//    else
//      matvec_no_replication(tgv, mv, vv);
  }

}; // struct nest_map_mr_factory

} // namespace composition


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Custom PARAGRAPH termination detection which signals termination
/// after a predefined number of invocations of the @p receive_notify method.
///
/// Used by @ref composition::nest_map_mr_factory to tie termination to the
/// completion of a predefined set of PARAGRAPH task completions.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
class counter_based_terminator
  : public terminator_base
{
private:
  /// @brief The number of times @p receive_notify should be called before
  /// signaling PARAGRAPH termination.
  size_type m_count;

public:
  explicit counter_based_terminator(size_type count = 2)
    : m_count(count)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starting function required by executor for terminators.
  /// Noop for this terminator.
  ///
  /// @return @c true if @ref receive_notify() has been called the required
  ///         number of times, otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  bool operator()(void)
  { return (m_count==0); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes base class method @ref terminator_base::call_notifier()
  ///  after the defined number of invocations.
  //////////////////////////////////////////////////////////////////////
  void receive_notify(void)
  {
    //stapl_assert(!m_executor_notifier.empty(),
    //  "result_based_terminator::receive_notify(): executor notifier not set");

    if (--m_count == 0)
      call_notifier();
  }

  size_type iterations(void) const noexcept override
  { return 0; }
}; // struct counter_based_terminator


//////////////////////////////////////////////////////////////////////
/// @brief Notifier used by custom termination detection for
/// @ref composition::nest_map_mr_factory.
///
/// Invokes @p receive_notify member on pointer to forward notification to
/// to underlying terminator.  Used instead of bind / function idiom to
/// avoid heap allocations.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
class counter_receive_notify_callback
{
private:
  /// @brief Pointer to underlying terminator to forward events to.
  counter_based_terminator* m_terminator_ptr;

public:
  counter_receive_notify_callback(counter_based_terminator* terminator_ptr)
    : m_terminator_ptr(terminator_ptr)
  { }

  void operator()(void) const
  {
    m_terminator_ptr->receive_notify();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Partial specialization to enable custom termination detection
/// for @ref composition::nest_map_mr_factory.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<>
struct terminator_initializer<composition::nest_map_mr_factory>
{
  typedef counter_based_terminator terminator_t;

  template<typename Result, typename SVS>
  terminator_t*
  operator()(Result const& result, task_graph& tg, SVS& svs) const
  {
    terminator_t* t_ptr = new terminator_t();

    size_t l2_npcols = 0;

    const size_t npcols = get<0>(svs).m_ct.npcols();

    for (size_t i = npcols; i > 1; i /= 2)
      ++l2_npcols;

    const size_t n_locs            = get_num_locations();
    const size_t my_loc            = get_location_id();
    const size_t final_reduce_tid  = l2_npcols * n_locs + my_loc;

    stapl_assert(!find_accessor<Result>()(result).available(),
      "terminator_initializer found available result");

    find_accessor<Result>()(result).request_notify(
      counter_receive_notify_callback(t_ptr)
    );

    typedef composition::nest_map_mr_factory::sv_t edge_t;

    // std::cout << get_location_id() << ": setting terminator to result and "
    //           << final_reduce_tid << "\n";

    tg.request_notify<edge_t>(
      final_reduce_tid,
      counter_receive_notify_callback(t_ptr)
    );

    return t_ptr;
  }
}; // struct terminator_initializer

} // namespace detail


namespace composition {

namespace result_of {

///////////////////////////////////////////////////////////////////////
/// @brief Reflect return type of map_func specialization when
/// workfunction is @ref prototype::functional::inner_product.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename V0, typename V1>
struct map_func<prototype::functional::inner_product, V0, V1>
  : public boost::result_of<
      paragraph<
        nest_map_mr_factory,
        typename V0::base_view_type,
        typename V1::base_view_type
      >()
    >
{ };

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Signature of function template @ref map_func that matches
/// cases when the workfunction is @ref prototype::functional::inner_product.
///
/// Specializes the implementation for this case, creating a PARAGRAPH using
/// the @ref nest_map_mr_factory factory.
///
/// @param wf An instance of the @ref prototype::functional::inner_product
///           with initialized elementary operations.
/// @param v0 A one dimensional view input.
/// @param v1 A one dimensional view input.
///
/// @return A view of the results of the spawned PARAGRAPH computation.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename V0, typename V1>
inline
typename result_of::map_func<
  prototype::functional::inner_product, V0, V1
>::type
map_func(prototype::functional::inner_product const& wf,
         V0 const& v0,
         V1 const& v1 //,
         /*unsigned int num_succs = 0*/)
{
  typedef typename V0::base_view_type v0_t;
  typedef typename V1::base_view_type v1_t;

  typedef paragraph<nest_map_mr_factory, v0_t, v1_t> tg_t;

  return (*new tg_t(nest_map_mr_factory(), v0_t(v0.m_ct), v1.m_view))((int) 0);

//  typedef ip_transform_coarse_wf coarse_wf_t;
//
//  return composition::map_func(is_coarse_wf(coarse_wf_t()), v0, v1);
//
  //typedef composition::m_mr_transform_wf coarse_wf_t;
//
  // v1 is repeated.  However, we're going to work with coarse for now,
  // so don't repeat it since m_mr_transform_wf will deal with it.
  //
  //return composition::map_func(
  //  is_coarse_wf(coarse_wf_t()), transpose(v0), v1.m_view);
}


// If the functor is the result of a bind2nd invocation, unwind it, passing
// the underlying functor as the wf and transforming m_value (fixed view)
// to a repeated, top level argument.  Normalizes structure for future
// transforms and make view visible to task placement policy.
//
namespace result_of {

///////////////////////////////////////////////////////////////////////
/// @brief Reflect return type of map_func specialization when unary
/// workfunction is an instantiation of the @ref binder2nd class template.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename Stored, typename V0>
struct map_func<binder2nd<Operation, Stored, false>, V0>
  : public map_func<Operation, V0, repeat_view<Stored> >
{ };


///////////////////////////////////////////////////////////////////////
/// @brief Reflect return type of map_func specialization when unary
/// workfunction is an instantiation of the @ref binder1st class template.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename Stored, typename V0>
struct map_func<binder1st<Operation, Stored, true>, V0>
  : public map_func<Operation, V0, repeat_view<Stored> >
{ };

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Converts @ref map_func call with @ref stapl::binder2nd as
/// the workfunction to one with the base workfunction of type
/// @p Operation and a @ref repeat_view in place of the bound view
/// of type @p Stored.
///
/// A replicated view and a view bound to a workfunction with a bind operation
/// are equivalent.  Normalize to a common form that makes the PARAGRAPH aware
/// of all views.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename Stored, typename V0>
inline
typename result_of::map_func<binder2nd<Operation, Stored, false>, V0>::type
map_func(binder2nd<Operation, Stored, false> const& wf, V0 const& v0)
{
  return composition::map_func(wf.op(), v0, repeat_view<Stored>(wf.m_value));
}


//////////////////////////////////////////////////////////////////////
/// @brief Converts @ref map_func call with @ref stapl::binder1st as
/// the workfunction to one with the base workfunction of type
/// @p Operation and a @ref repeat_view in place of the bound view
/// of type @p Stored.
///
/// A replicated view and a view bound to a workfunction with a bind operation
/// are equivalent.  Normalize to a common form that makes the PARAGRAPH aware
/// of all views.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename Stored, typename V0>
inline
typename result_of::map_func<binder1st<Operation, Stored, true>, V0>::type
map_func(binder1st<Operation, Stored, true> const& wf, V0 const& v0)
{
  return composition::map_func(wf.op(), v0, repeat_view<Stored>(wf.m_value));
}

} // namespace composition

} // namespace stapl

#endif // ifndef STAPL_SKELETONS_EXPLICIT_NEW_MAP_TRANSFORMATIONS_HPP
