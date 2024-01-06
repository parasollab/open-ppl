#include <stapl/algorithms/algorithm.hpp>

namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function which sets the second argument if the given predicate
///   returns true for the given value.
/// @tparam BinaryPredicate Binary functor which is called on the input element.
//////////////////////////////////////////////////////////////////////
template <typename BinaryPredicate>
struct partition_count_matches_b
{
  BinaryPredicate m_predicate;

  typedef void result_type;

  partition_count_matches_b(BinaryPredicate const& predicate)
    : m_predicate(predicate)
  {}

  template <typename ValueRef1, typename ValueRef2, typename CountRef>
  void operator()(ValueRef1 value1, ValueRef2 value2, CountRef count)
  {
    bool test = m_predicate(value1,value2);
    count.set(test);
  }

  void define_type(typer& t)
  {
    t.member(m_predicate);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function for @ref stable_partition() which copies the input
///   element to the appropriate position in the output view.
/// @tparam BinaryPredicate Binary functor which is used to partition the input.
//////////////////////////////////////////////////////////////////////
template <typename BinaryPredicate>
struct partition_fill_temp_b
{
  BinaryPredicate m_predicate;
  unsigned int   m_offset;

  typedef void result_type;

  partition_fill_temp_b(BinaryPredicate const& predicate, unsigned int offset)
    : m_predicate(predicate), m_offset(offset)
  {}

  template <typename ValueRef1, typename ValueRef2, typename CountRef,
            typename TempView>
  void operator()(ValueRef1 value1, ValueRef2 value2, CountRef count,
                  TempView temp_view)
  {
    bool test = m_predicate(value1,value2);
    temp_view.set_element(count.value(test, m_offset), value1);
  }

  template <typename ValueRef1, typename ValueRef2, typename ValueRef3,
            typename CountRef, typename TempView>
  void operator()(ValueRef1 value1, ValueRef2 value2, ValueRef3 value3,
                  CountRef count, TempView temp_view)
  {
    bool test = m_predicate(value1,value2);
    temp_view.set_element(count.value(test, m_offset), value3);
  }

  void define_type(typer& t)
  {
    t.member(m_predicate);
    t.member(m_offset);
  }
};

} // namespace algo_details

//////////////////////////////////////////////////////////////////////
/// @brief Partition the input such that all elements for which the predicate
///   returns true are ordered before those for which it returned false, while
///   also maintaining the relative ordering of the elements.
/// @param pview One-dimensional view of the input.
/// @param predicate Binary functor used to partition the input.
/// @return The number of elements for which the predicate returned true.
/// @ingroup reorderingAlgorithms
///
/// This algorithm is mutating.
/// @note Replacing static_array with array will eliminate possible
///       synchronous communications generated when the data is not
///       aligned.
/////////////////////////////////////////2/////////////////////////////
template <typename View1, typename View2, typename Pred>
size_t stable_partition_b(View1 const& pview1, View2 const& pview2,
                          Pred predicate)
{
  typedef static_array<algo_details::partition_pred_counter>  cnt_container_t;
  typedef array_view<cnt_container_t>                         cnt_view_t;

  typedef static_array<typename View1::value_type>  tmp_container_t;
  typedef array_view<tmp_container_t>              tmp_view_t;

  unsigned int num_elements = pview1.size();
  cnt_view_t count_view(new cnt_container_t(num_elements));
  tmp_view_t temp_view(new tmp_container_t(num_elements));

  map_func(algo_details::partition_count_matches_b<Pred>(predicate),
           pview1, pview2, count_view);

  algo_details::partition_pred_counter num_satisfy_elements =
    stapl::accumulate(count_view, algo_details::partition_pred_counter());
  stapl::partial_sum(count_view, count_view, false);

  map_func(algo_details::partition_fill_temp_b<Pred>(
           predicate, num_satisfy_elements.m_is_match),
           pview1, pview2, count_view, make_repeat_view(temp_view));

  stapl::copy(temp_view, pview1);
  return num_satisfy_elements.m_is_match;
}

//////////////////////////////////////////////////////////////////////
/// @brief Partition the input such that all elements for which the predicate
///   returns true are ordered before those for which it returned false, while
///   also maintaining the relative ordering of the elements.
/// @param pview One-dimensional view of the input.
/// @param predicate Binary functor used to partition the input.
/// @return The number of elements for which the predicate returned true.
/// @ingroup reorderingAlgorithms
///
/// This algorithm is mutating.
/// @note Replacing static_array with array will eliminate possible
///       synchronous communications generated when the data is not
///       aligned.
/////////////////////////////////////////2/////////////////////////////
template <typename View1, typename View2, typename View3, typename Pred>
size_t stable_partition_b(View1 const& pview1, View2 const& pview2, 
                          View3 const& pview3, Pred predicate)
{
  typedef static_array<algo_details::partition_pred_counter>  cnt_container_t;
  typedef array_view<cnt_container_t>                         cnt_view_t;

  typedef static_array<typename View1::value_type>  tmp_container_t;
  typedef array_view<tmp_container_t>              tmp_view_t;

  unsigned int num_elements = pview1.size();
  cnt_view_t count_view(new cnt_container_t(num_elements));
  tmp_view_t temp_view(new tmp_container_t(num_elements));

  map_func(algo_details::partition_count_matches_b<Pred>(predicate),
           pview2, pview3, count_view);

#ifdef DEBUG
stapl::do_once([&]() {
  cerr << "stable_partition_b: pview2" << endl;
  for( int i=0; i<pview2.size(); i++ ) {
    cerr << pview2[i] << ",";
  }
  cerr << endl;
  cerr << "stable_partition_b: pview3" << endl;
  for( int i=0; i<pview3.size(); i++ ) {
    cerr << pview3[i] << ",";
  }
  cerr << endl;
});
#endif

  algo_details::partition_pred_counter num_satisfy_elements =
    stapl::accumulate(count_view, algo_details::partition_pred_counter());
  stapl::partial_sum(count_view, count_view, false);

  map_func(algo_details::partition_fill_temp_b<Pred>(
           predicate, num_satisfy_elements.m_is_match),
           pview2, pview3, pview1, count_view, make_repeat_view(temp_view));

#ifdef DEBUG
stapl::do_once([&]() {
  cerr << "stable_partition_b:" << endl;
  for( int i=0; i<temp_view.size(); i++ ) {
    cerr << temp_view[i] << ",";
  }
  cerr << endl;
});
#endif

  stapl::copy(temp_view, pview1);
  return num_satisfy_elements.m_is_match;
}


//////////////////////////////////////////////////////////////////////
/// @brief Partition the input such that all elements for which the predicate
///   returns true are ordered before those for which it returned false.
/// @param pview1 One-dimensional view of the input.
/// @param pview2 One-dimensional view of the input.
/// @param predicate Binary functor used to partition the input.
/// @ingroup reorderingAlgorithms
///
/// This algorithm is mutating, and currently calls @ref stable_partition().
//////////////////////////////////////////////////////////////////////
template <typename View1, typename View2, typename Pred>
size_t partition_b(View1 const& pview1, View2 const& pview2, Pred predicate)
{
  return stable_partition_b(pview1, pview2, predicate);
};

//////////////////////////////////////////////////////////////////////
/// @brief Partition the input such that all elements for which the predicate
///   returns true are ordered before those for which it returned false.
/// @param pview1 One-dimensional view of the input.
/// @param pview2 One-dimensional view of the input.
/// @param predicate Binary functor used to partition the input.
/// @ingroup reorderingAlgorithms
///
/// This algorithm is mutating, and currently calls @ref stable_partition().
//////////////////////////////////////////////////////////////////////
template <typename View1, typename View2, typename View3, typename Pred>
size_t partition_b(View1 const& pview1, View2 const& pview2, 
                   View3 const& pview3, Pred predicate)
{
  return stable_partition_b(pview1, pview2, pview3, predicate);
};
 
//////////////////////////////////////////////////////////////////////
/// @brief Replace the values from the first view for which the given
///   predicate returns true with the new value.
/// @param vw1 One-dimensional view of input 1
/// @param vw2 One-dimensional view of input 2
/// @param predicate Binary functor which returns true for replaced elements.
///   predicate is applied to two input views
/// @param new_value Value used to replace elements.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Predicate>
void replace_if(View1& vw1, View2& vw2, Predicate pred,
                typename View1::value_type const& new_value)
{
  stapl::map_func(algo_details::assign_if
                  <Predicate, typename View1::value_type>
                  (pred, new_value), vw1, vw2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Replace the values from the first view for which the given
///   predicate returns true with the new value.
/// @param vw1 One-dimensional view of input 1
/// @param vw2 One-dimensional view of input 2
/// @param vw3 One-dimensional view of input 3
/// @param predicate Binary functor which returns true for replaced elements.
///   predicate is applied to second and third input views
/// @param new_value Value used to replace elements.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename View3, typename Predicate>
void replace_if(View1& vw1, View2& vw2, View3& vw3, Predicate pred,
                typename View1::value_type const& new_value)
{
  stapl::map_func(algo_details::assign_if
                  <Predicate, typename View1::value_type>
                  (pred, new_value), vw1, vw2, vw3);
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the input view to the output, except for those
///   elements for which the given predicate returns true, which are replaced
///   with the given value.
/// @param vw0 One-dimensional view of the first input.
/// @param vw1 One-dimensional view of the second input.
/// @param vw2 One-dimensional view of the output.
/// @param pred Binary functor which returns true for replaced elements.
/// @param new_value Value used to replace elements for which the functor
///   returns true.
/// @return Iterator pointing to the end of the output view.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the output view. The input and output views must be
/// the same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename View2,
         typename Predicate>
typename View1::iterator
replace_copy_if(View0 const& vw0, View1 const& vw1, View2 const& vw2,
                Predicate pred, typename View0::value_type new_value)
{
  stapl::map_func(
      algo_details::copy_with_replacement
      <Predicate, typename View0::value_type>
      (pred,new_value), vw0, vw1, vw2);
  return vw2.end();
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the input view to the output, except for those
///   elements for which the given predicate returns true, which are replaced
///   with the given value.
/// @param vw0 One-dimensional view of the first input.
/// @param vw1 One-dimensional view of the second input.
/// @param vw2 One-dimensional view of the output.
/// @param pred Binary functor which returns true for replaced elements.
/// @param new_value Value used to replace elements for which the functor
///   returns true.
/// @return Iterator pointing to the end of the output view.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the output view. The input and output views must be
/// the same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename View2, typename View3,
         typename Predicate>
typename View1::iterator
replace_copy_if(View0 const& vw0, View1 const& vw1, View2 const& vw2,
                View3 const& vw3, Predicate pred,
                typename View0::value_type new_value)
{
  stapl::map_func(
      algo_details::copy_with_replacement
      <Predicate, typename View0::value_type>
      (pred,new_value), vw0, vw1, vw2, vw3);
  return vw3.end();
}

//////////////////////////////////////////////////////////////////////
/// @brief Remove the values from the first view for which the given
///   predicate returns false.
/// @param view1 One-dimensional view of input 1
/// @param view2 One-dimensional view of input 2
/// @param predicate Binary functor which returns false for removed elements.
///   predicate is applied to the two input views
/// @return View over the copied range, which is the same as or smaller than the
///   provided input view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Pred>
View1 collect_if(View1 const& view1, View2 const& view2, Pred predicate)
{
  typedef typename View1::domain_type dom_t;
  size_t truesize=partition_b(view1,view2,predicate);

  if (truesize!=0) {
    dom_t new_dom(view1.domain().first(),
                  view1.domain().advance(view1.domain().first(),truesize - 1),
                  view1.domain());
    return View1(view1.container(), new_dom);
  }
  else
    return View1(view1.container(), dom_t());
}

//////////////////////////////////////////////////////////////////////
/// @brief Remove the values from the first view for which the given
///   predicate returns false.
/// @param view1 One-dimensional view of input 1
/// @param view2 One-dimensional view of input 2
/// @param view2 One-dimensional view of input 3
/// @param predicate Binary functor which returns false for removed elements.
///   predicate is applied to the second and third input views
/// @return View over the copied range, which is the same as or smaller
///   than the provided input view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename View3, typename Pred>
View1 collect_if(View1 const& view1, View2 const& view2, View3 const& view3,
             Pred predicate)
{
  typedef typename View1::domain_type dom_t;
  size_t truesize=partition_b(view1,view2,view3,predicate);

  if (truesize!=0) {
    dom_t new_dom(view1.domain().first(),
                  view1.domain().advance(view1.domain().first(),truesize - 1),
                  view1.domain());
    return View1(view1.container(), new_dom);
  } else {
    return View1(view1.container(), dom_t());
  }
}

#if 0
// these don't work because of thenegatepred()

 //////////////////////////////////////////////////////////////////////
/// @brief Remove the values from the input view for which the given
///   predicate returns true.
/// @param view1 One-dimensional view of input 1.
/// @param view2 One-dimensional view of input 2.
/// @param predicate Binary functor which returns true for removed elements.
///   Predicate is applied to the input views.
/// @return View over the copied range, which is the same as or smaller
///   than the provided input view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Pred>
View1 remove_if(View1 const& view1, View2 const& view2, Pred predicate)
{
  typedef typename View1::domain_type dom_t;
  stapl::unary_negate<Pred> thenegatepred(predicate);
  size_t truesize=partition_b(view1,view2,thenegatepred);

  if (truesize!=0) {
    dom_t new_dom(view1.domain().first(),
                  view1.domain().advance(view1.domain().first(),truesize - 1),
                  view1.domain());
    return View1(view1.container(), new_dom);
  }
  else
     return View1(view1.container(), dom_t());
}

//////////////////////////////////////////////////////////////////////
/// @brief Remove the values from the input view for which the given
///   predicate returns true.
/// @param view1 One-dimensional view of input 1.
/// @param view2 One-dimensional view of input 2.
/// @param view3 One-dimensional view of input 3.
/// @param predicate Binary functor which returns true for removed elements.
///   Predicate is applied to the second and third input views.
/// @return View over the copied range, which is the same as or smaller
///   than the provided input view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename View3, typename Pred>
View1 remove_if(View1 const& view1, View2 const& view2, View3 const& view3,
                Pred predicate)
{
  typedef typename View1::domain_type dom_t;
  stapl::unary_negate<Pred> thenegatepred(predicate);
  size_t truesize=partition_b(view1,view2,view3,thenegatepred);

  if (truesize!=0) {
    dom_t new_dom(view1.domain().first(),
                  view1.domain().advance(view1.domain().first(),truesize - 1),
                  view1.domain());
    return View1(view1.container(), new_dom);
  }
  else
     return View1(view1.container(), dom_t());
}
#endif

//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the first input view to the output,
///   except for those elements for which the given predicate returns true.
/// @param vw0 One-dimensional view of first input.
/// @param vw1 One-dimensional view of second input.
/// @param vw2 One-dimensional view of the output.
/// @param predicate Binary functor which returns true for removed elements.
///   Predicate is applied to the two input views.
/// @return View over the copied range, which is the same as or smaller
///   than the provided output view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the output view. The input and output views
/// must be the same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename View2, typename Pred>
View1 remove_copy_if(View0 const& vw0, View1 & vw1, View2 & vw2,
                     Pred predicate)
{
  copy(vw0, vw2);
  return remove_if(vw1,vw2, predicate);
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the first input view to the output,
///   except for those elements for which the given predicate returns true.
/// @param vw0 One-dimensional view of first input.
/// @param vw1 One-dimensional view of second input.
/// @param vw2 One-dimensional view of third input.
/// @param vw3 One-dimensional view of the output.
/// @param predicate Binary functor which returns true for removed elements.
///   Predicate is applied to the second and third input views.
/// @return View over the copied range, which is the same as or smaller
///   than the provided output view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the output view. The input and output views
/// must bethe same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename View2, typename View3,
         typename Pred>
View1 remove_copy_if(View0 const& vw0, View1 & vw1, View2 & vw2, View3 & vw3,
                     Pred predicate)
{
  copy(vw0, vw3);
  return remove_if(vw3,vw1,vw2, predicate);
}

//////////////////////////////////////////////////////////////////////
/// @param vw0 One-dimensional view of the first input.
/// @param vw1 One-dimensional view of the second input.
/// @param vw2 One-dimensional view of the output.
/// @param predicate Binary functor which returns true for copied elements.
///   predicate is applied to the two input views
/// @return View over the copied range, which is the same as or smaller
///   than the provided output view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the output view. The input and output views
/// must be the same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename View2, typename Pred>
View1 copy_if(View0 const& vw0, View1 & vw1, View2 & vw2, Pred predicate)
{
  copy(vw0, vw2);
  return collect_if(vw2,vw1, predicate);
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the first input view to the output those
///   elements for which the given predicate returns true.
/// @param vw0 One-dimensional view of the first input.
/// @param vw1 One-dimensional view of the second input.
/// @param vw2 One-dimensional view of the third input.
/// @param vw3 One-dimensional view of the output.
/// @param predicate Binary functor which returns true for copied elements.
///   predicate is applied to the second and third input views
/// @return View over the copied range, which is the same as or smaller
///   than the provided output view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the output view. The input and output views
/// must be the same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename View2, typename View3,
         typename Pred>
View1 copy_if(View0 const& vw0, View1 & vw1, View2 & vw2, View3 & vw3,
              Pred predicate)
{
  copy(vw0, vw3);
  return collect_if(vw3,vw1,vw2, predicate);
}

}
