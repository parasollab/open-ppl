#include <stapl/runtime.hpp>

template<typename Predicate>
class Find_map
{
private:
  Predicate m_pred;

public:
  Find_map(Predicate const& pred)
    : m_pred(pred)
  { }

  template<typename Ref>
  Ref operator()(Ref elem) const
  {
    if (m_pred(elem)) {
      return elem;
    } else {
      return Ref(stapl::null_reference());
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_pred);
  }
};

class Find_reduce
{
public:
  template<typename Ref1, typename Ref2>
  Ref1 operator()(Ref1 lhs, Ref2 rhs) const
  {
    if (!is_null_reference(lhs)) {
      return lhs;
    } else {
      return rhs;
    }
  }
};

template<typename View, typename Predicate>
typename View::reference
Find_if (View const& view, Predicate const& pred)
{
  auto x= stapl::map_reduce(Find_map<Predicate>(pred), Find_reduce(), view);
  return x;
}

template<typename View, typename T>
typename View::reference
Find(View const& view, T const& value)
{
  auto x= Find_if (view, stapl::bind2nd(stapl::equal_to<T>(), value));
  return x;
}

///////////////////////////////////////////////////////////////////////////////

#if 0
struct match_cont_wf {
  template<typename View1, typename View2>
  result_type operator()(View1 left, View2 right)
  {
#if 0
    auto neq_values = stapl::mismatch(left,right);

    if (stapl::is_null_reference(neq_values.first) ) {
      return left;
    } else {
      return View1::reference(stapl::null_reference());
    }
#endif
  }
  typedef View1::reference result_type;
};
#endif

template<typename View1, typename View2>
typename View1::reference
match_cont(View1 const& view1, View2 const& view2)
{
//stapl::map_reduce( match_cont_wf(), Find_reduce(), view1, view1);
//stapl::make_repeat_view(view2) );
}

