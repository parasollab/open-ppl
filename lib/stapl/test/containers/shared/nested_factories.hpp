/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_CONTAINERS_NESTED_FACTORIES_HPP
#define STAPL_TEST_CONTAINERS_NESTED_FACTORIES_HPP

#include <stapl/skeletons/explicit/map_prototype.hpp>


template<typename View>
struct fast_view_checker
  : public std::false_type
{ };


template<typename BaseView>
struct fast_view_checker<stapl::localized_view<BaseView>>
  : public std::true_type
{ };


template<typename T, typename BinaryOp>
class combiner
{
private:
  BinaryOp m_binary_op;

public:
  typedef T result_type;

  combiner(BinaryOp const& binary_op)
    : m_binary_op(binary_op)
  { }

  template<typename Refs>
  result_type operator()(Refs elems) const
  {
    result_type result = elems[0];

    for (std::size_t i = 1; i < elems.size(); ++i)
      result = m_binary_op(result, elems[i]);

    return result;
  }

  void define_type(stapl::typer& t)
  { t.member(m_binary_op); }
};


template<typename T, typename BinaryOp>
class coarse_combiner
{
private:
  BinaryOp m_binary_op;

public:
  typedef T result_type;

  coarse_combiner(BinaryOp const& binary_op)
    : m_binary_op(binary_op)
  { }

  template<typename View>
  result_type operator()(View v) const
  {
    stapl_assert(fast_view_checker<View>::value,
      "coarse_combiner working on non-localized views");

    result_type result = *v.begin();
    auto iter          = ++v.begin();
    auto e_iter        = v.end();

    for (; iter != e_iter; ++iter)
      result = std::min<int>(result, *iter);

    return result;
  }

  void define_type(stapl::typer& t)
  { t.member(m_binary_op); }
};


template<typename T, typename MapOp, typename BinaryOp>
class coarse_mr_combiner
{
private:
  MapOp    m_map_op;
  BinaryOp m_binary_op;

public:
  typedef T result_type;

  coarse_mr_combiner(MapOp const& map_op, BinaryOp const& binary_op)
    : m_map_op(map_op), m_binary_op(binary_op)
  { }

  template<typename View>
  result_type operator()(View v) const
  {
    stapl_assert(fast_view_checker<View>::value,
      "coarse_mr_combiner working on non-localized views");

    result_type result = m_map_op(*v.begin());
    auto iter          = ++v.begin();
    auto e_iter        = v.end();

    for (; iter != e_iter; ++iter)
      result = m_binary_op(result, m_map_op(*iter));

    printf("coarse_mr_combiner returning %d\n",result);
    return result;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_map_op);
    t.member(m_binary_op);
  }
};


template<typename PGV, typename BinaryOp>
void
populate_flat_reduction_tree(PGV const& pgv,
                             BinaryOp const& binop,
                             std::vector<std::size_t> const& local_task_ids,
                             unsigned int num_elements)
{
  const stapl::location_type loc_id = pgv.graph().get_location_id();
  const stapl::location_type n_locs = pgv.graph().get_num_locations();

  // (1) Collect Local Results
  const unsigned int local_collector_tid = num_elements + loc_id;

  pgv.add_task(local_collector_tid, binop, 1,
               stapl::consume<int>(pgv, local_task_ids));

  // (2) Collect Global Results
  const unsigned int global_collector_tid = num_elements + n_locs;

  if (loc_id == 0)
  {
    std:: vector<std::size_t> global_task_ids;

    for (unsigned int i = num_elements; i < num_elements + n_locs; ++i)
      global_task_ids.push_back(i);

    pgv.add_task(global_collector_tid, binop, n_locs,
                 stapl::consume<int>(pgv, global_task_ids));
  }

  // (3) Broadcast Out
  pgv.set_result(global_collector_tid);
}


template<typename BinaryOp>
class leaf_reduce_factory
  : public stapl::factory_wf
{
private:
  BinaryOp m_binary_op;

public:
  using result_type = int;
  using coarsener_type = stapl::default_coarsener;

  leaf_reduce_factory(BinaryOp const& binary_op)
    : m_binary_op(binary_op)
  { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  template<typename PGV, typename View>
  void operator()(PGV const& pgv, View& v) const
  {
    auto iter = std::get<0>(partition_id_set(v));

    std::vector<std::size_t> local_task_ids;

    // Spawn a coarsened, sequential reduction workfunction coarsened subview
    for (; !iter.at_end(); ++iter)
    {
      local_task_ids.push_back(*iter);
      pgv.add_task(*iter, coarse_combiner<result_type, BinaryOp>(m_binary_op),
                   1, std::make_pair(&v, *iter));
    }

    populate_flat_reduction_tree(
      pgv, combiner<result_type, BinaryOp>(m_binary_op),
      local_task_ids, v.size());
  }

  void define_type(stapl::typer& t)
  { t.member(m_binary_op); }
};


template<typename BinaryOp>
class nested_reduce_factory
  : public stapl::factory_wf
{
private:
  unsigned int m_num_elements;
  BinaryOp     m_binary_op;

public:
  using result_type = int;
  using coarsener_type = stapl::default_coarsener;

  nested_reduce_factory(unsigned int num_elements,
                        BinaryOp const& binary_op)
    : m_num_elements(num_elements),
      m_binary_op(binary_op)
  { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  template<typename PGV, typename View>
  void operator()(PGV const& pgv, View& v) const
  {
    auto iter = std::get<0>(partition_id_set(v));

    std::vector<std::size_t> local_task_ids;

    // Spawn a reduction PARAGRAPH for each element
    stapl::gang g;

    for (; !iter.at_end(); ++iter)
    {
      auto subview = v.get_subview(*iter);

      unsigned int task_id = subview.domain().first();

      for (auto elem : subview)
      {
        local_task_ids.push_back(task_id);

        pgv.add_task(task_id, leaf_reduce_factory<BinaryOp>(m_binary_op),
                     1, stapl::localize_ref(elem));
        ++task_id;
      }
    }

    g.leave();

    populate_flat_reduction_tree(pgv, combiner<result_type,
                                 BinaryOp>(m_binary_op),
                                 local_task_ids, m_num_elements);
  }
};


template<typename View, typename BinaryOp>
typename View::value_type::value_type
nested_reduce(BinaryOp op, View& vw)
{
  typedef nested_reduce_factory<BinaryOp>                             factory_t;
  typedef stapl::paragraph<stapl::default_scheduler, factory_t, View>
                                                                    paragraph_t;

  return paragraph_t(factory_t(vw.size(), op), vw)();
}


template<typename MapOp, typename BinaryOp>
class leaf_map_reduce_factory
  : public stapl::factory_wf
{
private:
  MapOp    m_map_op;
  BinaryOp m_binary_op;

public:
  using result_type = int;
  using coarsener_type = stapl::default_coarsener;

  leaf_map_reduce_factory(MapOp const& map_op, BinaryOp const& binary_op)
    : m_map_op(map_op), m_binary_op(binary_op)
  { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  template<typename PGV, typename View>
  void operator()(PGV const& pgv, View& v) const
  {
    auto iter = std::get<0>(partition_id_set(v));

    std::vector<std::size_t> local_task_ids;

    // Spawn a coarsened, sequential map-reduce workfunction for each
    // coarsened subview
    for (; !iter.at_end(); ++iter)
    {
      local_task_ids.push_back(*iter);
      pgv.add_task(*iter,
        coarse_mr_combiner<result_type, MapOp, BinaryOp>(m_map_op, m_binary_op),
        1, std::make_pair(&v, *iter));
    }

    populate_flat_reduction_tree(
      pgv, combiner<result_type, BinaryOp>(m_binary_op),
      local_task_ids, v.size());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_map_op);
    t.member(m_binary_op);
  }
};


template<typename MapOp, typename BinaryOp>
class nested_map_reduce_factory
  : public stapl::factory_wf
{
private:
  unsigned int m_num_elements;
  MapOp        m_map_op;
  BinaryOp     m_binary_op;

public:
  using result_type = int;
  using coarsener_type = stapl::default_coarsener;

  nested_map_reduce_factory(unsigned int num_elements,
                        MapOp const& mop,
                        BinaryOp const& binary_op)
    : m_num_elements(num_elements),
      m_map_op(mop),
      m_binary_op(binary_op)
  { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  template<typename PGV, typename View>
  void operator()(PGV const& pgv, View& v) const
  {
    auto iter = std::get<0>(partition_id_set(v));

    std::vector<std::size_t> local_task_ids;

    // Spawn a reduction PARAGRAPH for each element
    stapl::gang g;

    for (; !iter.at_end(); ++iter)
    {
      auto subview = v.get_subview(*iter);

      unsigned int task_id = subview.domain().first();

      for (auto elem : subview)
      {
        local_task_ids.push_back(task_id);

        pgv.add_task(task_id,
                     leaf_map_reduce_factory<MapOp, BinaryOp>(m_map_op,
                                                              m_binary_op),
                     1, stapl::localize_ref(elem));
        ++task_id;
      }
    }

    g.leave();

    populate_flat_reduction_tree(pgv, combiner<result_type,
                                 BinaryOp>(m_binary_op),
                                 local_task_ids, m_num_elements);
  }
};


template<typename View, typename MapOp, typename BinaryOp>
typename View::value_type::value_type::second_type
nested_map_reduce(MapOp mop, BinaryOp op, View& vw)
{
  typedef nested_map_reduce_factory<MapOp, BinaryOp>                  factory_t;
  typedef stapl::paragraph<stapl::default_scheduler, factory_t, View>
                                                                    paragraph_t;

  return paragraph_t(factory_t(vw.size(), mop, op), vw)();
}


template<typename Op>
class map_combiner
{
private:
  Op m_op;

public:
  typedef void result_type;

  map_combiner(Op const& op)
    : m_op(op)
  { }

  template<typename Refs>
  result_type operator()(Refs elems) const
  {
    for (std::size_t i = 0; i < elems.size(); ++i)
      m_op(elems[i]);
  }

  void define_type(stapl::typer& t)
  { t.member(m_op); }
};


template<typename Op>
class coarse_map_combiner
{
private:
  Op m_op;

public:
  typedef void result_type;

  coarse_map_combiner(Op const& op)
    : m_op(op)
  { }

  template<typename View>
  result_type operator()(View v) const
  {
    stapl_assert(fast_view_checker<View>::value,
      "coarse_map_combiner working on non-localized views");

    auto iter          = v.begin();
    auto e_iter        = v.end();

    for (; iter != e_iter; ++iter)
      m_op(*iter);
  }

  void define_type(stapl::typer& t)
  { t.member(m_op); }
};


template<typename Op>
class leaf_map_func_factory
  : public stapl::factory_wf
{
private:
  Op m_op;

public:
  using result_type = void;
  using coarsener_type = stapl::default_coarsener;

  leaf_map_func_factory(Op const& op)
    : m_op(op)
  { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  template<typename PGV, typename View>
  void operator()(PGV const& pgv, View& v) const
  {
    auto iter = std::get<0>(partition_id_set(v));
    std::vector<std::size_t> local_task_ids;

    // Spawn a coarsened, sequential map workfunction for each coarsened subview
    for (; !iter.at_end(); ++iter)
    {
      local_task_ids.push_back(*iter);
      pgv.add_task(*iter, coarse_map_combiner<Op>(m_op),
                   0, std::make_pair(&v, *iter));
      std::ostringstream os;
    }
  }

  void define_type(stapl::typer& t)
  { t.member(m_op); }
};


template<typename Op>
class nested_map_func_factory
  : public stapl::factory_wf
{
private:
  unsigned int m_num_elements;
  Op     m_op;

public:
  using result_type = void;
  using coarsener_type = stapl::default_coarsener;

  nested_map_func_factory(unsigned int num_elements,
                          Op const& op)
    : m_num_elements(num_elements),
      m_op(op)
  { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  template<typename PGV, typename View>
  void operator()(PGV const& pgv, View& v) const
  {
    auto iter = std::get<0>(partition_id_set(v));

    std::vector<std::size_t> local_task_ids;

    // Spawn a map_func PARAGRAPH for each element
    stapl::gang g;

    for (; !iter.at_end(); ++iter)
    {
      auto subview = v.get_subview(*iter);

      unsigned int task_id = subview.domain().first();

      for (auto elem : subview)
      {
        local_task_ids.push_back(task_id);

        pgv.add_task(task_id, leaf_map_func_factory<Op>(m_op),
                     0, localize_ref(elem));
        ++task_id;
      }
    }

    g.leave();
  }
};


template<typename Op, typename View>
void
nested_map_func(Op op, View& vw)
{
  typedef nested_map_func_factory<Op>                   factory_t;
  typedef stapl::paragraph<
            stapl::default_scheduler, factory_t, View>  paragraph_t;

  paragraph_t(factory_t(vw.size(), op), vw)();
}

#endif // STAPL_TEST_CONTAINERS_NESTED_FACTORIES_HPP
