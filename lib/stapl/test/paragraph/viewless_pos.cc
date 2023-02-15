/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cmath>
#include <iostream>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/utility/do_once.hpp>

struct empty_wf
{
  typedef int result_type;

  result_type operator()(void) const
  {
    return 1;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to return the result of the map_reduce operation
///   on each location.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct result_wf
{
  typedef T result_type;

  template<typename Ref>
  T operator()(Ref x) const
  {
    return x;
  }
};


struct viewless_factory
  : public stapl::task_factory_base
{
private:
  struct tree_info
  {
    int width;
    int height;
    int leaf_offset;
    int id_offset;
    int root_task_id;
    int curr_id;
  };

  struct contains_leaf
  {
    int leaf;

    explicit contains_leaf(int l)
      : leaf(l)
    { }

    bool operator()(tree_info const& tree)
    {
      return (tree.leaf_offset <= leaf)
        && (leaf < tree.leaf_offset + tree.width);
    }
  };

public:
  using result_type = empty_wf::result_type;

  template<typename TGV>
  void operator()(TGV const& tgv)
  {
    //suppress warnings about conversions.
    #pragma warning( disable : 2259 )
    float n = static_cast<float>(stapl::get_num_locations());
    std::vector<tree_info> trees;
    int elem_remaining = n;
    int leaf_offset = 0;
    int id_offset = 0;

    for (int curr_width = std::pow(2.0,std::floor(std::log2(n)));
         curr_width != 0 && elem_remaining != 0;
         curr_width >>= 1)
    {
      if (curr_width <= elem_remaining)
      {
        tree_info t;

        t.width = curr_width;
        t.height = static_cast<int>(std::log2(curr_width));
        t.leaf_offset = leaf_offset;
        t.id_offset = id_offset;
        t.root_task_id = id_offset + 2*curr_width - 2;
        t.curr_id = id_offset + curr_width; //curr_id is not used for map tasks

        trees.push_back(t);

        elem_remaining -= curr_width;
        leaf_offset += curr_width;
        id_offset += 2*curr_width - 1;
      }
    }

    size_t i = stapl::get_location_id();

    // first create the set of map tasks
    auto t = std::find_if(trees.begin(), trees.end(), contains_leaf(i));

    const size_t tid = i - t->leaf_offset + t->id_offset;
    //const size_t consumer_tid =
    //  t->width + (i - t->leaf_offset)/2 + t->id_offset;
    //produce(consumer_tid)
    const size_t num_succs = 1;

    tgv.add_task(tid, empty_wf(), num_succs);

    stapl::rmi_fence();

    // tasks to combine results from map tasks together
    int n_steps = static_cast<int>(std::ceil(std::log2(n)));
    for (int s = 1; s <= n_steps; ++s)
    {
      t           = std::find_if(trees.begin(), trees.end(), contains_leaf(i));
      int two_s   = static_cast<int>(std::pow(2.0, s));
      int two_sm1 = static_cast<int>(std::pow(2.0, s-1));
      //int two_sp1 = static_cast<int>(std::pow(2.0, s+1));

      if (s <= t->height && i % two_s == 0)
      {
        const std::size_t tid_curr  = t->curr_id + (i - t->leaf_offset)/two_s;

        const std::size_t tid_left  =
          t->curr_id - t->width/two_sm1 + 2*(i - t->leaf_offset)/two_s;

        const std::size_t tid_right = tid_left + 1;
        //std::size_t tid_succ;
        //if (s < t->height)
        //tid_succ = t->curr_id + t->width/two_s + (i - t->leaf_offset)/two_sp1;
        //else
        //tid_succ =  trees.back().root_task_id + std::distance(t, trees.end());

        tgv.add_task(tid_curr, stapl::plus<int>(), num_succs,
                      stapl::consume<int>(tgv, tid_left),
                      stapl::consume<int>(tgv, tid_right));
      }

      for (t = trees.begin(); t != trees.end(); ++t)
      {
        if (s <= t->height)
        {
          t->curr_id += t->width/two_s;
        } else if ((s == t->height + 1) &&
                   (std::distance(t, trees.end()) > 1) &&
                   (i == static_cast<size_t>(t->leaf_offset)))
        {
          const std::size_t tid_curr  =
            trees.back().root_task_id + std::distance(t, trees.end()) - 1;
          const std::size_t tid_left  = t->root_task_id;
          const std::size_t tid_right = (t+1)->root_task_id;
          //const std::size_t tid_succ  = tid + 1;

          tgv.add_task(tid_curr, stapl::plus<int>(), num_succs,
                       stapl::consume<int>(tgv, tid_left),
                       stapl::consume<int>(tgv, tid_right));

          t->root_task_id = tid;
        }
      }
    }

    const size_t loc        = stapl::get_location_id();
    const size_t retval_tid = trees.back().root_task_id + trees.size() + loc;

    size_t parent_tid;

    if (loc == 0)
      parent_tid = trees.back().root_task_id + trees.size() - 1;
    else
      parent_tid = trees.back().root_task_id + trees.size() + loc / 2;

    // compute the set of tasks consuming this result task.
    size_t nlocs = this->get_num_locations();
    //std::vector<size_t> successors;
    std::size_t num_succs2 = 1;
    if (loc == 0)
    {
      // location 0 is only consumed by location 1 if it exists.
      if (nlocs != 1)
        //successors.push_back(1);
        ++num_succs2;
    }
    else if (nlocs == 2*loc + 1)
    {
      // if the tree isn't full
       ++num_succs2;
      //successors.push_back(2*loc);
    }
    else if (loc < nlocs/2)
    {
      num_succs2 += 2;
      //successors.push_back(2*loc);
      //successors.push_back(2*loc+1);
    }

    tgv.add_task(
      retval_tid, result_wf<int>(),
      num_succs2, stapl::consume<int>(tgv, parent_tid)
    );
    tgv.set_result(retval_tid);
    this->m_finished = true;
  }

  void reset()
  {
    this->m_finished = false;
  }
};


stapl::exit_code stapl_main(int, char*[])
{
  stapl::do_once([](void) {
    std::cout << "Testing paragraph with no views...";
  });

  size_t result       = make_paragraph(viewless_factory())();
  const size_t n_locs = stapl::get_num_locations();

  stapl::do_once([=](void) {
   if (result == n_locs)
     std::cout << "Passed\n";
    else
     std::cout << "Failed\n";
  });

  return EXIT_SUCCESS;
}
