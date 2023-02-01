/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_TEST_PATTERNS_HPP
#define STAPL_PARAGRAPH_TEST_PATTERNS_HPP

#include <stapl/paragraph/paragraph.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#define PARAGRAPH_PATTERN_DEBUG 0

#if PARAGRAPH_PATTERN_DEBUG
#include <sstream>
#endif

const int INVALID = -1;

template<bool INVERT>
class basic_pattern_base
{
  int m_rows;
  int m_cols;

  typedef std::vector<size_t> list_t;
public:
  basic_pattern_base(int rows, int cols)
    : m_rows(rows), m_cols(cols)
  {
    stapl_assert(m_rows > 0 && m_cols > 0, "invalid rows or cols specified");
  }

  list_t get_pred_list(int) const;

  list_t get_succ_list(int) const;

  int rows(void) const { return m_rows; }

  int cols(void) const { return m_cols; }

  int num_elems(void) const { return m_cols * m_rows; }

  void define_type(stapl::typer& t)
  {
    t.member(m_rows);
    t.member(m_cols);
  }

protected:
  void check_tid(int tid) const
  {
    stapl_assert(
      (tid >= 0) && (tid < this->num_elems()), "received invalid task id!"
    );
  }

  void check_tid_invalid(int tid) const
  {
    stapl_assert(
      (tid >= -1) && (tid < this->num_elems()), "received invalid task id!"
    );
  }

private:
  virtual int get_pred(int) const = 0;

  virtual int get_succ(int) const = 0;
};

template<>
std::vector<size_t> basic_pattern_base<false>::get_pred_list(int tid) const
{
  int pred = get_pred(tid);
  this->check_tid_invalid(pred);
  return pred==INVALID ? list_t() : list_t(1, pred);
}

template<>
std::vector<size_t> basic_pattern_base<true>::get_pred_list(int tid) const
{
  int pred = this->get_succ(tid);
  this->check_tid_invalid(pred);
  return pred==INVALID ? list_t() : list_t(1, pred);
}

template<>
std::vector<size_t> basic_pattern_base<false>::get_succ_list(int tid) const
{
  int succ = this->get_succ(tid);
  this->check_tid_invalid(succ);
  return succ==INVALID ? list_t() : list_t(1, succ);
}

template<>
std::vector<size_t> basic_pattern_base<true>::get_succ_list(int tid) const
{
  int succ = this->get_pred(tid);
  this->check_tid_invalid(succ);
  return succ==INVALID ? list_t() : list_t(1, succ);
}

//  0   0   0
//
//  0   0   0
//
//  0   0   0
template<bool INVERT = false>
class basic_pattern_one
  : public basic_pattern_base<INVERT>
{
public:
  basic_pattern_one(int rows, int cols)
    : basic_pattern_base<INVERT>(rows, cols)
  { }

private:
  int get_pred(int tid) const
  {
    this->check_tid(tid);
    return INVALID;
  }

  int get_succ(int tid) const
  {
    this->check_tid(tid);
    return INVALID;
  }
};

// 0-->0-->0
//
// 0-->0-->0
//
// 0-->0-->0
template<bool INVERT = false>
class basic_pattern_two
  : public basic_pattern_base<INVERT>
{
public:
  basic_pattern_two(int rows, int cols)
    : basic_pattern_base<INVERT>(rows, cols)
  { }

private:
  int get_pred(int tid) const
  {
    this->check_tid(tid);
    if ( (tid%this->cols()) == 0 )
      return INVALID;
    else
      return tid-1;
  }

  int get_succ(int tid) const
  {
    this->check_tid(tid);
    if ( ((tid+1)%this->cols()) == 0 )
      return INVALID;
    else
      return tid+1;
  }
};

// 0  0  0
// |  |  |
// v  v  v
// 0  0  0
template<bool INVERT = false>
class basic_pattern_three
  : public basic_pattern_base<INVERT>
{
public:
  basic_pattern_three(int rows, int cols)
    : basic_pattern_base<INVERT>(rows, cols)
  { }

private:
  int get_pred(int tid) const
  {
    this->check_tid(tid);
    if ( (tid/this->cols()) == 0 )
      return INVALID;
    else
      return tid-this->cols();
  }

  int get_succ(int tid) const
  {
    this->check_tid(tid);
    if ( (tid/(this->cols()*(this->rows()-1))) == 0 )
      return tid + this->cols();
    else
      return INVALID;
  }
};

// 0  0  0  |
//  \  \    |
//   \  \   |
//    v  v  |
// 0  0  0  v
template<bool INVERT = false>
class basic_pattern_four
  : public basic_pattern_base<INVERT>
{
public:
  basic_pattern_four(int rows, int cols)
    : basic_pattern_base<INVERT>(rows, cols)
  { }

private:
  int get_pred(int tid) const
  {
    this->check_tid(tid);
    if ( (tid/this->cols()) == 0 || (tid%this->cols()) == 0)
      return INVALID;
    else
      return tid-this->cols()-1;
  }

  int get_succ(int tid) const
  {
    this->check_tid(tid);
    if ( (tid/(this->cols()*(this->rows()-1))) == 0
        && ((tid+1)%this->cols()) != 0 )
      return tid + this->cols() + 1;
    else
      return INVALID;
  }
};

// 0  0  0
//   /  /
//  /  /
// v  v
// 0  0  0
template<bool INVERT = false>
class basic_pattern_five
  : public basic_pattern_base<INVERT>
{
public:
  basic_pattern_five(int rows, int cols)
    : basic_pattern_base<INVERT>(rows, cols)
  { }

private:
  int get_pred(int tid) const
  {
    this->check_tid(tid);
    if ( (tid/this->cols()) == 0 || ((tid+1)%this->cols()) == 0)
      return INVALID;
    else
      return tid-this->cols()+1;
  }

  int get_succ(int tid) const
  {
    this->check_tid(tid);
    if ( (tid/(this->cols()*(this->rows()-1))) == 0 && (tid%this->cols()) != 0 )
      return tid + this->cols() - 1;
    else
      return INVALID;
  }
};

// Pattern composition
// WARNING: Only very simple pattern composition is supported.
//          If a pattern is used more than once in any composition,
//          then the result is undefined.
template<typename Pattern1, typename Pattern2>
class composed_pattern
{
  Pattern1 m_pattern1;
  Pattern2 m_pattern2;
  int      m_rows;
  int      m_cols;

  typedef std::vector<size_t> list_t;
public:
  composed_pattern(Pattern1 const& pattern1, Pattern2 const& pattern2)
    : m_pattern1(pattern1),
      m_pattern2(pattern2),
      m_rows(pattern1.rows()),
      m_cols(pattern1.cols())
  {
    // make sure the patterns are compatible
    stapl_assert(m_pattern1.rows() == m_pattern2.rows()
                 && m_pattern1.cols() == m_pattern2.cols(),
                 "patterns differ in number of rows or number of cols or both");
  }

  list_t get_pred_list (int tid) const
  {
    list_t pred_list(m_pattern1.get_pred_list(tid));
    list_t list2 = m_pattern2.get_pred_list(tid);
    pred_list.insert(pred_list.end(), list2.begin(), list2.end());
    return pred_list;
  }

  list_t get_succ_list (int tid) const
  {
    list_t succ_list(m_pattern1.get_succ_list(tid));
    list_t list2 = m_pattern2.get_succ_list(tid);
    succ_list.insert(succ_list.end(), list2.begin(), list2.end());
    return succ_list;
  }

  int rows(void) const { return m_rows; }

  int cols(void) const { return m_cols; }

  int num_elems(void) const { return m_rows * m_cols; }

  void define_type(stapl::typer& t)
  {
    t.member(m_pattern1);
    t.member(m_pattern2);
    t.member(m_rows);
    t.member(m_cols);
  }
};


struct empty_wf
{
  size_t m_sleep_time;

  typedef void result_type;

  empty_wf(size_t sleep_time)
    : m_sleep_time(sleep_time)
  { }

  result_type operator()() const
  { std::this_thread::sleep_for(std::chrono::microseconds{m_sleep_time}); }

  void define_type(stapl::typer& t)
  { t.member(m_sleep_time); }
};


class pattern_partitioner_t
{
  int m_cols_per_proc;
  int m_rows;

  typedef std::vector<size_t> list_t;
public:
  template<typename Pattern>
  pattern_partitioner_t(Pattern const& pattern)
    : m_cols_per_proc(pattern.cols()/stapl::get_num_locations()),
      m_rows(pattern.rows())
  {
    // the number of cols should be perfectly divisible
    // by the number of procs. For now the number of rows
    // should be > 1. The number of cols_per_proc should
    // obviously be > 0.
    stapl_assert((!(pattern.cols()%stapl::get_num_locations())
                  || stapl::get_num_locations() == 1 )
                  && m_cols_per_proc > 0 && m_rows > 1,
                  "invalid configiuration specified !" );
  }

  list_t get_local_tids(size_t pid)
  {
    stapl_assert( pid<stapl::get_num_locations(), "received invalid proc id!" );
    list_t l;
    for (int y = 0; y < m_rows; y++) {
      for (int x = 0; x < m_cols_per_proc; x++) {
        int tid = (y*(m_cols_per_proc*stapl::get_num_locations()))
                  + (pid*m_cols_per_proc) + x;
        l.push_back(tid);
      }
    }
    return l;
  }
};

template<typename Pattern, typename SchedInfo>
struct viewless_pattern_factory_t
  : public stapl::task_factory_base
{
  typedef std::vector<size_t>            list_t;

private:
  Pattern             m_pattern;
  size_t              m_tasks_per_call;
  size_t              m_sleep_time;
  std::vector<size_t> m_tasks;
  size_t              m_last_index;

public:
  using result_type = empty_wf::result_type;
  using internal_result_type = empty_wf::result_type;

  viewless_pattern_factory_t(Pattern const& pattern, size_t tasks_per_call,
                             size_t sleep_time)
    : stapl::task_factory_base(true),
      m_pattern(pattern),
      m_tasks_per_call(tasks_per_call),
      m_sleep_time(sleep_time),
      m_tasks(
        pattern_partitioner_t(pattern).get_local_tids(stapl::get_location_id())
      ),
      m_last_index(0)
  { }

  template<typename TGV>
  void operator()(TGV const& tgv)
  {
    // prepare for incremental generation
    unsigned int start_index = m_last_index;
    // advance the last_index
    m_last_index += m_tasks_per_call;

    if (m_tasks_per_call==0 || m_last_index >= m_tasks.size()) {
      m_last_index = m_tasks.size();
      this->m_finished = true;
    }

    #if PARAGRAPH_PATTERN_DEBUG
    std::cout << "PID " << stapl::get_location_id() << ": adding tasks "
              << start_index << " through " << m_last_index-1 << std::endl;
    #endif

    // add the tasks
    for (size_t index = start_index; index < m_last_index; ++index) {
      size_t tid       = m_tasks[index];
      list_t pred_list = m_pattern.get_pred_list(tid);
      list_t succ_list = m_pattern.get_succ_list(tid);

      #if PARAGRAPH_PATTERN_DEBUG
        std::stringstream ss;
        ss << "PID " << stapl::get_location_id() << ": tid " << tid
           << " preds =";

        for (unsigned int i=0; i<pred_list.size(); ++i)
          ss << " " << pred_list[i];

        ss << " succs =";
        for (unsigned int i=0; i<succ_list.size(); ++i)
          ss << " " << succ_list[i];

        ss << std::endl;
        std::cout << ss.str();
      #endif
      if (pred_list.size())
      {
        tgv.add_task(
          SchedInfo(), tid, empty_wf(m_sleep_time), pred_list, succ_list.size()
        );
      }
      else
      {
        tgv.add_task(
          SchedInfo(), tid, empty_wf(m_sleep_time), succ_list.size()
        );
      }
    }

    //stapl::rmi_fence();
  }

  void reset()
  {
    stapl_assert(false, "why reset?"); this->m_finished = false;
  }
};

// FIXME: combine the the viewless pattern factory and the pattern factory
template<typename Pattern, typename WF, typename ViewIn,
          typename ViewOut, typename SchedInfo>
struct pattern_factory_t
  : public stapl::task_factory_base
{
  typedef std::vector<size_t>            list_t;

private:
  Pattern             m_pattern;
  size_t              m_tasks_per_call;
  size_t              m_sleep_time;
  std::vector<size_t> m_tasks;
  size_t              m_last_index;
  ViewIn              m_vw_in;
  ViewOut             m_vw_out;

public:
  //typedef typename WF::result_type result_type;
  using result_type = void;
  using internal_result_type = typename WF::result_type;

  pattern_factory_t(Pattern const& pattern, size_t tasks_per_call,
                    size_t sleep_time, ViewIn& vw_in, ViewOut& vw_out)
    : stapl::task_factory_base(true),
      m_pattern(pattern),
      m_tasks_per_call(tasks_per_call),
      m_sleep_time(sleep_time),
      m_tasks(
        pattern_partitioner_t(pattern).get_local_tids(stapl::get_location_id())
      ),
      m_last_index(0),
      m_vw_in(vw_in),
      m_vw_out(vw_out)
  { }

  template<typename TGV>
  void operator()(TGV const& tgv)
  {
    // prepare for incremental generation
    unsigned int start_index = m_last_index;
    // advance the last_index
    m_last_index += m_tasks_per_call;

    if (m_tasks_per_call==0 || m_last_index >= m_tasks.size()) {
      m_last_index = m_tasks.size();
      this->m_finished = true;
    }

    #if PARAGRAPH_PATTERN_DEBUG
    std::cout << "PID " << stapl::get_location_id() << ": adding tasks "
              << start_index << " through " << m_last_index-1 << std::endl;
    #endif

    // add the tasks
    for (size_t index = start_index; index < m_last_index; ++index) {
      size_t tid       = m_tasks[index];
      list_t pred_list = m_pattern.get_pred_list(tid);
      list_t succ_list = m_pattern.get_succ_list(tid);

      #if PARAGRAPH_PATTERN_DEBUG
        std::stringstream ss;
        ss << "PID " << stapl::get_location_id() << ": tid " << tid
           << " preds =";

        for (unsigned int i=0; i<pred_list.size(); ++i)
          ss << " " << pred_list[i];

        ss << " succs =";
        for (unsigned int i=0; i<succ_list.size(); ++i)
          ss << " " << succ_list[i];

        ss << std::endl;
        std::cout << ss.str();
      #endif

      switch(pred_list.size())
      {
        case 0:
        {
          tgv.add_task(SchedInfo(), tid,
                       WF(m_vw_in, m_vw_out, tid), succ_list.size());
          break;
        }
        case 1:
        {
          tgv.add_task(
            SchedInfo(), tid, WF(m_vw_in, m_vw_out, tid), succ_list.size(),
            stapl::consume<internal_result_type>(tgv, pred_list[0]));
          break;
        }
        case 2:
        {
          tgv.add_task(
            SchedInfo(), tid, WF(m_vw_in, m_vw_out, tid), succ_list.size(),
            stapl::consume<internal_result_type>(tgv, pred_list[0]),
            stapl::consume<internal_result_type>(tgv, pred_list[1])
          );
          break;
        }
        case 3:
        {
          tgv.add_task(
            SchedInfo(), tid, WF(m_vw_in, m_vw_out, tid), succ_list.size(),
            stapl::consume<internal_result_type>(tgv, pred_list[0]),
            stapl::consume<internal_result_type>(tgv, pred_list[1]),
            stapl::consume<internal_result_type>(tgv, pred_list[2])
          );
          break;
        }
        default:
        {
          stapl_assert(false, "only 0<= num_preds <=3 are supported");
          break;
        }
      }
    }

    //stapl::rmi_fence();
  }

  void reset()
  {
    stapl_assert(false, "why reset?"); this->m_finished = false;
  }
};

// FIXME: combine the the factories !
template<typename Pattern, typename WF, typename ViewIn, typename ViewOut>
struct delayed_succs_factory_t
  : public stapl::task_factory_base
{
  typedef std::vector<size_t>           list_t;

private:
  Pattern                                 m_pattern;
  std::vector<size_t>                     m_tasks;
  std::vector<std::pair<size_t, size_t> > m_succs_info;
  ViewIn                                  m_vw_in;
  ViewOut                                 m_vw_out;
  bool                                    m_first_call;

public:
  using result_type = void;
  using internal_result_type = typename WF::result_type;

  delayed_succs_factory_t(Pattern const& pattern,
                          ViewIn& vw_in, ViewOut& vw_out)
    : stapl::task_factory_base(true),
      m_pattern(pattern),
      m_tasks(
        pattern_partitioner_t(pattern).get_local_tids(stapl::get_location_id())
      ),
      m_vw_in(vw_in),
      m_vw_out(vw_out),
      m_first_call(true)
  {
    for (size_t index = 0; index < m_tasks.size(); ++index) {
      size_t tid        = m_tasks[index];
      list_t succ_list  = m_pattern.get_succ_list(tid);
      m_succs_info.push_back(std::make_pair(tid, succ_list.size()));
    }
  }

  template<typename TGV>
  void operator()(TGV const& tgv)
  {
    if (!m_first_call)
    {
      if (m_succs_info.empty())
      {
        this->m_finished = true;
      }
      else
      {
        // randomly set succs for one task
        size_t index = rand()%m_succs_info.size();
        tgv.set_num_succs(
          m_succs_info[index].first, m_succs_info[index].second
        );
        m_succs_info.erase(m_succs_info.begin() + index);
      }

      return;
    }

    m_first_call = false;


    // randomly call set num succs for some tasks
    stapl_assert(!m_succs_info.empty(),
      "no tasks to add in first call ?");

    size_t n_size = m_succs_info.size();
    size_t index  = rand()%n_size;

    for (size_t i=index; i<n_size; ++i)
      tgv.set_num_succs(m_succs_info[i].first, m_succs_info[i].second);

    m_succs_info.erase(m_succs_info.begin() + index, m_succs_info.end());


    // add the tasks
    for (size_t index = 0; index < m_tasks.size(); ++index) {
      size_t tid        = m_tasks[index];
      list_t pred_list  = m_pattern.get_pred_list(tid);
      list_t succ_list  = m_pattern.get_succ_list(tid);

      switch(pred_list.size())
      {
        case 0:
        {
          tgv.add_task(tid, WF(m_vw_in, m_vw_out, tid), stapl::defer_spec);
          break;
        }
        case 1:
        {
          tgv.add_task(tid, WF(m_vw_in, m_vw_out, tid), stapl::defer_spec,
            stapl::consume<internal_result_type>(tgv, pred_list[0]));
          break;
        }
        case 2:
        {
          tgv.add_task(tid, WF(m_vw_in, m_vw_out, tid), stapl::defer_spec,
            stapl::consume<internal_result_type>(tgv, pred_list[0]),
            stapl::consume<internal_result_type>(tgv, pred_list[1]));
          break;
        }
        case 3:
        {
          tgv.add_task(tid, WF(m_vw_in, m_vw_out, tid), stapl::defer_spec,
            stapl::consume<internal_result_type>(tgv, pred_list[0]),
            stapl::consume<internal_result_type>(tgv, pred_list[1]),
            stapl::consume<internal_result_type>(tgv, pred_list[2]));
          break;
        }
        default:
        {
          stapl_assert(false, "only 0<= num_preds <=3 are supported");
          break;
        }
      }
    }
    //stapl::rmi_fence();
  }

  void reset()
  {
    stapl_assert(false, "why reset?"); this->m_finished = false;
  }
};


#endif // STAPL_PARAGRAPH_TEST_PATTERNS_HPP

