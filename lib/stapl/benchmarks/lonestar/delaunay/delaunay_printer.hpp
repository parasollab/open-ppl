/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_PRINTER_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_PRINTER_HPP

#include <stapl/utility/do_once.hpp>
#include <stapl/runtime/utility/timer.hpp>
#include <stapl/runtime/counter/counter.hpp>
#include <stapl/runtime/counter/default_counters.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <stdio.h>
#include <boost/io/ios_state.hpp>


namespace stapl
{


namespace delaunay
{


using std::string;
using stapl::counter;
using stapl::default_timer;


template <typename Data>
void formatedprint(Data const& data, size_t width=6, char fill=' ')
{
  boost::io::ios_flags_saver ifs(std::cout);
  std::cout.fill(fill);
  std::cout.width(width);
  std::cout << std::fixed << data;
}


template <typename Data>
void formatedprintnumber(Data const& data, size_t width=6, char fill=' ',
                         size_t precision = 6)
{
  boost::io::ios_flags_saver ifs(std::cout);
  std::cout.fill(fill);
  std::cout.width(width);
  std::cout.precision(precision);
  std::cout << std::right << std::fixed << data;
  std::cout.fill(' ');
  std::cout.precision(6);
}


void filledprint(char fill, size_t width)
{
  boost::io::ios_flags_saver ifs(std::cout);
  std::cout.fill(fill);
  std::cout.width(width);
  std::cout << fill;
  std::cout.fill(' ');
}


struct print_stat
{
  static std::string m_header_label;
  static size_t m_total_width;
  static size_t m_max_label_length;
  static size_t m_stat_width;
  static size_t m_num_pad;
  std::vector<print_stat> m_stats;
  typedef decltype(m_stats.back()) stat_iterator;


  print_stat* m_parent;
  std::string m_label;
  double m_time;
  double m_summed_time;
  int m_level;
  bool m_marked;
  counter<default_timer> m_timer;

  print_stat(std::string const& label, bool marked = false)
    : m_parent(0), m_label(label), m_time(0.0), m_summed_time(0.0), m_level(0),
      m_marked(marked)
  {
    m_stats.reserve(32);
  }

  print_stat(print_stat* parent_stat, std::string const& label,
             bool marked = false)
    : m_parent(parent_stat), m_label(label), m_time(0.0), m_summed_time(0.0),
      m_level(parent_stat->m_level + 1), m_marked(marked)
  {
    m_stats.reserve(32);
  }

  print_stat* add_stat(std::string const& label, double time = 0.0)
  {
    m_stats.push_back(print_stat(this, label, false));
    m_summed_time += time;
    m_stats.back().m_time += time;
    return &(m_stats.back());
  }

  print_stat* add_marked_stat(std::string const& label, double time = 0.0)
  {
    m_stats.push_back(print_stat(this, label, true));
    m_summed_time += time;
    m_stats.back().m_time += time;
    return &(m_stats.back());
  }

  void reset()
  {
    m_timer.reset();
  }

  void start()
  {
    m_timer.reset();
    m_timer.start();
  }

  double stop()
  {
    double value = m_timer.stop();
    m_time += value;
    if (m_parent != 0) {
      m_parent->m_summed_time += value;
    }
    m_timer.reset();
    return value;
  }

  int calculate_width()
  {
    int stat_width = 0;
    stat_width += m_num_pad;
    stat_width += 1;  // Space ' '

    stat_width += m_label.length();
    stat_width += 1;  // Colon ':'

    stat_width += 7;  // Time Precision

    for (int i=0; i<m_level; ++i) {
      stat_width += 2;  // Space + Bracket " ["
      stat_width += 6;  // Percent Precision
      stat_width += 3;  // Percent + Bracket + Space "%] "
    }
    stat_width += m_num_pad;
    return stat_width;
  }

  double get_total_time()
  {
    return std::max(m_time, m_summed_time);
  }

  double get_overall_time()
  {
    if (m_parent == 0) {
      return get_total_time();
    } else {
      return m_parent->get_overall_time();
    }
  }

  double get_parent_total_time()
  {
    if (m_parent != 0) {
      return m_parent->get_total_time();
    } else {
      return get_total_time();
    }
  }

  int print_overall_percentage(double time = -1.0)
  {
    return print_percentage(time, get_overall_time(), false);
  }

  int print_percentage(double time = -1.0, double total_time = -1.0)
  {
    return print_percentage(time, total_time, false);
  }

  int print_darkend_percentage(double time = -1.0, double total_time = -1.0)
  {
    return print_percentage(time, total_time, true);
  }

  int print_percentage(double time, double total_time, bool darken)
  {
    if (time < 0) {
      time = get_total_time();
    }
    if (total_time < 0) {
      total_time = get_parent_total_time();
    }
    int printed = 0;
    if (darken) {
      std::cout << "\e[2m";
    }
    std::cout << "[";                               printed += 1;
    double percent = 100.0 * time / total_time;
    formatedprintnumber(percent, 6, ' ', 2);        printed += 6;
    std::cout << "%] ";                             printed += 3;
    if (darken) {
      std::cout << "\e[0m";
    }
    return printed;
  }

  void recurse_print_percentages(double time, int& printed)
  {
    if (m_parent != 0) {
      m_parent->recurse_print_percentages(time, printed);
      printed += print_darkend_percentage(time);
    }
  }

  int print_percentages()
  {
    int printed = 0;
    if (m_parent != 0) {
      m_parent->recurse_print_percentages(get_total_time(), printed);
    }
    printed += print_percentage();
    return printed;
  }

  void print_separator(char ch = '-')
  {
    filledprint('#', m_num_pad);
    std::cout << " ";
    filledprint(ch, m_total_width - 2*m_num_pad - 2);
    std::cout << " ";
    filledprint('#', m_num_pad);
    std::cout << std::endl;
  }

  void print(bool all_precentages = false, bool overall_time = false)
  {
    int printed = m_num_pad + 1 + m_max_label_length + 2;
    filledprint('#', m_num_pad);
    std::cout << " ";
    formatedprint(m_label, m_max_label_length, ' ');
    std::cout << ": ";

    printed += 7 + 1;
    formatedprintnumber(m_time, 7, ' ', 3);
    std::cout << " ";

    if (overall_time) {
      printed += print_overall_percentage();
    } else if (all_precentages) {
      printed += print_percentages();
    } else {
      printed += print_percentage();
    }

    int remaining = m_total_width - printed - m_num_pad;
    if (remaining > 0) {
      filledprint(' ', remaining);
    }
    filledprint('#', m_num_pad);
    std::cout << std::endl;
  }


  void print_recursive(bool print_last_separator = false)
  {
    print(true);

    if (m_stats.size() != 0) {
      print_separator('-');
      size_t counter = 0;
      for (auto& stat : m_stats) {
        stat.print_recursive(++counter < m_stats.size());
      }
      if (print_last_separator) {
        print_separator('-');
      }
    }
  }

  void print_marked_recursive()
  {
    for (auto& stat : m_stats) {
      stat.print_marked_recursive();
    }
    if (m_marked) {
      print(false, true);
    }
  }

  void recurse_update()
  {
    m_summed_time = 0.0;
    m_time += m_timer.value();
    m_timer.reset();
    if (m_parent != 0) {
      m_parent->m_summed_time += m_time;
    }
    if (m_label.length() > m_max_label_length) {
      m_max_label_length = m_label.length();
      m_stat_width = calculate_width();
    }
    for (auto& stat : m_stats) {
      stat.m_parent = this;
      stat.recurse_update();
    }
  }

  void main_update()
  {
    m_total_width      = print_stat::m_header_label.length() + 8;
    m_max_label_length = 0;
    m_stat_width       = 0;
    m_num_pad          = 3;

    m_stat_width = 0;
    m_max_label_length = 0;
    recurse_update();

    m_total_width = print_stat::m_header_label.length() + 2*m_num_pad + 2;
  }

  void print_main_header()
  {
    int header_num_pad = m_num_pad;
    int diff = (int)m_stat_width - (int)m_total_width;
    if (diff > 0) {
      if (diff % 2 != 0) {
        diff += 1;
      }
      m_total_width += diff;
      header_num_pad += diff / 2;
    }

    filledprint('#', m_total_width);
    std::cout << std::endl;
    filledprint('#', header_num_pad);
    std::cout << " " << m_header_label << " ";
    filledprint('#', header_num_pad);
    std::cout << std::endl;
    filledprint('#', m_total_width);
    std::cout << std::endl;
  }

  void print_main(bool use_summed_time = false)
  {
    boost::io::ios_flags_saver ifs(std::cout);

    main_update();
    print_main_header();


    for (auto& stat : m_stats) {
      stat.print_recursive();
    }

    filledprint('#', m_total_width);
    std::cout << std::endl;


    for (auto& stat : m_stats) {
      stat.print_marked_recursive();
    }

    print();

    filledprint('#', m_total_width);
    std::cout << std::endl << std::endl;
  }
};


std::string print_stat::m_header_label;
size_t print_stat::m_total_width      = 8;
size_t print_stat::m_max_label_length = 0;
size_t print_stat::m_stat_width       = 0;
size_t print_stat::m_num_pad          = 3;


} // namespace delaunay


} // namespace stapl



#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_PRINTER_HPP */
