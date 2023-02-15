/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef TEST_RESULT_H
#define TEST_RESULT_H

#include <string>
#include <ostream>
#include <sstream>

// Prototypes of the methods defined in test_functions.h used to control
// reporting.
bool report_parallel(void);
bool report_sequential(void);

class test_result
{
public:
  test_result(void)
    : m_title(""), m_success(false), m_ref_ran(true)
  { }

  test_result(const char* n)
    : m_title(n), m_success(false), m_ref_ran(true)
  { }

  test_result(const std::string& n)
    : m_title(n), m_success(false), m_ref_ran(true)
  { }

  virtual ~test_result(void) = default;

  void set_title(const char* name)
  {
    m_title = name;
  }

  std::string get_title(void) const
  {
    return m_title;
  }

  void set_result(bool res, bool ref_ran)
  {
    m_success = res;
    m_ref_ran = ref_ran;
  }

  bool get_result(void) const
  {
    return m_success;
  }

  bool get_ref_ran(void) const
  {
    return m_ref_ran;
  }

protected:
  std::string m_title;
  bool        m_success;
  bool        m_ref_ran;

  friend std::ostream& operator<<(std::ostream& os, test_result const& res);
};

inline std::ostream& operator<<(std::ostream& os, test_result const& res)
{
  os << "Test: " << res.m_title << "\n";
  if (res.m_success && res.m_ref_ran) {
    os << "Status: PASS\n";
  }
  else if (res.m_ref_ran) {
    os << "Status: FAIL\n";
  }
  else {
    os << "Status: NA\n";
  }

  return os;
}


class timed_test_result
  : public test_result
{
public:
  timed_test_result(void)
    : test_result(), m_time_par(0.), m_time_seq(0.)
  { }

  timed_test_result(const char* n)
    : test_result(n), m_time_par(0.), m_time_seq(0.)
  { }

  timed_test_result(const std::string& n)
    : test_result(n), m_time_par(0.), m_time_seq(0.)
  { }

  virtual ~timed_test_result(void) = default;

  void set_time_parallel(double time)
  { m_time_par = time; }

  double get_time_parallel(void) const
  { return m_time_par; }

  void set_time_sequential(double time)
  { m_time_seq = time; }

  double get_time_sequential(void) const
  { return m_time_seq; }

  double get_speedup(void) const
  { return m_time_seq/m_time_par; }

  test_result& operator()(void)
  { return *this; }


protected:
  double m_time_par;
  double m_time_seq;

  friend std::ostream& operator<<(std::ostream& os,
                                  timed_test_result const& res);
};

inline std::ostream& operator<<(std::ostream& os, timed_test_result const& res)
{
  os << "Test: " << res.get_title() << "\n";
  if (res.get_result() && res.get_ref_ran()) {
    os << "Status: PASS\n";
    if (report_parallel())
    {
      os << "Version: stapl\n"
         << "Time: " << res.m_time_par << "\n";
    }
    if (report_sequential())
    {
      os << "Version: stl\n"
         << "Time: " << res.m_time_seq << "\n\n";
    }
  }
  else if (res.get_ref_ran()) {
    if (report_parallel())
    {
      os << "Status: FAIL\n"
         << "Version: stapl\n"
         << "Time: " << res.m_time_par << "\n";
    }
    else
      os << "Status: NA\n";
    if (report_sequential())
    {
      os << "Version: stl\n"
         << "Time: " << res.m_time_seq << "\n\n";
    }
  }
  else {
    os << "Status: NA\n";
    if (report_parallel())
    {
      os << "Version: stapl\n"
         << "Time: " << res.m_time_par << "\n\n";
    }
  }
  return os;
}

class combined_test_result
  : public timed_test_result
{
public:
  combined_test_result(void)
    : timed_test_result(), m_func(""),  m_message("")
  {}

  combined_test_result(timed_test_result const& r)
    : timed_test_result(r)
  { }

  combined_test_result(combined_test_result const& c,
                       timed_test_result const& r)
    : timed_test_result(c), m_func(c.m_func), m_message(c.m_message)
  {
    if (this->m_title == r.get_title())
    {
      this->m_time_par += r.get_time_parallel();
      this->m_time_seq += r.get_time_sequential();
      this->m_success = this->m_success && r.get_result();
      this->m_ref_ran = this->m_ref_ran && r.get_ref_ran();
    }
    else
    {
      std::cerr << "Combining tests with different titles: "
                << c.get_title() << ", and " << r.get_title() << "\n";
      std::exit(EXIT_FAILURE);
    }
  }

  virtual ~combined_test_result(void) = default;

  void add(timed_test_result const& r)
  {
    if (this->m_title == r.get_title())
    {
      this->m_success = this->m_success && r.get_result();
      this->m_ref_ran = this->m_ref_ran && r.get_ref_ran();
      this->m_time_par += r.get_time_parallel();
      this->m_time_seq += r.get_time_sequential();
    }
    else
    {
      std::cerr << "Combining tests with different titles: "
                << this->m_title << ", and " << r.get_title() << "\n";
      std::exit(EXIT_FAILURE);
    }
  }

  friend combined_test_result operator&(timed_test_result const &r1,
                                        timed_test_result const& r2);
  friend combined_test_result operator&(combined_test_result const& cr,
                                        timed_test_result const& r2);
  friend combined_test_result operator&(timed_test_result const& r2,
                                        combined_test_result const& cr);
  friend std::ostream& operator<<(std::ostream& os,
                                  combined_test_result const& cr);

private:
  std::string m_func;
  std::string m_message;
};

combined_test_result operator&(timed_test_result const& r1,
                               timed_test_result const& r2)
{
  combined_test_result c(r1);
  c.add(r2);
  return c;
}

combined_test_result operator&(combined_test_result const& cr,
                               timed_test_result const& r)
{
  return combined_test_result(cr, r);
}

combined_test_result operator&(timed_test_result const& r,
                               combined_test_result const& cr)
{
  return combined_test_result(cr, r);
}

std::ostream& operator<<(std::ostream& os, combined_test_result const& cr)
{
  os << "Test: " << cr.get_title() << "\n";
  if (cr.get_result() && cr.get_ref_ran()) {
    os << "Status: PASS\n";
    if (report_parallel())
    {
      os << "Version: stapl\n"
         << "Time: " << cr.get_time_parallel() << "\n";
    }
    if (report_sequential())
    {
      os << "Version: stl\n"
         << "Time: " << cr.get_time_sequential() << "\n\n";
    }
  }
  else if (cr.get_ref_ran()) {
    if (report_parallel())
    {
      os << "Status: FAIL\n"
         << "Version: stapl\n"
         << "Time: " << cr.get_time_parallel() << "\n";
    }
    else
      os << "Status: NA\n";
    if (report_sequential())
    {
      os << "Version: stl\n"
         << "Time: " << cr.get_time_sequential() << "\n\n";
    }
  }
  else {
    os << "Status: NA\n";
    if (report_parallel())
    {
       os << "Version: stapl\n"
          << "Time: " << cr.get_time_parallel() << "\n\n";
    }
  }
  return os;
}

#endif
