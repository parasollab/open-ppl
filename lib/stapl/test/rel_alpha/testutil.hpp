/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef TESTUTIL_HPP
#define TESTUTIL_HPP

#include <iostream>
#include <stapl/runtime.hpp>

extern void set_random_seed();

extern bool opt_quiet, opt_stapl, opt_manual, opt_noref;

//////////////////////////////////////////////////////////////////////
// output utilities
//////////////////////////////////////////////////////////////////////

template <typename Container>
void xprlog(Container& vec)
{
  using std::cerr;
  using std::endl;

  if( stapl::get_location_id() == 0 ) {
    int size = vec.size();

    cerr << "\nxprlog " << size << endl;
    for (int i = 0; i < 10; i += 1 ) {
      cerr << "[" << i << "] " << vec[i] << " ";
    }
    cerr << endl;

    if (size < 100 ) {
      return;
    }

    for (int i = 10; i < 100; i += 10 ) {
      cerr << "[" << i << "] " << vec[i] << " ";
    }
    cerr << endl;

    if (size < 1000 ) {
      return;
    }

    for (int i = 100; i < 1000; i += 100 ) {
      cerr << "[" << i << "] " << vec[i] << " ";
    }
    cerr << endl;

    if (size < 10000 ) {
      return;
    }

    for (int i = 1000; i < 10000; i += 1000 ) {
      cerr << "[" << i << "] " << vec[i] << " ";
    }
    cerr << endl;
    cerr.flush();
  }
}

template <typename Container>
void xpr10(Container& vec)
{
  using std::cerr;
  using std::endl;

  if( stapl::get_location_id() == 0 ) {
    int size = vec.size();

    cerr << "\nxprdec " << size << endl;
    for (int r = 0; r < size; r += 100 ) {
      for (int c = r; c < r+100; c += 10 ) {
        cerr << "[" << c << "] " << vec[c] << " ";
      }
      cerr << endl;
    }
    cerr << endl;
    cerr.flush();
  }
}

template <typename Container>
void xpr100(Container& vec)
{
  using std::cerr;
  using std::endl;

  if( stapl::get_location_id() == 0 ) {
    int size = vec.size();

    cerr << "\nxprdec " << size << endl;
    for (int r = 0; r < 100; r += 10 ) {
      for (int c = r; c < r+10; c += 1 ) {
        cerr << "[" << c << "] " << vec[c] << " ";
      }
      cerr << endl;
    }
    cerr << endl;
    cerr.flush();
  }
}

template <typename Container>
void xprall(Container& vec)
{
  using std::cerr;
  using std::endl;

  if( stapl::get_location_id() == 0 ) {
    int size = vec.size();

    cerr << "\nxprdec " << size << endl;
    for (int r = 0; r < size; r += 10 ) {
      for (int c = r; c < r+10; c += 1 ) {
        cerr << "[" << c << "] " << vec[c] << " ";
      }
      cerr << endl;
    }
    cerr << endl;
    cerr.flush();
  }
}

//////////////////////////////////////////////////////////////////////
// checking utilities
//////////////////////////////////////////////////////////////////////

template <typename Container>
bool check_container( const char *name,
                  const char *version1, const char *version2,
                  double time1, double time2,
                  const char *note1, const char *note2,
                  Container & p, Container & q)
{
  using std::cerr;
  using std::endl;

  int size = p.size();

  bool pass = true;
  int ndx = -1;
  for (int i = 0; i < size; i++) {
    if (p[i] != q[i] ) {
      ndx = i;
      pass = false;
      break;
    }
  }
  if (pass) {
    if( stapl::get_location_id() == 0 ) {
      cerr << "Test : t" << name << endl;
      cerr << "Status : PASS\n";
      cerr.flush();
    }
  } else {
    if( stapl::get_location_id() == 0 ) {
      cerr << "Test : t" << name << endl;
      cerr << "Status : FAIL\n";
      cerr.flush();
    }
  }

  if( opt_stapl && 0 == strcmp("STAPL",version1) ) {

    if( stapl::get_location_id() == 0 ) {
      cerr << "Version : " << version1 << endl;
    }

    if ( !opt_quiet ) {
      if( stapl::get_location_id() == 0 ) {
        cerr << "Time : " << (1000.0 * time1) << endl;
        cerr.flush();
      }
      if (note1 ) {
        if( stapl::get_location_id() == 0 ) {
          cerr << "Note : " << note1 << endl;
          cerr.flush();
        }
      }
    }
  }

  if( opt_manual && 0 == strcmp("manual",version2) ) {

    if( stapl::get_location_id() == 0 ) {
      cerr << "Version : " << version2 << endl;
    }

    if ( !opt_quiet ) {
      if( stapl::get_location_id() == 0 ) {
        cerr << "Time : " << (1000.0 * time2) << endl;
        cerr.flush();
      }
      if (note2 ) {
        if( stapl::get_location_id() == 0 ) {
          cerr << "Note : " << note2 << endl;
          cerr.flush();
        }
      }
    }
  }

  if (!pass && !opt_quiet ) {
    if( stapl::get_location_id() == 0 ) {
      cerr << "algorithm disagrees with reference at " << ndx << endl;
      cerr << "\nAlgorithm:\n";
      xprlog(p);
      cerr << "\nReference:\n";
      xprlog(q);
      cerr.flush();
    }
  }
  return pass;
}

template <typename Container>
bool check_multiarray( const char *name,
                  const char *version1, const char *version2,
                  double time1, double time2,
                  const char *note1, const char *note2,
                  Container & p, Container & q, size_t rank, size_t *shape)
{
  using std::cerr;
  using std::endl;

  int size = p.size();

  bool pass = true;
#if 0
  int ndx = -1;

  for (int i = 0; i < size; i++) {
    if (p[i] != q[i] ) {
      ndx = i;
      pass = false;
      break;
    }
  }

  if (pass) {
    if( stapl::get_location_id() == 0 ) {
      cerr << "Test : t" << name << endl;
      cerr << "Status : PASS\n";
      cerr.flush();
    }
  } else {
    if( stapl::get_location_id() == 0 ) {
      cerr << "Test : t" << name << endl;
      cerr << "Status : FAIL\n";
      cerr.flush();
    }
  }

  if( opt_stapl && 0 == strcmp("STAPL",version1) ) {

    if( stapl::get_location_id() == 0 ) {
      cerr << "Version : " << version1 << endl;
    }

    if ( !opt_quiet ) {
      if( stapl::get_location_id() == 0 ) {
        cerr << "Time : " << (1000.0 * time1) << endl;
        cerr.flush();
      }
      if (note1 ) {
        if( stapl::get_location_id() == 0 ) {
          cerr << "Note : " << note1 << endl;
          cerr.flush();
        }
      }
    }
  }

  if( opt_manual && 0 == strcmp("manual",version2) ) {

    if( stapl::get_location_id() == 0 ) {
      cerr << "Version : " << version2 << endl;
    }

    if ( !opt_quiet ) {
      if( stapl::get_location_id() == 0 ) {
        cerr << "Time : " << (1000.0 * time2) << endl;
        cerr.flush();
      }
      if (note2 ) {
        if( stapl::get_location_id() == 0 ) {
          cerr << "Note : " << note2 << endl;
          cerr.flush();
        }
      }
    }
  }

  if (!pass && !opt_quiet ) {
    if( stapl::get_location_id() == 0 ) {
      cerr << "algorithm disagrees with reference at " << ndx << endl;
      cerr << "\nAlgorithm:\n";
      xprlog(p);
      cerr << "\nReference:\n";
      xprlog(q);
      cerr.flush();
    }
  }
#endif
  return pass;
}

template <typename Element>
bool check_scalar( const char *name,
                  const char *version1, const char *version2,
                  double time1, double time2,
                  const char *note1, const char *note2,
                  Element p, Element q )
{
  using std::cerr;
  using std::endl;

  bool pass = true;
  if (p != q ) {
    pass = false;
  }

  if (pass) {
    if( stapl::get_location_id() == 0 ) {
      cerr << "Test : t" << name << endl;
      cerr << "Status : PASS\n";
      cerr.flush();
    }
  } else {
    if( stapl::get_location_id() == 0 ) {
      cerr << "Test : t" << name << endl;
      cerr << "Status : FAIL\n";
      cerr.flush();
    }
  }

  if( opt_stapl && 0 == strcmp("STAPL",version1) ) {

    if( stapl::get_location_id() == 0 ) {
      cerr << "Version : " << version1 << endl;
      cerr.flush();
    }

    if ( !opt_quiet ) {
      if( stapl::get_location_id() == 0 ) {
        cerr << "Time : " << (1000.0 * time1) << endl;
        cerr.flush();
      }
      if (note1 ) {
        if( stapl::get_location_id() == 0 ) {
          cerr << "Note : " << note1 << endl;
          cerr.flush();
        }
      }
    }
  }

  if( opt_manual && 0 == strcmp("manual",version2) ) {

    if( stapl::get_location_id() == 0 ) {
      cerr << "Version : " << version2 << endl;
      cerr.flush();
    }

    if ( !opt_quiet ) {
      if( stapl::get_location_id() == 0 ) {
        cerr << "Time : " << (1000.0 * time2) << endl;
        cerr.flush();
      }
      if (note2 ) {
        if( stapl::get_location_id() == 0 ) {
          cerr << "Note : " << note2 << endl;
          cerr.flush();
        }
      }
    }
  }

  if (!pass && !opt_quiet ) {
    if( stapl::get_location_id() == 0 ) {
      cerr << "algorithm disagrees with reference \n";
      pass = false;
      cerr << "Algorithm : " << p << endl;
      cerr << "Reference : " << q << endl;
      cerr.flush();
    }
  }
  return pass;
}

inline bool known_fail( const char *name,
                        const char *version1, const char *version2,
                        const char *note1, const char *note2 )
{
  using std::cerr;
  using std::endl;

  if( stapl::get_location_id() == 0 ) {
    cerr << "Test : t" << name << endl;
    cerr << "Status : FAIL\n";

    if( opt_stapl && 0 == strcmp("STAPL",version1) ) {
      cerr << "Version : " << version1 << endl;

      if ( !opt_quiet ) {
        cerr << "Time : 0.0\n";
      }
      if (note1 ) {
        cerr << "Note : " << note1 << endl;
      }
    }

    if( opt_manual && 0 == strcmp("manual",version2) ) {

      cerr << "Version : " << version2 << endl;

      if ( !opt_quiet ) {
        cerr << "Time : 0.0\n";
      }
      if (note2 ) {
        cerr << "Note : " << note2 << endl;
      }
    }
  }

  return false;
}

#endif
