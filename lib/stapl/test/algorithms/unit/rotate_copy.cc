/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <iostream>
#include <stapl/array.hpp>
#include <stapl/vector.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/views/counting_view.hpp>

using namespace std;
using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::array<int> p_string_type;
  typedef array_view<p_string_type> pstringView;

  typedef stapl::array<char> p_char_type;
  typedef array_view<p_char_type> pcharView;

#if BUG_1120_RESOLVED
  typedef stapl::vector<int> p_int_type_v;
  typedef vector_view<p_int_type_v> pintView_v;

  typedef stapl::vector<char> p_char_type_v;
  typedef vector_view<p_char_type_v> pcharView_v;
#endif

  int k = 3;
  unsigned int j = 0;
  int nelem = 10;
  int l;
  int p = 5;
  if (argc > 1)
  {
    nelem = atoi(argv[1]);
    if (nelem == 0)
    {
      cout << "The number of elements must be greater than 0.\n";
      return EXIT_FAILURE;
    }
  }

  bool result;
  p_string_type pstr1(nelem);
  p_string_type pstr2(nelem);

  pstringView pstrv1(pstr1);
  pstringView pstrv2(pstr2);

  p_char_type pchar1(nelem);
  p_char_type pchar2(nelem);

  pcharView pcharv1(pchar1);
  pcharView pcharv2(pchar2);

#if BUG_1120_RESOLVED
  //Vector

  p_int_type_v pvint1(nelem);
  p_int_type_v pvint2(nelem);

  pintView_v pvintv1(pvint1);
  pintView_v pvintv2(pvint2);

  p_char_type_v pvchar1(nelem);
  p_char_type_v pvchar2(nelem);

  pcharView_v pvcharv1(pvchar1);
  pcharView_v pvcharv2(pvchar2);
#endif

  //Array
  copy(counting_view<int> (nelem, 0), pstrv1);

  copy(counting_view<char> (nelem, 'a'), pcharv1);

#if BUG_1120_RESOLVED
  //vector
  copy(counting_view<int> (nelem, 0), pvintv1);

  copy(counting_view<char> (nelem, 'a'), pvcharv1);
#endif

  //array
  rotate_copy(pstrv1, pstrv2, k);

  rotate_copy(pcharv1, pcharv2, k);

#if BUG_1120_RESOLVED
  //Vector
  rotate_copy(pvintv1, pvintv2, k);

  rotate_copy(pvcharv1, pvcharv2, k);

  //Vector in array

  rotate_copy(pvintv1, pstrv2, k);
#endif

  //Test all of the elements
  //Test a rotate copy of ---a p_array with a p_array-- of INT
  result = true;
  while (j < pstr1.size())
  {
    l = (nelem - k + j) % nelem;
    if (pstr1[j] == pstr2[l])
      result = result&true;
    else
      result = result&false;
    j++;
  }

#if BUG_1120_RESOLVED
  //Test a rotate copy of ---a p_array with a p_vector-- of INT
  copy(counting_view<int> (nelem, 20), pstrv2);
  copy(pvintv2, pstrv2);
  j = 0;
  while (j < pstr1.size())
  {
    l = (nelem - k + j) % nelem;
    if (pstr1[j] == pstr2[l])
      result = result&true;
    else
      result = result&false;
    j++;
  }

  //Test a rotate copy of ---a p_vector with a p_vector-- of INT
  copy(counting_view<int> (nelem, 20), pstrv1);
  copy(counting_view<int> (nelem, 30), pstrv2);
  copy(pvintv1, pstrv1);
  copy(pvintv2, pstrv2);
  j = 0;
  while (j < pstr1.size())
  {
    l = (nelem - k + j) % nelem;
    if (pstr1[j] == pstr2[l])
      result = result&true;
    else
      result = result&false;
    j++;
  }

  //Test a rotate copy of ---a p_vector with a p_array-- of INT
  copy(counting_view<int> (nelem, 20), pstrv1);
  copy(pvintv1, pstrv1);
  j = 0;
  while (j < pstr1.size())
  {
    l = (nelem - k + j) % nelem;
    if (pstr1[j] == pstr2[l])
      result = result&true;
    else
      result = result&false;
    j++;
  }
#endif

  //Test a rotate copy of ---a p_array with a p_array-- of CHAR
  j = 0;
  while (j < pchar1.size())
  {
    l = (nelem - k + j) % nelem;
    if (pchar1[j] == pchar2[l])
      result = result&true;
    else
      result = result&false;
    j++;
  }

#if BUG_1120_RESOLVED
  //Test a rotate copy of ---a p_vector with a p_vector-- of CHAR
  copy(counting_view<char> (nelem, 'p'), pcharv1);
  copy(counting_view<char> (nelem, 'h'), pcharv2);
  copy(pvcharv1, pcharv1);
  copy(pvcharv2, pcharv2);
  j = 0;
  while (j < pchar1.size())
  {
    l = (nelem - k + j) % nelem;
    if (pchar1[j] == pchar2[l])
      result = result&true;
    else
      result = result&false;
    j++;
  }

  //Test a rotate copy of ---a p_vector with a p_array-- of INT increasing k
  for (int z = -14; z < 13; z++)
  {
    rotate_copy(pvintv1, pstrv2, z);
    copy(counting_view<int> (nelem, 20), pstrv1);
    copy(pvintv1, pstrv1);
    j = 0;
    while (j < pstr1.size())
    {
      l = (nelem - z%nelem + j) % nelem;
      if (pstr1[j] == pstr2[l])
        result = result&true;
      else
        result = result&false;
      j++;
    }
  }
#endif
  if (get_location_id() == 0)
  {
    cout << endl;
    cout << "Test rotate_copy : ";
    if (result == true)
      cout << "[PASSED] ";
    else
      cout << "[FAILED] ";
  }

  //Test of copy_n
  stapl::copy_n(pstrv1, pstrv2, p);
  l = 0;
  result = true;
  while (l < p) {
    if (pstr1[l] != pstr2[l]) {
      result = false;
    }
    l++;
  }

#if BUG_1120_RESOLVED
  p = p + 2;
  stapl::copy_n(pcharv1, pvcharv2, p);
  l = 0;
  while (l < p) {
    if (pchar1[l] != pvchar2[l]) {
      result = result&false;
    }
    l++;
  }
#endif
  if (get_location_id() == 0) {
    cout << endl << "Test of copy_n : ";
    if (result) {
      cout << "[PASSED]";
    } else {
      cout << "[FAILED]";
    }
    cout << endl;
  }

  return EXIT_SUCCESS;
}
