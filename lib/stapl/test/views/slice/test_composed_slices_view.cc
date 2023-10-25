/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/multiarray.hpp>
#include <stapl/views/slices_view.hpp>

using namespace stapl;

exit_code stapl_main(int argc, char** argv)
{
  //
  // Initialize a 5D multiarray with deterministic values.
  //
  auto func = [](int i, int j, int k, int a, int b)
                { return i-2*j+3*k-4*a+5*b; };

  multiarray<5, int> vma(make_tuple(8, 8, 8, 8, 8), 3);

  auto vvw = make_multiarray_view(vma);

  do_once([&vvw, func]() {
    for (int i=0;i<8;++i)
      for (int j=0;j<8;++j)
        for (int k=0;k<8;++k)
          for (int a=0;a<8;++a)
            for (int b=0;b<8;++b)
              vvw(i,j,k,a,b) = func(i,j,k,a,b);
  });


  //
  // 2D->2D->1D->value, contiguous.
  //
  auto v1 = make_slices_view<0, 1, 2, 3>(vvw);
  auto v2 = make_slices_view<0, 1>(v1);

  do_once([&v2, func]()
  {
    std::cout << "Testing 2D->2D->1D->value, contiguous...";
    bool b_equal = true;

    for (int i=0;i<8;++i)
      for (int j=0;j<8;++j)
        for (int k=0;k<8;++k)
          for (int a=0;a<8;++a)
            for (int b=0;b<8;++b)
              if (v2(i,j)(k,a)[b] != func(i,j,k,a,b))
                b_equal = false;

    if (b_equal)
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  });


  //
  // 3D->1D->1D->value, contiguous.
  //
  auto v3 = make_slices_view<0, 1, 2, 3>(vvw);
  auto v4 = make_slices_view<0, 1, 2>(v3);

  do_once([&v4, func]()
  {
    std::cout << "Testing 3D->1D->1D->value, contiguous...";
    bool b_equal = true;

    for (int i=0;i<8;++i)
      for (int j=0;j<8;++j)
        for (int k=0;k<8;++k)
          for (int a=0;a<8;++a)
            for (int b=0;b<8;++b)
              if (v4(i,j,k)[a][b] != func(i,j,k,a,b))
                b_equal = false;

    if (b_equal)
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  });


  //
  // 3D->1D->1D->value, non contiguous.
  //
  auto v5 = make_slices_view<0, 1, 2, 4>(vvw);
  auto v6 = make_slices_view<0, 1, 2>(v5);

  do_once([&v6, func]()
  {
    std::cout << "Testing 3D->1D->1D->value, non contiguous...";
    bool b_equal = true;

    for (int i=0;i<8;++i)
      for (int j=0;j<8;++j)
        for (int k=0;k<8;++k)
          for (int a=0;a<8;++a)
            for (int b=0;b<8;++b)
              if (v6(i,j,k)[b][a] != func(i,j,k,a,b))
                b_equal = false;

    if (b_equal)
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  });


  //
  // 1D->1D->3D->value, non contiguous.
  //
  auto v7 = make_slices_view<3,4>(vvw);
  auto v8 = make_slices_view<1>(v7);

  do_once([&v8, func]()
  {
    std::cout << "Testing 1D->1D->3D->value, non contiguous...";
    bool b_equal = true;

    for (int i=0;i<8;++i)
      for (int j=0;j<8;++j)
        for (int k=0;k<8;++k)
          for (int a=0;a<8;++a)
            for (int b=0;b<8;++b)
              if (v8[b][a](i,j,k) != func(i,j,k,a,b))
                b_equal = false;

    if (b_equal)
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  });

  return EXIT_SUCCESS;
}
