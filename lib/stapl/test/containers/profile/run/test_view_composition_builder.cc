/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/vector.hpp>
#include <stapl/views/overlap_view.hpp>
#include <stapl/views/strided_view.hpp>
#include "../view_composition_builder.hpp"
#include "../../../test_report.hpp"
#include "../value_type_util.hpp"

using namespace stapl;

#include <functional>
namespace ph = std::placeholders;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  using ar_type = array<int>;
  using vec_type = vector<MVT>;

  ar_type ar(10);
  vec_type vec(10);
  bool passed = true;

  stapl::do_once([&ar, &vec]() {ar[3] = 3; ar[6] = 6; vec[3] = 3; vec[6] = 6;});
  rmi_fence();

  using av_type = array_view<ar_type>;
  auto av = view_composition<av_type>::construct(ar);

  using av_type_check = typename std::remove_pointer<decltype(av)>::type;
  static_assert(
    std::is_same<typename av_type_check::view_container_type, ar_type>::value,
    "Mismatched types for array_view<array> composition.");

  passed &= (*av)[3] == 3;

  using av2_type = array_view<array_view<ar_type>>;
  auto av2 = view_composition<av2_type>::construct(ar);

  using av2_type_check = typename std::remove_pointer<decltype(av2)>::type;
  static_assert(
    std::is_same<
      typename av2_type_check::view_container_type, av_type
    >::value
    &&
    std::is_same<
      av2_type_check::view_container_type
                    ::view_container_type, ar_type
    >::value,
    "Mismatched types for array_view<array_view<array>> composition.");

  passed &= (*av2)[3] == 3;

  rmi_fence();
  delete av2;

  using av3_type = array_view<array_view<array_view<ar_type>>>;
  auto av3 = view_composition<av3_type>::construct(new ar_type(10));

  using av3_type_check = typename std::remove_pointer<decltype(av3)>::type;
  static_assert(
    std::is_same<
      typename av3_type_check::view_container_type, av2_type
    >::value
    &&
    std::is_same<
      typename av3_type_check::view_container_type
                             ::view_container_type, av_type
    >::value
    &&
    std::is_same<
      typename av3_type_check::view_container_type
                             ::view_container_type
                             ::view_container_type, ar_type
    >::value,
    "Mismatched types for array_view<array_view<array_view<array>>> "
    "composition.");

  do_once([av3]() { (*av3)[3] = 3; });
  rmi_fence();

  passed &= (*av3)[3] == 3;

  rmi_fence();
  delete av3;

  using vv_type = vector_view<vec_type>;
  using avvv_type = array_view<vv_type>;
  auto avvv = view_composition<avvv_type>::construct(vec);

  using avvv_type_check = typename std::remove_pointer<decltype(avvv)>::type;
  static_assert(
    std::is_same<
      typename avvv_type_check::view_container_type, vv_type
    >::value
    &&
    std::is_same<
      typename avvv_type_check::view_container_type
                              ::view_container_type, vec_type
    >::value,
    "Mismatched types for vector_view<array_view<array>> composition.");

  passed &= (*avvv)[3] == 3;

  rmi_fence();
  delete avvv;

  STAPL_TEST_REPORT(passed, "Testing view_composition_builder");

  size_t c = 2, l = 2, r = 1;
  auto ovv_av_maker = std::bind(make_overlap_view<av_type>, ph::_1, c, l, r);
  auto makers1 = stapl::make_tuple(ovv_av_maker);
  auto makers2 = stapl::make_tuple(make_array_view<ar_type>, ovv_av_maker);

  functional_view_composition<av_type, decltype(makers1)> ovv_av_builder(*av);
  auto ovv_av = ovv_av_builder.create(makers1);

  passed &= ovv_av && ovv_av->size() == 5 &&
            (*ovv_av)[0].size() == c+r && (*ovv_av)[4].size() == c+l &&
            (*ovv_av)[2][(*ovv_av)[2].domain().last()] == 6 &&
            (*ovv_av)[4][(*ovv_av)[4].domain().first()] == 6;
  for (int i = 1; i <= 3; ++i)
    passed &= (*ovv_av)[i].size() == c+l+r;

  using ovv_av_type_check =
    typename std::remove_pointer<decltype(ovv_av)>::type;
  static_assert(
    std::is_same<
      typename ovv_av_type_check::view_container_type
                                ::view_container_type, av_type
    >::value,
    "Mismatched types for overlap_view<array_view<array>> composition.");

  ovv_av_builder.clear();
  ovv_av = ovv_av_builder.create(makers1);

  functional_view_composition<ar_type, decltype(makers2)> ovv_av_ar_builder(ar);
  auto ovv_av_ar = ovv_av_ar_builder.create(makers2);

  using ovv_av_ar_type_check =
    typename std::remove_pointer<decltype(ovv_av_ar)>::type;
  static_assert(
    std::is_same<
      typename ovv_av_ar_type_check::view_container_type::view_container_type,
      av_type
    >::value,
    "Mismatched types for overlap_view<array_view<array>> composition.");

  passed &= ovv_av->size() == ovv_av_ar->size();
  for (size_t i = 0; i < ovv_av->size(); ++i)
    passed &= (*ovv_av)[i].domain().first()==(*ovv_av_ar)[i].domain().first()
      && (*ovv_av)[i].domain().last()==(*ovv_av_ar)[i].domain().last();

  using sv_type = typename strided_view<av_type>::type;

  size_t step1 = 3, start1 = 0;
  size_t step2 = 2, start2 = 0;
  auto makers3 = stapl::make_tuple(
    make_array_view<ar_type>,
    std::bind(make_strided_view<av_type>, ph::_1, step1, start1),
    std::bind(make_strided_view<sv_type>, ph::_1, step2, start2)
  );

  functional_view_composition<ar_type, decltype(makers3)>
    sv_sv_av_ar_builder(ar);

  auto sv_sv_av_ar = sv_sv_av_ar_builder.create(makers3);

  using sv_sv_av_ar_type_check =
    typename std::remove_pointer<decltype(sv_sv_av_ar)>::type;
  static_assert(
    std::is_same<
      typename sv_sv_av_ar_type_check::view_container_type, ar_type
    >::value,
    "Mismatched types for strided_view<strided_view<array_view<array>>> "
    "composition.");

  passed &= (*sv_sv_av_ar)[1] == 6;

  sv_sv_av_ar = sv_sv_av_ar_builder.create(makers3);

  passed &= (*sv_sv_av_ar)[1] == 6;

  STAPL_TEST_REPORT(passed, "Testing functional_view_composition_builder");

  rmi_fence();
  delete av;

  return EXIT_SUCCESS;
}
