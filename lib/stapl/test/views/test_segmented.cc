#include <stapl/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/segmented_view.hpp>

using namespace stapl;

template<typename T, std::size_t ...Indices>
typename homogeneous_tuple_type<sizeof...(Indices), T>::type
vec_to_tuple(std::vector<T> const& dims, index_sequence<Indices...>)
{
  using tuple_t = typename homogeneous_tuple_type<sizeof...(Indices), T>::type;

  return tuple_t(dims[Indices]...);
}


std::vector<int> vectorify(std::string const& s, int expected_entries)
{
  std::vector<int> dims;

  auto match_comma = [](char c) { return c == ','; };

  if (expected_entries-1 != std::count_if(s.begin(), s.end(), match_comma))
    abort("Invalid input specification");

  auto iter = s.begin();

  for (int i=0; i < expected_entries; ++i)
  {
    auto end_iter = std::find_if(iter, s.end(), match_comma);

    std::string q(iter, end_iter);

    dims.push_back(atoi(q.c_str()));

    if (end_iter != s.end())
      iter = ++end_iter;
  }

  return dims;
}


exit_code stapl_main(int argc, char** argv)
{
  constexpr size_t ndims = 3;

  using value_type = int;

  if (argc < 4)
    abort("./test_segmented problem_dims");

  std::vector<int> problem_dims     = vectorify(argv[1], ndims);
  std::vector<int> inner_dims       = vectorify(argv[2], ndims);
  std::vector<int> outer_dims       = vectorify(argv[3], ndims);

  do_once([&]
  {
    std::cout << "segmented_view:\n";

    std::cout << "  problem_dims = ";
    for (auto dim : problem_dims)
      std::cout << dim << " ";
    std::cout << "\n";

    std::cout << "  inner dims   = ";
    for (auto dim : inner_dims)
      std::cout << dim << " ";
    std::cout << "\n";

    std::cout << "  outer dims   = ";
    for (auto dim : outer_dims)
      std::cout << dim << " ";
    std::cout << "\n";
  });

  //
  // Container and View Initialization
  //
  using container_t = multiarray<ndims, double>;
  using view_t      = multiarray_view<container_t>;
  auto dims         = vec_to_tuple(problem_dims, make_index_sequence<ndims>());

  container_t ct(dims);
  view_t      vw(ct);

  using tuple_t          = homogeneous_tuple_type<ndims, size_t>::type;
  using traversal_t      = typename default_traversal<ndims>::type;
  using linear_t         = nd_linearize<tuple_t, traversal_t>;

  linear_t linearizer(dims);

  using traversal_t      = typename default_traversal<ndims>::type;
  using partition_t      = multiarray_impl::block_partition<traversal_t>;
  using domain_t         = typename partition_t::value_type;

  auto seg_dims1 = vec_to_tuple(inner_dims, make_index_sequence<ndims>());
  auto seg_dims2 = vec_to_tuple(outer_dims, make_index_sequence<ndims>());

  partition_t seg_part1(domain_t(dims), seg_dims1);

  auto seg_view1 = make_segmented_view(vw, seg_part1);

  partition_t seg_part2(seg_part1.domain(), seg_dims2);

  auto seg_view2 = make_segmented_view(seg_view1, seg_part2);

  for (int i=0; i<get<0>(seg_dims2); ++i)
    for (int j=0; j<get<1>(seg_dims2); ++j)
      for (int k=0; k<get<2>(seg_dims2); ++k)
  {
    auto v1       = seg_view2(i,j,k);
    auto v1_first = v1.domain().first();
    auto v1_last  = v1.domain().last();

    for (size_t i1=get<0>(v1_first); i1<=get<0>(v1_last); ++i1)
      for (size_t j1=get<1>(v1_first); j1<=get<1>(v1_last); ++j1)
        for (size_t k1=get<2>(v1_first); k1<=get<2>(v1_last); ++k1)
        {
          auto v2       = v1(i1, j1, k1);
          auto v2_first = v2.domain().first();
          auto v2_last  = v2.domain().last();

          for (size_t i2=get<0>(v2_first); i2<=get<0>(v2_last); ++i2)
            for (size_t j2=get<1>(v2_first); j2<=get<1>(v2_last); ++j2)
              for (size_t k2=get<2>(v2_first); k2<=get<2>(v2_last); ++k2)
                v2(i2, j2, k2) = linearizer(i2, j2, k2);
        }
  }

  bool b_passed = true;

  for (int i=0; i<get<0>(dims); ++i)
    for (int j=0; j<get<1>(dims); ++j)
      for (int k=0; k<get<2>(dims); ++k)
        if (ct(i,j,k) != linearizer(i,j,k))
          b_passed = false;

  do_once([b_passed]()
  {
    if (b_passed)
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  });

  return EXIT_SUCCESS;
}
