#include <stapl/multiarray.hpp>
#include <stapl/views/segmented_view.hpp>
#include <stapl/numeric.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/containers/partitions/ndim_partition.hpp>

struct inner
{
  using result_type = void;
  template <typename Ref>
  result_type operator() (Ref&& elem)
  {
    elem = 7;
  }
};

struct outer
{
  using result_type = void;
  template <typename Seg>
  result_type operator() (Seg&& seg)
  {
    stapl::map_func<stapl::skeletons::tags::no_coarsening>(
      inner(), std::forward<Seg>(seg));
  }
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  if (argc < 7)
    return EXIT_FAILURE;

  const std::size_t m = atol(argv[1]);
  const std::size_t n = atol(argv[2]);
  const std::size_t p = atol(argv[3]);
  const std::size_t M = atol(argv[4]);
  const std::size_t N = atol(argv[5]);
  const std::size_t P = atol(argv[6]);

  using trav_tp = stapl::default_traversal<3>::type;
  using dom_1_tp = stapl::indexed_domain<size_t>;
  using part_1_tp = stapl::balanced_partition<dom_1_tp>;
  using part_tp = stapl::nd_partition<
                    stapl::tuple<part_1_tp, part_1_tp, part_1_tp>, trav_tp>;

  part_1_tp m_part(dom_1_tp(0, m-1), M);
  part_1_tp n_part(dom_1_tp(0, n-1), N);
  part_1_tp p_part(dom_1_tp(0, p-1), P);
  part_tp coarse_part(m_part, n_part, p_part);

  // Initialize all elements to 1.
  stapl::multiarray<3, int> a_ct(std::make_tuple(m, n, p), 1);
  auto a_vw = stapl::make_multiarray_view(a_ct);

  // Get initial sum
  size_t init_sum = stapl::accumulate(stapl::linear_view(a_vw), (std::size_t)0);

  using seg_vw_tp = stapl::segmented_view<decltype(a_vw), part_tp>;
  seg_vw_tp b_vw(a_vw, coarse_part);

  stapl::map_func<stapl::skeletons::tags::no_coarsening>(outer(), b_vw);

  // Get updated sum
  size_t update_sum = stapl::accumulate(stapl::linear_view(a_vw), (std::size_t)0);

  stapl::do_once([&]()
    {
      std::cerr << "Test: segmented_view_metadata\n"
                << "Version: stapl\n";

      if (init_sum != m*n*p) {
        std::cerr << "Status: FAIL\n"
                  << "Notes: multiarray initialization failed.\n";
      } else if (update_sum != 7*m*n*p) {
        std::cerr << "Status: FAIL\n"
                  << "Notes: map_func on multidimensional segmented_view failed.\n";
      } else {
        std::cerr << "Status: PASS\n";
      }
    });

  return init_sum == m*n*p && update_sum == 7*m*n*p ? EXIT_SUCCESS : EXIT_FAILURE;
}
