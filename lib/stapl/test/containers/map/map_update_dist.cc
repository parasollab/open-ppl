#include <stapl/map.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/numeric.hpp>
#include <cstdlib>

struct get_first
{
  typedef int result_type;

  template <typename Element>
  result_type operator()(Element e) const
  { return e.first; }
};


struct get_second
{
  typedef int result_type;

  template <typename Element>
  result_type operator()(Element e) const
  { return e.second; }
};

typedef std::tuple<std::pair<long int, long int>, unsigned long int,
          stapl::location_type> update_elem;

struct balance_then_what
  : public stapl::balance_map<1, long int, unsigned long int>
{
private:
  typedef stapl::balance_map<1, long int, unsigned long int> base_type;

  std::vector<update_elem> m_thats_what;

public:
  typedef long int gid_type;
  typedef unsigned long int index_type;

  balance_then_what(gid_type num_gids, index_type num_blocks)
    : base_type(num_gids, num_blocks)
  { }

  index_type operator()(gid_type id) const
  {
    if (id < this->m_num_gids)
      return base_type::operator()(id);
    else
    {
      for (auto what : m_thats_what)
      {
        if (std::get<0>(what).first <= id && id <= std::get<0>(what).second)
          return std::get<1>(what);
      }
    }
    stapl::abort("I don't know what");
    return index_type();
  }

  void update(std::vector<update_elem> const& updates, size_t level)
  {
    std::copy(updates.begin(), updates.end(), std::back_inserter(m_thats_what));
  }
};

struct updateable_identity
  : public stapl::identity_map<unsigned long int,
      stapl::location_type>
{
private:
  typedef stapl::identity_map<unsigned long int, stapl::location_type>
    base_type;

  std::vector<update_elem> m_not_id;

public:
  typedef unsigned long int    gid_type;
  typedef stapl::location_type index_type;

  index_type operator()(gid_type id) const
  {
    if (m_not_id.empty())
      return id;
    else
    {
      for (auto not_id : m_not_id)
      {
        if (std::get<1>(not_id) == id)
          return std::get<2>(not_id);
      }
      return id;
    }
  }

  void update(std::vector<update_elem> const& updates, size_t level)
  {
    std::copy(updates.begin(), updates.end(), std::back_inserter(m_not_id));
  }
};

stapl::exit_code stapl_main(int argc, char** argv)
{
  typedef stapl::distribution_spec<stapl::indexed_domain<long int>>
    partitioning_view_type;
  typedef stapl::map<long int, long int, stapl::less<long int>,
            stapl::view_based_partition<partitioning_view_type>,
            stapl::view_based_mapper<partitioning_view_type> > container_t;
  typedef stapl::map_view<container_t> view_t;

  int n = 100;
  if (argc == 2)
    n = atoi(argv[1]);

  stapl::location_type lid = stapl::get_location_id();
  stapl::location_type nlocs = stapl::get_num_locations();

  container_t m(
      stapl::arbitrary(stapl::indexed_domain<long int>(n), nlocs,
        balance_then_what(n, nlocs), updateable_identity()));

  int num_big_blocks = n % nlocs;
  int block = n / nlocs + (lid < (unsigned int)num_big_blocks ? 1 : 0);
  int start = block * lid +
    (lid >= (unsigned int)num_big_blocks ? num_big_blocks : 0);
  int end   = start + block-1;

  for (int i = start; i < end+1; ++i)
  {
    if (i+1 != n)
      m[i+1] = i+1;
  }
  if (lid == nlocs-1)
  {
    update_elem ue(std::make_pair(n, n), nlocs-1, nlocs-1);
    std::vector<update_elem> update_info(1, ue);
    m.update_distribution(update_info);
  }

  // Wait on update_distribution to complete
  stapl::rmi_fence();

  if (lid == 0)
    m.insert(n, n);
  // Wait on all locations to finish populating the container
  stapl::rmi_fence();

  view_t v(m);

  int res = stapl::map_reduce(get_second(), stapl::plus<int>(), v);
  if (res == n*(n+1)/2 && !lid)
    std::cout << "Passed\n";
  else if (!lid)
    std::cout << "Failed with " << res << " expected " << n*(n+1)/2 << "\n";
  return EXIT_SUCCESS;
}
