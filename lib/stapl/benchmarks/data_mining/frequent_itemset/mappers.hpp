#include <stapl/containers/mapping/mapper.hpp>

template <typename ContainerGID>
class mapper_functor
{

private:
  stapl::mapper<size_t> m_mapper;
  typedef size_t cid_type;
  typedef typename stapl::indexed_domain<long unsigned int> map_dom_t;
public:
  typedef stapl::location_type index_type;
  typedef cid_type   gid_type;


  mapper_functor()
    : m_mapper(map_dom_t(stapl::get_num_locations()))
  { }

  index_type operator()(cid_type const& cid) const
  {
    return m_mapper.map(cid);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_mapper);
  }

  void update(std::vector<std::tuple<std::pair<gid_type,gid_type>,
              unsigned long int, stapl::location_type>> const &, size_t) { }
};

template <typename GIDType>
class GID_to_PID
{
private:
  stapl::array<stapl::arbitrary_partition_info> & m_part_info;

public:
  typedef GIDType gid_type;
  typedef size_t index_type;

  GID_to_PID( stapl::array<stapl::arbitrary_partition_info> & part_info )
      : m_part_info(part_info)
  { }

  index_type operator()(gid_type const& gid) const
  {
    index_type result = 0;

    size_t value = gid;
    size_t lo = 0;
    size_t hi = m_part_info.size()-1;

    while ( lo <= hi ) {
      int mid = (lo + hi) / 2;
      stapl::arbitrary_partition_info info = m_part_info[mid];
      pair<size_t,size_t> dom_pair = info.domain();
      size_t begin = dom_pair.first;
      size_t end = dom_pair.second;
      if ( value >= begin && value <= end ) {
         result = mid;
         break;
      }
      if ( value > end ) {
        lo = mid+1;
      } else if ( value < begin ) {
        hi = mid-1;
      } else {
        // assert(0);
        break;
      }
    }
    return result;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_part_info);
  }

  void update(std::vector<std::tuple<std::pair<gid_type,gid_type>,
              unsigned long int, stapl::location_type>> const &, size_t) { }
};

typedef GID_to_PID<ulong> GID_to_PID_tp;
typedef mapper_functor<ulong> PID_to_LID_tp;
