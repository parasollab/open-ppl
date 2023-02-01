#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PROPERTIES_PAGE_RANK_KLA_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PROPERTIES_PAGE_RANK_KLA_HPP

#include <stapl/containers/graph/graph.hpp>
#include <stapl/views/proxy_macros.hpp>

namespace stapl {

namespace properties {

//////////////////////////////////////////////////////////////////////
/// @brief Value of a rank at a specific iteration of asynchronous PageRank
//////////////////////////////////////////////////////////////////////
template<typename T>
class iteration_rank
{
public:
  using value_type = T;

  /// The number of messages received for this iteration
  std::size_t m_num_seen;
  /// The running rank of this iteration
  value_type m_rank;
  /// Whether or not the rank is ready to be sent
  bool m_ready;
  /// Whether or not the rank has been sent to all neighbors
  bool m_sent;

public:
  iteration_rank(std::size_t num_seen, value_type rank)
    : m_num_seen(num_seen), m_rank(rank), m_ready(false), m_sent(false)
  { }

  iteration_rank(void) = default;

  std::size_t num_seen() const { return m_num_seen; }
  value_type rank() const { return m_rank; }
  bool ready() const { return m_ready; }
  bool sent() const { return m_sent; }

  void increment_seen(void)
  {
    ++m_num_seen;
  }

  void add_rank(value_type rank)
  {
    m_rank += rank;
  }

  void mark_ready()
  {
    m_ready = true;
  }

  void mark_sent()
  {
    m_sent = true;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_seen);
    t.member(m_rank);
    t.member(m_ready);
    t.member(m_sent);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Property for the KLA PageRank algorithm
//////////////////////////////////////////////////////////////////////
class page_rank_kla_property
{
public:
  using value_type = double;

  /// Map from iteration to the running rank value for that iteration
  using futures_container_t =
    std::unordered_map<size_t, iteration_rank<value_type>>;

private:
  size_t m_in_degree;
  size_t m_max_iteration;
  size_t m_curr_iteration;
  long m_last_sent_iteration;
  futures_container_t m_futures;

public:
  page_rank_kla_property() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the rank of the highest iteration
  //////////////////////////////////////////////////////////////////////
  value_type rank() const
  {
    using value_type = futures_container_t::value_type;

    auto it = std::max_element(m_futures.begin(),
                               m_futures.end(),
                               [](value_type const& x, value_type const& y) {
                                 return x.first < y.first;
                               });
    return m_futures.find(it->first)->second.rank();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the rank of a specific iteration
  //////////////////////////////////////////////////////////////////////
  value_type rank(std::size_t iter) const
  {
    auto it = m_futures.find(iter);

    stapl_assert(it != m_futures.end(),
                 "Getting the rank for an iteration that does not have one");

    return it->second.rank();
  }

  size_t in_degree() const
  { return m_in_degree;  }

  void in_degree(size_t val)
  { m_in_degree = val;  }

  void increment_iteration()
  {
    ++m_curr_iteration;
  }

  void update_last_sent_iteration(long iteration)
  {
    m_last_sent_iteration = iteration;
  }

  std::size_t last_sent_iteration() const
  {
    return m_last_sent_iteration;
  }

  size_t iteration() const
  {
    return m_curr_iteration;
  }

  size_t max_iteration() const
  {
    return m_max_iteration;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a rank contribution for a specific iteration.
  /// @return If this vertex is now ready to propagate its rank after
  ///         the contribution.
  //////////////////////////////////////////////////////////////////////
  bool future_add(size_t iter, double rank)
  {
    auto it = m_futures.find(iter);
    if (it != m_futures.end()) {
      it->second.add_rank(rank);
      it->second.increment_seen();
    } else {
      m_futures.emplace(std::piecewise_construct,
                        std::forward_as_tuple(iter),
                        std::forward_as_tuple(1, rank+0.15));
    }

    auto& r = m_futures.find(iter)->second;

    // If we've received messages from all of our neighbors, mark ourself as
    // ready and update the iteration
    const bool is_ready = r.num_seen() == m_in_degree;
    if (is_ready) {
      r.mark_ready();

      m_curr_iteration = std::max(m_curr_iteration, iter);
    }

    return is_ready;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initialize the property
  //////////////////////////////////////////////////////////////////////
  void initialize_rank(double rank)
  {
    m_futures.clear();
    m_futures.emplace(std::piecewise_construct,
                      std::forward_as_tuple(0),
                      std::forward_as_tuple(0, rank));
    m_futures.find(0)->second.mark_ready();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initialize the number of iterations
  //////////////////////////////////////////////////////////////////////
  void iteration(size_t val)
  {
    m_max_iteration = val;
    m_curr_iteration = 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Mark a specific iteration as having its value sent to this
  ///        vertex's neighbors
  //////////////////////////////////////////////////////////////////////
  void mark_as_sent(std::size_t iter)
  {
    auto it = m_futures.find(iter);

    stapl_assert(it != m_futures.end(),
                 "Marking an iteration to be sent that doesn't have any value");

    it->second.mark_sent();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Whether or not the specific iteration has received contributions
  ///        from all of its neighbors
  //////////////////////////////////////////////////////////////////////
  bool is_ready(std::size_t iter) const
  {
    auto it = m_futures.find(iter);

    return it == m_futures.end() ? false : it->second.ready();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Whether or not the specific iteration has already sent its
  ///        rank to all of its neighbors
  //////////////////////////////////////////////////////////////////////
  bool was_sent(std::size_t iter) const
  {
    auto it = m_futures.find(iter);

    return it == m_futures.end() ? false : it->second.sent();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_in_degree);
    t.member(m_curr_iteration);
    t.member(m_max_iteration);
    t.member(m_last_sent_iteration);
    t.member(m_futures);
  }
};

} // namespace properties

template <typename Accessor>
class proxy<properties::page_rank_kla_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::page_rank_kla_property target_t;
  typedef properties::page_rank_kla_property::value_type value_type;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  value_type rank() const
  {
    using fn_t = value_type (target_t::*)(void) const;
    fn_t fn = &target_t::rank;

    return Accessor::const_invoke(fn);
  }

  value_type rank(std::size_t i) const
  {
    using fn_t = value_type (target_t::*)(std::size_t) const;
    fn_t fn = &target_t::rank;

    return Accessor::const_invoke(fn, i);
  }

  bool future_add(std::size_t i, value_type r)
  {
    return Accessor::invoke(&target_t::future_add, i, r);
  }

  STAPL_PROXY_METHOD_RETURN(in_degree, size_t);
  STAPL_PROXY_METHOD(in_degree, size_t);
  STAPL_PROXY_METHOD(update_last_sent_iteration, size_t);
  STAPL_PROXY_METHOD_RETURN(iteration, size_t);
  STAPL_PROXY_METHOD(iteration, size_t);
  STAPL_PROXY_METHOD_RETURN(max_iteration, size_t);
  STAPL_PROXY_METHOD_RETURN(was_sent, bool, std::size_t);
  STAPL_PROXY_METHOD_RETURN(is_ready, bool, std::size_t);
  STAPL_PROXY_METHOD_RETURN(last_sent_iteration, std::size_t);
  STAPL_PROXY_METHOD(mark_as_sent, std::size_t);
  STAPL_PROXY_METHOD(initialize_rank, value_type);

}; // class proxy

} // namespace stapl

#endif
