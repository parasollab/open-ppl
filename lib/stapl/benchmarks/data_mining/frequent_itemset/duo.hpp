#include <stapl/utility/hash_fwd.hpp>
template <typename T>
class duo
{
private:
  T m_first, m_second;
public:
  duo()
    : m_first(0), m_second(0)
  { }
  duo(T first, T second)
    : m_first(first), m_second(second)
  { }

  bool operator==(duo const& arg) const
  {
    return m_first == arg.m_first && m_second == arg.m_second;
  }

  bool operator<(duo const& arg) const
  {
    return m_first < arg.m_first && m_second < arg.m_second;
  }

  friend size_t hash_value(duo const& d)
  {
    size_t seed = 0;
    boost::hash_combine(seed, d.m_first);
    boost::hash_combine(seed, d.m_second);
    return seed;
  }

  T first()
  {
    return m_first;
  }
  T second()
  {
    return m_second;
  }


  void define_type(stapl::typer& t)
  {
    t.member(m_first);
    t.member(m_second);
  }
};
