#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/watts_strogatz.hpp>

stapl::exit_code stapl_main(int argc, char* argv[])
{
  //! [Example]
  using view_type = stapl::graph_view<stapl::multidigraph<int>>;

  auto v = stapl::generators::make_watts_strogatz<view_type>(16, 8, 0.2, true);
  //! [Example]

  std::cout << v.size() << std::endl;

  return 0;
}
