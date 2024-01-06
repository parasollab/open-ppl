#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/binary_tree_network.hpp>

stapl::exit_code stapl_main(int argc, char* argv[])
{
  //! [Example]
  using graph_type = stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES>;
  using view_type = stapl::graph_view<graph_type>;

  auto v = stapl::generators::make_binary_tree_network<view_type>(16);
  //! [Example]

  std::cout << v.size() << std::endl;

  return 0;
}
