#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/grid.hpp>

stapl::exit_code stapl_main(int argc, char* argv[])
{
  //! [Example]
  using view_type = stapl::graph_view<stapl::multidigraph<int>>;

  std::array<std::size_t, 3> dims{{8, 16, 8}};

  auto v = stapl::generators::make_grid<view_type>(dims, true);
  //! [Example]

  std::cout << v.size() << std::endl;

  return 0;
}
