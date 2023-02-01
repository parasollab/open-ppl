/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_COMMAND_LINE_APP_HPP
#define STAPL_BENCHMARKS_GAP_COMMAND_LINE_APP_HPP

#include <benchmarks/graph/g500/g500.h>
#include <stapl/containers/graph/short_csr_graph.hpp>
#include <stapl/containers/graph/csr_utils.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/sharded_graph_io.hpp>
#include <stapl/containers/graph/generators/erdos_renyi.hpp>
#include <stapl/containers/graph/generators/watts_strogatz.hpp>
#include <stapl/containers/graph/algorithms/create_level_machine.hpp>
#include <stapl/containers/graph/algorithms/create_level_hubs.hpp>
#include <stapl/containers/graph/algorithms/execution_policy.hpp>
#include <stapl/runtime.hpp>

#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>

#include <boost/program_options.hpp>

#include "../../utilities/scoped_counter_logger.hpp"


namespace po = boost::program_options;


//////////////////////////////////////////////////////////////////////
/// @brief Work function to set edge properties to a random int between 0-9
//////////////////////////////////////////////////////////////////////
class set_edge_property
{
public:
  using result_type = void;

  template<typename V>
  void operator()(V v) const
  {
    std::mt19937 gen(v.descriptor());
    std::uniform_int_distribution<> dis(1, 255);

    for (auto e : v)
      e.property() = dis(gen);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to set edge properties of a graph
//////////////////////////////////////////////////////////////////////
template<typename Property>
struct set_edge_properties
{
  template<typename View>
  static void apply(View vw)
  {
    map_func(set_edge_property{}, vw);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when a graph has no edge property
//////////////////////////////////////////////////////////////////////
template<>
struct set_edge_properties<stapl::properties::no_property>
{
  template<typename View>
  static void apply(View vw)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction for Kronecker graph generation
//////////////////////////////////////////////////////////////////////
template<typename T>
struct reflect_graph_type
{
  using graph_t = T;
};

//////////////////////////////////////////////////////////////////////
/// @brief Enum for the graph storage model
//////////////////////////////////////////////////////////////////////
enum class graph_model
{
  csr, adj
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the type of a graph for the benchmark
///        based on the graph model.
//////////////////////////////////////////////////////////////////////
template<typename VP, typename EP, graph_model Model>
struct compute_graph_type
{
  using type = stapl::small_short_csr_graph<stapl::UNDIRECTED, VP, EP>;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for adjacency list.
//////////////////////////////////////////////////////////////////////
template<typename VP, typename EP>
struct compute_graph_type<VP, EP, graph_model::adj>
{
  using type = stapl::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES, VP, EP>;
};

//////////////////////////////////////////////////////////////////////
/// @brief Try to parse a command-line argument from a variables map
///        and abort if the argument is not found.
/// @param arg The argument to parse (e.g., "help" or "filename")
/// @param vars A map of variables from program_options
//////////////////////////////////////////////////////////////////////
template<typename T>
T try_parse_arg(std::string const& arg, po::variables_map const& vars)
{
  try {
    return vars[arg].template as<T>();
  } catch (std::exception e) {
    stapl::do_once([&](){
      std::cerr << "Cannot parse required parameter " << arg << std::endl;
    });

    stapl::abort("Error processing command line inputs");
  }

  return T{};
}

//////////////////////////////////////////////////////////////////////
/// @brief Options class when a benchmark doesn't need any arguments
///        besides the provided defaults.
//////////////////////////////////////////////////////////////////////
class no_options
{
public:
  void opts(po::options_description& desc)
  { }

  void parse(po::variables_map const& vars)
  { }
};

//////////////////////////////////////////////////////////////////////
/// @brief An object that stores arguments that have been passed in
///        from the command line.
///
/// @tparam Options A variadic pack of options, where an option class
///                 describes the command line options that are expected
///                 for a given benchmark.
//////////////////////////////////////////////////////////////////////
template<typename... Options>
class command_line_app
  : public Options...
{
protected:
  po::options_description m_desc;
  po::variables_map m_vars;

  using expand_t = int[];
  using logger_type = scoped_counter_logger<
    stapl::counter<stapl::default_timer>
  >;


public:
  command_line_app()
    : m_desc("Options")
  {
    m_desc.add_options()
      ("help,h", "Print help messages")
      ("trials", po::value<std::size_t>()->required(), "Number of trials")
      ("paradigm", po::value<std::string>()->required(),
        "The paradigm to execute the graph algorithm (lsync,async,kla,hier")
      ("k", po::value<std::size_t>()->default_value(1), "Level of asynchrony")
      ("hub_degree", po::value<std::size_t>(),
        "Degree after which a vertex is considered a hub for h_hub_paradigm")
      ("type,t", po::value<std::string>()->required(),
       "Type of the input graph (er|nsel|mtx|el|sadj)")
      ("file,f", po::value<std::string>(), "Name of the input file")
      ("prob", po::value<double>(), "Probablity for Erdos-Renyi generator")
      ("n", po::value<std::size_t>(), "Size for Erdos-Renyi generator")
      ("scale", po::value<std::size_t>(), "Scale for Kronecker graph")
      ("ef", po::value<std::size_t>(), "Edge-factor for Kronecker graph");

    (void) expand_t{ (Options::opts(m_desc), 0)... };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Parse command-line arguments.
  //////////////////////////////////////////////////////////////////////
  void operator()(const int argc, char* const argv[])
  {
    try {
      po::store(
        po::command_line_parser(argc, argv).options(m_desc).run(), m_vars
      );

      if (m_vars.count("help")) {
        std::cout << m_desc << std::endl;
        exit(1);
      }

      po::notify(m_vars);
    } catch(std::exception& e) {
      stapl::do_once([&](){
        std::cerr << "Command-line error: " << e.what() << std::endl;
      });

      stapl::abort("Error processing command line inputs");
    }

    // Call parse on other options
    (void) expand_t{ (Options::parse(m_vars), 0)... };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of trials for the benchmark
  //////////////////////////////////////////////////////////////////////
  std::size_t trials() const
  {
    return m_vars["trials"].template as<std::size_t>();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The string name of the paradigm
  //////////////////////////////////////////////////////////////////////
  std::string paradigm() const
  {
    return m_vars["paradigm"].template as<std::string>();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Level of asynchrony for the benchmark
  //////////////////////////////////////////////////////////////////////
  std::size_t k() const
  {
    return m_vars["k"].template as<std::size_t>();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assert that the paradigm specified from the command line
  ///        is compatible with this benchmark.
  //////////////////////////////////////////////////////////////////////
  template<typename... String>
  void supported_paradigms(String&&... paradigms)
  {
    std::initializer_list<std::string> il{paradigms...};
    if (std::find(il.begin(), il.end(), this->paradigm()) == il.end())
      stapl::abort("Execution policy not supported for this benchmark");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a graph_view of a graph that has been specified according
  ///        to command-line arguments.
  ///
  /// @return View of appropriate graph
  //////////////////////////////////////////////////////////////////////
  template<typename Property,
           typename EdgeProperty = stapl::properties::no_property,
           graph_model Model = graph_model::csr>
  stapl::graph_view<
    typename compute_graph_type<Property, EdgeProperty, Model>::type
  >
  create_graph() const
  {
    logger_type logger("Created graph");

    const std::string type = m_vars["type"].template as<std::string>();

    using graph_type = typename compute_graph_type<
      Property, EdgeProperty, Model
    >::type;
    using view_type = stapl::graph_view<graph_type>;

    std::unordered_map<std::string, std::function<view_type()>> dispatcher;

    // Sharded adjacency list reader
    dispatcher["sadj"] = [&]() {
      const auto filename = try_parse_arg<std::string>("file", m_vars);
      return stapl::sharded_graph_reader<graph_type>(
        filename, stapl::read_adj_list_line()
      );
    };

    // Matrix market reader
    dispatcher["mtx"] = [&]() {
      const auto filename = try_parse_arg<std::string>("file", m_vars);
      return stapl::read_matrix_market<graph_type>(filename);
    };

    // Edge-list reader
    dispatcher["el"] = [&]() {
      const auto filename = try_parse_arg<std::string>("file", m_vars);
      return stapl::read_edge_list<graph_type>(filename);
    };

    // Nested sharded edge-list reader
    dispatcher["nsel"] = [&]() {
      const auto filename = try_parse_arg<std::string>("file", m_vars);
      return stapl::nested_sharded_graph_reader<graph_type>(
        filename, stapl::read_edge_list_line()
      );
    };

    // Erodos-Renyi
    dispatcher["er"] = [&]() {
      const std::size_t n = try_parse_arg<std::size_t>("n", m_vars);
      const double prob = try_parse_arg<double>("prob", m_vars);

      return stapl::generators::make_erdos_renyi<view_type>(n, prob);
    };

    // Kronecker (Graph500)
    dispatcher["kron"] = [&]() {
      const std::size_t scale = try_parse_arg<std::size_t>("scale", m_vars);
      const std::size_t ef = try_parse_arg<std::size_t>("ef", m_vars);

      const std::size_t n = std::pow(2.0, static_cast<double>(scale));

      return graph500_benchmark_generator<reflect_graph_type<graph_type>>(
        n, ef, scale
      );
    };

    // Newman-Watts-Strogatz
    dispatcher["ws"] = [&]() {
      const std::size_t n = try_parse_arg<std::size_t>("n", m_vars);
      const std::size_t ef = try_parse_arg<std::size_t>("ef", m_vars);
      const double prob = try_parse_arg<double>("prob", m_vars);

      return stapl::generators::make_newman_watts_strogatz<view_type>(
        n, ef, prob
      );
    };

    // Handle incorrect types
    if (dispatcher.count(type) == 0) {
      std::cerr << "Type of graph should be one of ";
      for (auto const& x : dispatcher)
        std::cerr << x.first << " ";
      std::cerr << std::endl;
      exit(1);
    }

    // Choose reader based on user input of type and return
    // created graph view
    auto vw = dispatcher[type]();

    try_commit(vw.container());

    // Set random edge properties
    set_edge_properties<EdgeProperty>::apply(vw);

    return vw;
  }

  template<typename View>
  stapl::sgl::execution_policy<View> execution_policy(View& vw) const
  {
    logger_type log{"Set up policy"};

    if (this->paradigm() == "hier") {
      auto h = create_level_machine_helper(vw);
      return stapl::sgl::hierarchical_policy<View>{h};
    }
    else if (this->paradigm() == "hubs") {
      const std::size_t hub_degree =
        try_parse_arg<std::size_t>("hub_degree", m_vars);

      auto hubs = create_level_hubs_helper(vw, hub_degree);
      auto h = create_level_machine_helper(vw);
      return stapl::sgl::hierarchical_hubs_policy<View>{h, hubs, hub_degree};
    }
    else if (this->paradigm() == "lsync") {
      return stapl::sgl::level_sync_policy{};
    }
    else if (this->paradigm() == "async") {
      return stapl::sgl::async_policy{};
    }
    else if (this->paradigm() == "kla") {
      const std::size_t k = try_parse_arg<std::size_t>("k", m_vars);
      return stapl::sgl::kla_policy{k};
    }
    else {
      std::cerr << "Unknown paradigm type " << this->paradigm() << std::endl;
      stapl::abort(false);

      return stapl::sgl::level_sync_policy{};
    }
  }
};

#endif
