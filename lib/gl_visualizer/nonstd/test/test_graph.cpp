#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include "nonstd/graph.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_graph error: ";


template <typename graph>
void
node_iterate(graph& g, const string& expected, const string& what) {
  ostringstream out;
  for(auto i = g.nodes_begin(); i != g.nodes_end(); ++i)
    out << " node '" << (*i).property() << "', " << i->out_degree() << " edges\n";
  assert_msg(out.str() == expected, er + what + ", expected:\n" + expected +
      "but got:\n" + out.str());
}


template <typename graph>
void
edge_iterate(graph& g, const string& expected, const string& what) {
  ostringstream out;
  for(auto i = g.edges_begin(); i != g.edges_end(); ++i)
    out << " edge '" << (*i).property() << "', (" << i->source()->property()
        << ", " << i->target()->property() << ")\n";
  assert_msg(out.str() == expected, er + what + ", expected:\n" + expected +
      "but got:\n" + out.str());
}


template <typename graph>
void
reverse_edge_iterate(graph& g, const string& expected) {
  ostringstream out;

  // Check reverse non-const edge iteration.
  for(typename graph::edge_iterator i = --g.edges_end();
      i != --g.edges_begin(); --i)
    out << " edge '" << (*i).property() << "', (" << i->source()->property()
        << ", " << i->target()->property() << ")\n";
  assert_msg(out.str() == expected, er + "when testing reverse non-const edge "
      "iteration, expected:\n" + expected + "but got:\n" + out.str());

  // Check reverse const edge iteration with implicit non-const to const iter
  // conversion.
  out = ostringstream();
  for(typename graph::const_edge_iterator i = --g.edges_end();
      i != --g.edges_begin(); --i)
    out << " edge '" << (*i).property() << "', (" << i->source()->property()
        << ", " << i->target()->property() << ")\n";
  assert_msg(out.str() == expected, er + "when testing reverse const edge "
      "iteration, expected:\n" + expected + "but got:\n" + out.str());
}


int main() {
  cerr << "\ttesting graph..." << flush;

  using graph = graph_type<char, string>;
  graph g;
  const graph& G = g;
  map<char, typename graph::node*> m;

  // Exercise node creation.
  m['a'] = g.add_node('a');
  m['b'] = g.add_node('b');
  m['c'] = g.add_node('c');
  m['d'] = g.add_node('d');
  m['e'] = g.add_node('e');

  // Exercise edge creation.
  g.add_edge(m['a'], m['b'], "ab");
  g.add_edge(m['a'], m['c'], "ac");
  g.add_edge(m['b'], m['c'], "bc");
  g.add_edge(m['c'], m['d'], "cd");
  g.add_edge(m['c'], m['e'], "ce");
  auto de = g.add_edge(m['d'], m['e'], "de");

  // Check sizes.
  assert_msg(g.num_nodes() == 5, er + "when testing graph size, expected |nodes|"
      " == 5, but got |nodes| == " + to_string(g.num_nodes()) + "!");
  assert_msg(g.num_edges() == 6, er + "when testing graph size, expected |edges|"
      " == 6, but got |edges| == " + to_string(g.num_edges()) + "!");

  string expected;

  // Check node iteration.
  expected = " node 'a', 2 edges\n"
             " node 'b', 1 edges\n"
             " node 'c', 2 edges\n"
             " node 'd', 1 edges\n"
             " node 'e', 0 edges\n";
  node_iterate(g, expected, "when testing non-const node iteration");
  node_iterate(G, expected, "when testing const node iteration");

  // Check edge iteration.
  expected = " edge 'ab', (a, b)\n"
             " edge 'ac', (a, c)\n"
             " edge 'bc', (b, c)\n"
             " edge 'cd', (c, d)\n"
             " edge 'ce', (c, e)\n"
             " edge 'de', (d, e)\n";
  edge_iterate(g, expected, "when testing non-const edge iteration");
  edge_iterate(G, expected, "when testing const edge iteration");

  // Check reverse edge iteration.
  expected = " edge 'de', (d, e)\n"
             " edge 'ce', (c, e)\n"
             " edge 'cd', (c, d)\n"
             " edge 'bc', (b, c)\n"
             " edge 'ac', (a, c)\n"
             " edge 'ab', (a, b)\n";
  reverse_edge_iterate(g, expected);

  // Check delete edge.
  g.delete_edge(de);
  g.delete_edges(m['a'], m['b']);
  expected = " edge 'ac', (a, c)\n"
             " edge 'bc', (b, c)\n"
             " edge 'cd', (c, d)\n"
             " edge 'ce', (c, e)\n";
  edge_iterate(g, expected, "after deleting edges 'de', 'ab'");

  // Check delete node.
  g.delete_node(m['b']);
  g.delete_node(m['d']);
  expected = " node 'a', 1 edges\n"
             " node 'c', 1 edges\n"
             " node 'e', 0 edges\n";
  node_iterate(g, expected, "after deleting nodes 'b', 'd'");
  expected = " edge 'ac', (a, c)\n"
             " edge 'ce', (c, e)\n";
  edge_iterate(g, expected, "after deleting nodes 'b', 'd'");

  // Add more stuff for adj testing.
  m['f'] = g.add_node('f');
  m['g'] = g.add_node('g');
  m['h'] = g.add_node('h');
  g.add_edge(m['c'], m['f'], "cf");
  g.add_edge(m['c'], m['g'], "cg");
  g.add_edge(m['c'], m['h'], "ch");
  g.add_edge(m['f'], m['a'], "fa");
  g.add_edge(m['f'], m['h'], "fh");
  expected = " node 'a', 1 edges\n"
             " node 'c', 4 edges\n"
             " node 'e', 0 edges\n"
             " node 'f', 2 edges\n"
             " node 'g', 0 edges\n"
             " node 'h', 0 edges\n";
  node_iterate(g, expected, "after adding nodes 'f', 'g', 'h'");
  expected = " edge 'ac', (a, c)\n"
             " edge 'ce', (c, e)\n"
             " edge 'cf', (c, f)\n"
             " edge 'cg', (c, g)\n"
             " edge 'ch', (c, h)\n"
             " edge 'fa', (f, a)\n"
             " edge 'fh', (f, h)\n";
  edge_iterate(g, expected, "after adding edges 'cf', 'cg', 'ch', 'fa', 'fh'");

  // Check adj iteration of node 'c'.
  expected = " edge 'ce', (c, e)\n"
             " edge 'cf', (c, f)\n"
             " edge 'cg', (c, g)\n"
             " edge 'ch', (c, h)\n";
  ostringstream out;
  for(auto e : *m['c'])
    out << " edge '" << (*e).property() << "', (" << e->source()->property()
        << ", " << e->target()->property() << ")\n";
  assert_msg(out.str() == expected, er + "when traversing the adjacency list of "
      "node 'c', expected:\n" + expected + "but got:\n" + out.str());
  out = ostringstream();
  for(const auto e : *m['c'])
    out << " edge '" << (*e).property() << "', (" << e->source()->property()
        << ", " << e->target()->property() << ")\n";
  assert_msg(out.str() == expected, er + "when traversing the adjacency list of "
      "node 'c' with const iterators, expected:\n" + expected + "but got:\n" +
      out.str());

  // Check adj removal.
  m['c']->remove_edges_from(m['a']);
  m['c']->remove_edges_to(m['h']);
  expected = " edge 'ce', (c, e)\n"
             " edge 'cf', (c, f)\n"
             " edge 'cg', (c, g)\n"
             " edge 'fa', (f, a)\n"
             " edge 'fh', (f, h)\n";
  edge_iterate(g, expected, "after removing edges 'ac', 'ch'");

  // Test DFS function.
  out = ostringstream();
  auto print = [&out](graph::node* const n, graph::node* const p) {
    out << p->property() << " -> " << n->property() << endl;
  };
  g.dfs_function(m['c'], print);
  expected = "c -> c\n"
             "c -> e\n"
             "c -> f\n"
             "f -> a\n"
             "f -> h\n"
             "c -> g\n";
  assert_msg(out.str() == expected, er + "when testing dfs_function, expected:\n"
      + expected + ", but got:\n" + out.str());

  // Test BFS function.
  out = ostringstream();
  g.bfs_function(m['c'], print);
  expected = "c -> c\n"
             "c -> e\n"
             "c -> f\n"
             "c -> g\n"
             "f -> a\n"
             "f -> h\n";
  assert_msg(out.str() == expected, er + "when testing bfs_function, expected:\n"
      + expected + ", but got:\n" + out.str());

  cerr << "passed" << endl;
}
