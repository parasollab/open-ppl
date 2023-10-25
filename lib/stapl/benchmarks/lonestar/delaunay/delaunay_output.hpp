
#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_OUTPUT_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_OUTPUT_HPP


#include <stapl/containers/graph/graph.hpp>


namespace stapl
{


namespace delaunay
{

using std::string;
using std::ofstream;



///////////////////////////////////////////////////////////////////////////////
/// @brief Overloadable workfunction for outputting the header of a dot file.
///////////////////////////////////////////////////////////////////////////////
template<typename View, typename VProp=use_default, typename EProp=use_default>
struct delaunay_output_header_wf
{
  typedef typename View::vertex_property VProp2;
  typedef typename View::edge_property   EProp2;
  typedef delaunay_output_header_wf<View, VProp2, EProp2> derived_type;
  typedef delaunay_output_header_wf<View, VProp, EProp>   this_type;

  View*    m_view;
  string   m_filename;
  ofstream m_file;

  delaunay_output_header_wf(View* view, string filename)
    : m_view(view), m_filename(filename)
  { }

  derived_type* derived()
  {
    return static_cast<derived_type*>(this);
  }

  void output_default_graph()
  {
    m_file << "\tgraph [\n";
    m_file << "\t\tsplines=line;\n";
    m_file << "\t\tnotranslate=true;\n";
    m_file << "\t\tlayout=neato;\n";
    m_file << "\t\toutputorder=nodesfirst;\n";
    m_file << "\t];\n\n";
  }

  void output_default_node()
  {
    m_file << "\tnode [\n";
    if (m_view->size() <= 1000) {
      m_file << "\t\theight=0.25;\n";
      m_file << "\t\twidth=0.25;\n";
      m_file << "\t\tshape=circle;\n";
    } else {
      m_file << "\t\theight=0.1;\n";
      m_file << "\t\twidth=0.1;\n";
      m_file << "\t\tshape=point;\n";
    }
    m_file << "\t\tfontsize=8;\n";
    m_file << "\t\tfixedsize=true;\n";
    m_file << "\t\tpin=true;\n";
    m_file << "\t];\n\n";
  }

  void output_default_edge()
  {
    m_file << "\tedge [\n";
    m_file << "\t\tpenwidth=1.5;\n";
    m_file << "\t\tarrowsize=0.5;\n";
    m_file << "\t];\n\n";
  }

  void output_color_subgraph(size_t num, string color)
  {
    m_file << "\tsubgraph loc" << num << " {\n";
    m_file << "\t\tnode [ style=filled; fillcolor=" << color << "; ];\n";
    m_file << "\t\tedge [ color=" << color << "; ];\n";
    m_file << "\t}\n\n";
  }

  void output_subgraphs()
  {
    static const char* colors[] = { "red", "gold", "blue", "orange", "green3",
       "plum", "cyan3", "gray32", "yellow", "navy", "powderblue", "deeppink",
       "springgreen", "darkolivegreen", "khaki", "purple", };
    size_t num_locations = this->m_view->get_num_locations();
    for (size_t n=0; n<num_locations; ++n) {
      size_t num = sizeof(colors) / sizeof(*colors);
      this->derived()->output_color_subgraph(n, colors[n % num]);
    }
  }

  void operator()()
  {
    // Requires [this] instead [&] to fix lambda capture bug in gcc-4.7.2.
    do_once([this]() {
      this->m_file.open(m_filename.c_str(), std::fstream::out);
      this->m_file << "graph pGraph {\n";

      this->derived()->output_default_graph();
      this->derived()->output_default_node();
      this->derived()->output_default_edge();
      this->derived()->output_subgraphs();

      this->m_file.close();
    });
  }

};


///////////////////////////////////////////////////////////////////////////////
/// @brief Overloadable workfunction for outputting each vertex.
///////////////////////////////////////////////////////////////////////////////
template<typename View, typename VProp=use_default, typename EProp=use_default>
struct delaunay_output_node_wf
{
  typedef delaunay_output_node_wf<View, VProp, EProp> this_type;
  typedef typename View::vertex_property VProp2;
  typedef typename View::edge_property   EProp2;
  typedef delaunay_output_node_wf<View, VProp2, EProp2> derived_type;

  typedef void result_type;

  View*  m_view;
  size_t m_loc;
  double m_scale;
  int    m_last_loc;
  size_t m_size;

  delaunay_output_node_wf(View* view, double scale = 200.0)
    : m_view(view), m_loc(view->get_location_id()), m_scale(scale),
      m_last_loc(-1), m_size(view->size())
  { }

  derived_type* derived()
  { return static_cast<derived_type*>(this); }

  template<typename Vertex>
  void output_subgraph_label(Vertex&& v, std::ostream& os)
  {
    if (m_last_loc == (int)m_loc) {
      return;
    } else if (m_loc != 0) {
      derived()->output_closing_label(v, os);
    }
    m_last_loc = m_loc;
    os << "\tsubgraph loc" << m_loc << " {\n";
  }

  template<typename Vertex>
  void output_closing_label(Vertex&& v, std::ostream& os)
  { os << "\t}\n"; }

  template<typename Vertex>
  void output_position(Vertex&& v, std::ostream& os)
  {
    auto p = v.property().get_point();

    double x = p.m_x / m_scale;
    double y = p.m_y / m_scale;

    os << "\t\t" << v.descriptor()
       << " [ pos = \"" << x << ", " << y << "\" ];\n";
  }


  template<typename Vertex>
  void output_node(Vertex&& v, std::ostream& os)
  {
    derived()->output_position(v, os);
  }

  template<typename Vertex>
  void operator() (Vertex&& v, std::ostream& os)
  {
    derived()->output_subgraph_label(v, os);

    derived()->output_node(v, os);

    if (v.descriptor() == m_size-1) {
      derived()->output_closing_label(v, os);
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_view);
    t.member(m_loc);
    t.member(m_scale);
    t.member(m_last_loc);
    t.member(m_size);
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Overloadable workfunction for outputting each edge.
///////////////////////////////////////////////////////////////////////////////
template<typename View, typename VProp=use_default, typename EProp=use_default>
struct delaunay_output_edge_wf
{
  typedef delaunay_output_edge_wf<View, VProp, EProp> this_type;

  typedef typename View::vertex_property VProp2;
  typedef typename View::edge_property   EProp2;
  typedef delaunay_output_edge_wf<View, VProp2, EProp2> derived_type;

  typedef void result_type;

  View*  m_view;
  size_t m_loc;
  int    m_last_loc;
  size_t m_size;

  delaunay_output_edge_wf(View* view)
    : m_view(view), m_loc(view->get_location_id()),
      m_last_loc(-1), m_size(view->size())
  { }

  derived_type* derived()
  { return static_cast<derived_type*>(this); }

  template<typename Vertex>
  void output_subgraph_label(Vertex&& v, std::ostream& os)
  {
    if (m_last_loc == (int)m_loc) {
      return;
    } else if (m_loc != 0) {
      derived()->output_closing_label(v, os);
    }
    m_last_loc = m_loc;
    os << "\tsubgraph loc" << m_loc << " {\n";
  }

  template<typename Vertex>
  void output_closing_label(Vertex&& v, std::ostream& os)
  { os << "\t}\n"; }

  template<typename Vertex, typename Edge>
  void output_edge(Vertex&& vertex, Edge&& edge, std::ostream& os)
  {
    if (edge.source() < edge.target()) {
      os << "\t\t" << edge.source() << " -- " << edge.target() << " ;\n";
    }
  }

  template<typename Vertex>
  void output_edges(Vertex&& v, std::ostream& os)
  {
    derived()->output_subgraph_label(v, os);
    for (auto aei = v.begin(); aei != v.end(); ++aei) {
      derived()->output_edge(v, *aei, os);
    }
    if (v.descriptor() == m_size-1) {
      derived()->output_closing_label(v, os);
    }
  }

  template<typename Vertex>
  void operator() (Vertex&& v, std::ostream& os)
  {
    derived()->output_edges(v, os);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_view);
    t.member(m_loc);
    t.member(m_last_loc);
    t.member(m_size);
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Workfunction that writes a delaunay_mesh_view to a dot file. The
/// workfunction makes calls to three overloadable workfunctions that can be
/// customized on a per vertex_property and/or edge_property basis. Overloading
/// a workfunction is achieved using a combination of CRTP and partial template
/// specialization. Simply implementing the specialized struct is sufficent.
///
/// Example:
/// template<typename View>
/// struct delaunay_output_node_wf<View, delaunay_vertex, delaunay_edge>
///   : delaunay_output_node_wf<View>
/// {
///   template<typename Vertex>
///   void output_node(Vertex&& v, std::ostream& os)
///   { /* Custom Output */ }
/// }
///////////////////////////////////////////////////////////////////////////////
template<typename View>
struct delaunay_output_wf
{
  typedef typename View::vertex_property VProp;
  typedef typename View::edge_property   EProp;
  typedef delaunay_output_wf<View>       this_type;

  typedef delaunay_output_header_wf<View, VProp, EProp> HeaderWF;
  typedef delaunay_output_node_wf<View, VProp, EProp>   NodeWF;
  typedef delaunay_output_edge_wf<View, VProp, EProp>   EdgeWF;

  View*    m_view;
  string   m_filename;
  HeaderWF m_header_wf;
  NodeWF   m_node_wf;
  EdgeWF   m_edge_wf;

  delaunay_output_wf(View* view, string filename)
    : m_view(view), m_filename(filename),
      m_header_wf(view, filename),
      m_node_wf(view),
      m_edge_wf(view)
  { }

  void write_to_file()
  {
    m_header_wf();

    output_graph_to_file<NodeWF> node_wf(m_filename, m_node_wf);
    serial(node_wf, stapl::native_view(*m_view));

    output_graph_to_file<EdgeWF> edge_wf(m_filename, m_edge_wf);
    serial(edge_wf, stapl::native_view(*m_view));


    do_once([&]() {
      std::ofstream ofile;
      ofile.open(m_filename.c_str(), std::fstream::out | std::fstream::app);
      ofile << "}" << std::endl;
      ofile.close();
    });
  }

};



} // namespace delaunay

} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_OUTPUT_HPP */
