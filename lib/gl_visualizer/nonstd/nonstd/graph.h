#ifndef NONSTD_GRAPH_H_
#define NONSTD_GRAPH_H_

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <list>
#include <queue>
#include <string>
#include <utility>


namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// A generic adjacency-list graph.
  ///
  /// Nodes stored in a linked list. Edges are allocated from the heap, and their
  /// pointers are stored in the adjacency lists of the source and target nodes.
  //////////////////////////////////////////////////////////////////////////////
  template <typename N, typename E = int>
  class graph_type
  {

    public:

      ///@name Local Types
      ///@{

      class node;
      class edge;

      typedef N node_property;
      typedef E edge_property;

      typedef std::list<node>  node_list;
      typedef std::list<edge*> adj_list;

      typedef typename node_list::iterator       node_iterator;
      typedef typename node_list::const_iterator const_node_iterator;

      typedef typename adj_list::iterator        adj_iterator;
      typedef typename adj_list::const_iterator  const_adj_iterator;

      template <typename NI, typename EI> class edge_iterator_type;

      typedef edge_iterator_type<node_iterator, adj_iterator> edge_iterator;
      typedef edge_iterator_type<const_node_iterator, const_adj_iterator>
          const_edge_iterator;

      ///@}

    private:

      ///@name Internal State
      ///@{

      node_list m_nodes;      ///< Node container.
      size_t m_num_edges{0};  ///< Current edge count.

      ///@}

    public:

      ///@name Construction
      ///@{

      graph_type();

      graph_type(const graph_type&) = delete; ///< Copy is disabled.
      graph_type(graph_type&&) = delete;      ///< Move is disabled.

      virtual ~graph_type() = default;

      ///@}
      ///@name Accessors
      ///@{

      size_t num_nodes() const noexcept;
      size_t num_edges() const noexcept;

      ///@}
      ///@name Iteration
      ///@{

      node_iterator begin() noexcept;
      node_iterator end() noexcept;
      node_iterator nodes_begin() noexcept;
      node_iterator nodes_end() noexcept;

      const_node_iterator begin() const noexcept;
      const_node_iterator end() const noexcept;
      const_node_iterator nodes_begin() const noexcept;
      const_node_iterator nodes_end() const noexcept;

      edge_iterator edges_begin() noexcept;
      edge_iterator edges_end() noexcept;

      const_edge_iterator edges_begin() const noexcept;
      const_edge_iterator edges_end() const noexcept;

      ///@}
      ///@name Node Modifiers
      ///@{

      /// Add a node to the graph.
      /// @param[in] _n The property for the new node.
      /// @return    A pointer to the new node.
      node* add_node(node_property&& _p);

      /// Remove a node from the graph.
      /// @param[in] _v The identifier of the node to remove.
      void delete_node(node* const _n);

      ///@}
      ///@name Edge Modifiers
      ///@{

      /// Add an edge between two nodes.
      /// @param[in] _s A pointer to the source node.
      /// @param[in] _t A pointer to the target node.
      /// @param[in] _p The edge property.
      edge* add_edge(node* const _s, node* const _t,
          const edge_property& _p = edge_property());

      /// Remove all edges between two nodes.
      /// @param[in] _s A pointer to the source node.
      /// @param[in] _t A pointer to the target node.
      void delete_edges(node* const _s, node* const _t);

      /// Remove an edge by handle.
      /// @param[in] _e The edge to delete.
      void delete_edge(edge* const _e);

      ///@}
      ///@name Function Traversals
      ///@{

      /// Preform a function on the nodes in depth-first order.
      /// @param[in] _root The initial node.
      /// @param[in] _f    A function to execute on the node and its parent.
      /// @param[in] _random Randomize the order in which children are visited.
      void dfs_function(node* const _root,
          std::function<void(node* const, node* const)> _f,
          const bool _random = false);

      /// Preform a function on the nodes in breadth-first order.
      /// @param[in] _root The initial node.
      /// @param[in] _f    A function to execute on the node and its parent.
      void bfs_function(node* const _root,
          std::function<void(node* const, node* const)> _f);

      ///@}
      ///@name Node, Edge, and Edge Iterator
      ///@{

      //////////////////////////////////////////////////////////////////////////
      /// Node representation includes an identifier, inbound and outbound
      /// adjacency lists, and a node_property object.
      //////////////////////////////////////////////////////////////////////////
      class node
      {

        friend class graph_type;
        friend class edge;
        friend edge_iterator;

        ///@name Internal State
        ///@{

        node_property m_property;    ///< The node property.
        adj_list      m_edges_in;    ///< The inbound adjacent edges.
        adj_list      m_edges_out;   ///< The outbound adjacent edges.
        char          m_label{' '};  ///< A label for use by graph algorithms.

        ///@}
        ///@name Node List Management
        ///@{

        char          m_sentinel{'n'}; ///< Is this a sentinel node?
        node_iterator m_iter;          ///< Location in the graph's node_list.

        ///@}

        public:

          ///@name Construction
          ///@{

          node() = default;
          node(const node_property& _p);
          node(node_property&& _p);

          node(const node& _n) = delete; ///< Copy is disabled.
          node(node&&) = delete;         ///< Move is disabled.

          ~node(); ///< Destruction also destroys attached edges.

          ///@}
          ///@name Assignment
          ///@{
          /// Assignment is disabled.

          node& operator=(const node& _n) = delete;
          node& operator=(node&& _n) = delete;

          ///@}
          ///@name Queries
          ///@{
          /// Get information about a node's property or edges.

          node_property& property() noexcept;
          const node_property& property() const noexcept;

          size_t in_degree() const noexcept;
          size_t out_degree() const noexcept;
          size_t degree() const noexcept;

          char& label() noexcept;
          const char& label() const noexcept;

          edge* get_edge_to(node* const _n) const noexcept;
          edge* get_edge_from(node* const _n) const noexcept;

          ///@}
          ///@name Iteration
          ///@{
          /// Iterate over the edge lists.

          adj_iterator begin() noexcept;
          adj_iterator end() noexcept;

          const_adj_iterator begin() const noexcept;
          const_adj_iterator end() const noexcept;

          adj_iterator in_edges_begin() noexcept;
          adj_iterator in_edges_end() noexcept;

          const_adj_iterator in_edges_begin() const noexcept;
          const_adj_iterator in_edges_end() const noexcept;

          ///@}
          ///@name Edge Modifiers
          ///@{

          /// Remove all edges between this and another node.
          /// @param[in] _n The other node to break connections with.
          void remove_edges_with(const node* const _n) noexcept;

          /// Remove all edges from this to another node.
          /// @param[in] _n The other node to break connections with.
          void remove_edges_from(const node* const _n) noexcept;

          /// Remove all edges from another node to this.
          /// @param[in] _n The other node to break connections with.
          void remove_edges_to(const node* const _n) noexcept;

          /// Remove all edges.
          void clear_edges() noexcept;

          ///@}
      };


      //////////////////////////////////////////////////////////////////////////
      /// Edge representation includes source and target pointers plus an
      /// edge_property object.
      //////////////////////////////////////////////////////////////////////////
      class edge
      {

        ///@name Internal State
        ///@{

        edge_property m_property;    ///< The edge property.
        node* const   m_source;      ///< The source node.
        node* const   m_target;      ///< The target node.
        char          m_label{' '};  ///< A label for use by graph algorithms.

        ///@}
        ///@name Adjacency List Handles
        ///@{
        /// Iterators to this edge's locations within the source and target
        /// adj_lists are stored to make finding it there O(1) time.

        adj_iterator m_source_iter;
        adj_iterator m_target_iter;

        ///@}

        public:

          ///@name Construction
          ///@{

          edge(node* const _s, node* const _t, const edge_property& _p);

          edge(const edge&) = delete; ///< Copy is disabled.
          edge(edge&&) = delete;      ///< Move is disabled.

          /// Deleting an edge also removes it from the adjacency lists of its
          /// source and target.
          ~edge();

          ///@}
          ///@name Assignment
          ///@{
          /// Assignment is disabled.

          edge& operator=(const edge& _e) = delete;
          edge& operator=(edge&& _e) = delete;

          ///@}
          ///@name Comparators
          ///@{
          /// All comparators check edge properties. Source/target vertices are
          /// not considered.

          bool operator==(const edge& _e) const noexcept;
          bool operator!=(const edge& _e) const noexcept;
          bool operator<(const edge& _e) const noexcept;
          bool operator<=(const edge& _e) const noexcept;
          bool operator>(const edge& _e) const noexcept;
          bool operator>=(const edge& _e) const noexcept;

          ///@}
          ///@name Queries
          ///@{

          edge_property& property() noexcept;
          const edge_property& property() const noexcept;

          node* source() const noexcept;
          node* target() const noexcept;

          node* opposite(node* const _o) const noexcept;

          char& label() noexcept;
          const char& label() const noexcept;

          ///@}

      };


      //////////////////////////////////////////////////////////////////////////
      /// Edge iterators traverse the outbound edges of each node in the
      /// node_list order.
      //////////////////////////////////////////////////////////////////////////
      template <typename NI, typename EI>
      class edge_iterator_type
      {

        ///@name Local Types
        ///@{

        typedef NI node_iter;
        typedef EI adj_iter;

        ///@}
        ///@name Internal State
        ///@{

        node_iter m_node; ///< The current node iterator.
        adj_iter  m_edge; ///< The current edge (adj) iterator.

        ///@}

        public:

          ///@name Construction
          ///@{

          edge_iterator_type(node_iter _n);
          edge_iterator_type(node_iter _n, adj_iter _e);

          ///@}
          ///@name Conversion to const
          ///@{

          operator const_edge_iterator() const noexcept;

          ///@}
          ///@name Interface
          ///@{

          edge& operator*() noexcept;
          const edge& operator*() const noexcept;

          edge* operator->() noexcept;
          const edge* operator->() const noexcept;

          edge_iterator_type& operator++();
          edge_iterator_type operator++(int);

          edge_iterator_type& operator--();
          edge_iterator_type operator--(int);

          ///@}
          ///@name Comparison
          ///@{

          bool operator==(const edge_iterator_type& _o) const noexcept;
          bool operator!=(const edge_iterator_type& _o) const noexcept;

          ///@}

        private:

          ///@name Helpers
          ///@{

          /// If m_edge has reached the end of m_node's adj_list, move to the
          /// next node with out-bound edges. No-op for the back sentinel.
          void validate_forward();

          /// If m_edge has reached the front of m_node's adj_list, move to the
          /// previous node with out-bound edges. No-op for the front sentinel.
          void validate_backward();

          ///@}
      };

      ///@}
  };

  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Graph ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  /*---------------------------- Construction --------------------------------*/

  template <typename N, typename E>
  graph_type<N, E>::
  graph_type()
  {
    m_nodes.emplace_front();
    m_nodes.front().m_sentinel = 'f';
    m_nodes.emplace_back();
    m_nodes.back().m_sentinel  = 'b';
  }

  /*------------------------------ Accessors ---------------------------------*/

  template <typename N, typename E>
  inline
  size_t
  graph_type<N, E>::
  num_nodes() const noexcept
  {
    return m_nodes.size() - 2;
  }


  template <typename N, typename E>
  inline
  size_t
  graph_type<N, E>::
  num_edges() const noexcept
  {
    return m_num_edges;
  }

  /*------------------------------ Iteration ---------------------------------*/

  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node_iterator
  graph_type<N, E>::
  begin() noexcept
  {
    return ++m_nodes.begin();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node_iterator
  graph_type<N, E>::
  end() noexcept
  {
    return --m_nodes.end();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node_iterator
  graph_type<N, E>::
  nodes_begin() noexcept
  {
    return begin();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node_iterator
  graph_type<N, E>::
  nodes_end() noexcept
  {
    return end();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_node_iterator
  graph_type<N, E>::
  begin() const noexcept
  {
    return ++m_nodes.begin();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_node_iterator
  graph_type<N, E>::
  end() const noexcept
  {
    return --m_nodes.end();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_node_iterator
  graph_type<N, E>::
  nodes_begin() const noexcept
  {
    return begin();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_node_iterator
  graph_type<N, E>::
  nodes_end() const noexcept
  {
    return end();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::edge_iterator
  graph_type<N, E>::
  edges_begin() noexcept
  {
    auto i = ++m_nodes.begin();
    return edge_iterator(i, i->begin());
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::edge_iterator
  graph_type<N, E>::
  edges_end() noexcept
  {
    auto i = --m_nodes.end();
    return edge_iterator(i, i->end());
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_edge_iterator
  graph_type<N, E>::
  edges_begin() const noexcept
  {
    auto i = ++m_nodes.begin();
    return const_edge_iterator(i, i->begin());
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_edge_iterator
  graph_type<N, E>::
  edges_end() const noexcept
  {
    auto i = --m_nodes.end();
    return const_edge_iterator(i, i->end());
  }

  /*-------------------------- Node Modifiers --------------------------------*/

  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node*
  graph_type<N, E>::
  add_node(node_property&& _p)
  {
    auto n = m_nodes.emplace(nodes_end(), std::move(_p));
    n->m_iter = n;
    return &*n;
  }


  template <typename N, typename E>
  inline
  void
  graph_type<N, E>::
  delete_node(node* const _n)
  {
    m_nodes.erase(_n->m_iter);
  }

  /*------------------------- Edge Modifiers ---------------------------------*/

  template <typename N, typename E>
  inline
  typename graph_type<N, E>::edge*
  graph_type<N, E>::
  add_edge(node* const _s, node* const _t, const edge_property& _p)
  {
    ++m_num_edges;
    return new edge(_s, _t, _p);
  }


  template <typename N, typename E>
  void
  graph_type<N, E>::
  delete_edges(node* const _s, node* const _t)
  {
    adj_list to_del;
    for(auto& e : _s->m_edges_out)
      if(e->target() == _t) to_del.push_back(e);
    for(auto& e : to_del) {
      delete e;
      --m_num_edges;
    }
  }


  template <typename N, typename E>
  inline
  void
  graph_type<N, E>::
  delete_edge(edge* const _e)
  {
    delete _e;
    --m_num_edges;
  }

  /*----------------------- Function Traversals ------------------------------*/

  template <typename N, typename E>
  void
  graph_type<N, E>::
  dfs_function(node* const _root,
      std::function<void(node* const, node* const)> _f, const bool _random)
  {
    // Mark all nodes unvisited.
    for(auto& n : *this)
      n.m_label = ' ';

    // Define the traversal function.
    std::function<void(node* const, node* const)> dfs =
        [&](node* const _n, node* const _p)
    {
      // Mark this node visited and execute the function.
      _n->label() = 'v';
      _f(_n, _p);

      if(!_random)
      {
        // This is an ordered traversal. Visit each edge in order.
        for(auto& e : *_n)
          if(e->target()->label() == ' ') dfs(e->target(), _n);
      }
      else
      {
        // This is a random traversal. Create a list of unvisited edges.
        std::vector<edge*> edges;
        edges.reserve(_n->out_degree());
        std::copy(_n->begin(), _n->end(), std::back_inserter(edges));

        // While we still have unvisited edges, choose a random one and visit
        // it.
        while(edges.size())
        {
          auto iter = edges.begin() + rand() % edges.size();
          edges.erase(iter);
          auto e = *iter;
          if(e->target()->label() == ' ')
            dfs(e->target(), _n);
        }
      }
    };

    // Execute the traversal from the root node.
    dfs(_root, _root);

    // Look for disconnected components that were not discovered in a prior
    // traversal and start another traversal there.
    for(auto& n : *this)
      if(n.m_label == ' ') dfs(&n, &n);
  }


  template <typename N, typename E>
  void
  graph_type<N, E>::
  bfs_function(node* const _root,
      std::function<void(node* const, node* const)> _f)
  {
    // Mark all nodes unvisited.
    for(auto& n : *this)
      n.m_label = ' ';

    // Define the traversal function.
    auto bfs = [&](node* const _n)
    {
      // Create a queue of frontier nodes and add the root to it.
      std::queue<node*> q;
      q.push(_n);

      // Execute the function on the root node.
      _f(_n, _n);

      // Keep going while we still have frontier nodes.
      while(q.size())
      {
        // Get the next node and mark it visited.
        node* n = q.front();
        q.pop();
        n->m_label = 'v';

        // Visit each adjacent node.
        for(auto& e : *n)
        {
          // If the node is unvisited, call _f on it and add it to the frontier.
          if(e->target()->m_label == ' ')
          {
            _f(e->target(), n);
            q.push(e->target());
          }
        }
      }
    };

    // Execute the traversal from the root node.
    bfs(_root);

    // Look for disconnected components that were not discovered in a prior
    // traversal and start another traversal there.
    for(auto& n : *this)
      if(n.m_label == ' ') bfs(&n);
  }

  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Node ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  /*----------------------------- Construction -------------------------------*/

  template <typename N, typename E>
  graph_type<N, E>::node::
  node(const node_property& _p) :
      m_property(_p)
  { }


  template <typename N, typename E>
  graph_type<N, E>::node::
  node(node_property&& _p) :
      m_property(std::move(_p))
  { }


  template <typename N, typename E>
  graph_type<N, E>::node::
  ~node()
  {
    clear_edges();
  }

  /*------------------------------- Queries ----------------------------------*/

  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node_property&
  graph_type<N, E>::node::
  property() noexcept
  {
    return m_property;
  }


  template <typename N, typename E>
  inline
  const typename graph_type<N, E>::node_property&
  graph_type<N, E>::node::
  property() const noexcept
  {
    return m_property;
  }


  template <typename N, typename E>
  inline
  size_t
  graph_type<N, E>::node::
  in_degree() const noexcept
  {
    return m_edges_in.size();
  }


  template <typename N, typename E>
  inline
  size_t
  graph_type<N, E>::node::
  out_degree() const noexcept
  {
    return m_edges_out.size();
  }


  template <typename N, typename E>
  inline
  size_t
  graph_type<N, E>::node::
  degree() const noexcept
  {
    return in_degree() + out_degree();
  }


  template <typename N, typename E>
  inline
  char&
  graph_type<N, E>::node::
  label() noexcept
  {
    return m_label;
  }


  template <typename N, typename E>
  inline
  const char&
  graph_type<N, E>::node::
  label() const noexcept
  {
    return m_label;
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::edge*
  graph_type<N, E>::node::
  get_edge_to(node* const _n) const noexcept
  {
    auto iter = std::find_if(m_edges_out.begin(), m_edges_out.end(),
        [_n](edge* const _e) {return _e->target() == _n;});
    return iter == m_edges_out.end() ? nullptr : *iter;
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::edge*
  graph_type<N, E>::node::
  get_edge_from(node* const _n) const noexcept
  {
    auto iter = std::find_if(m_edges_in.begin(), m_edges_in.end(),
        [_n](edge* const _e) {return _e->source() == _n;});
    return iter == m_edges_in.end() ? nullptr : *iter;
  }

  /*----------------------------- Iteration ----------------------------------*/

  template <typename N, typename E>
  inline
  typename graph_type<N, E>::adj_iterator
  graph_type<N, E>::node::
  begin() noexcept
  {
    return m_edges_out.begin();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::adj_iterator
  graph_type<N, E>::node::
  end() noexcept
  {
    return m_edges_out.end();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_adj_iterator
  graph_type<N, E>::node::
  begin() const noexcept
  {
    return m_edges_out.begin();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_adj_iterator
  graph_type<N, E>::node::
  end() const noexcept
  {
    return m_edges_out.end();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::adj_iterator
  graph_type<N, E>::node::
  in_edges_begin() noexcept
  {
    return m_edges_in.begin();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::adj_iterator
  graph_type<N, E>::node::
  in_edges_end() noexcept
  {
    return m_edges_in.end();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_adj_iterator
  graph_type<N, E>::node::
  in_edges_begin() const noexcept
  {
    return m_edges_in.begin();
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::const_adj_iterator
  graph_type<N, E>::node::
  in_edges_end() const noexcept
  {
    return m_edges_in.end();
  }

  /*------------------------- Edge Modifiers ---------------------------------*/

  template <typename N, typename E>
  inline
  void
  graph_type<N, E>::node::
  remove_edges_with(const node* const _n) noexcept
  {
    remove_edges_from(_n);
    remove_edges_to(_n);
  }


  template <typename N, typename E>
  void
  graph_type<N, E>::node::
  remove_edges_from(const node* const _n) noexcept
  {
    adj_list to_del;
    for(const auto& e : m_edges_in)
      if(e->source() == _n)
        to_del.push_back(e);
    for(auto& e : to_del)
      delete e;
  }


  template <typename N, typename E>
  void
  graph_type<N, E>::node::
  remove_edges_to(const node* const _n) noexcept
  {
    adj_list to_del;
    for(const auto& e : m_edges_out)
      if(e->target() == _n)
        to_del.push_back(e);
    for(auto& e : to_del)
      delete e;
  }


  template <typename N, typename E>
  void
  graph_type<N, E>::node::
  clear_edges() noexcept
  {
    while(!m_edges_in.empty())
      delete m_edges_in.front();
    while(!m_edges_out.empty())
      delete m_edges_out.front();
  }

  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Edge ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  /*-------------------------- Construction ----------------------------------*/

  template <typename N, typename E>
  graph_type<N, E>::edge::
  edge(node* const _s, node* const _t, const edge_property& _p)
      : m_property(_p), m_source(_s), m_target(_t)
  {
    adj_list& so = m_source->m_edges_out;
    adj_list& ti = m_target->m_edges_in;
    m_source_iter = so.insert(so.end(), this);
    m_target_iter = ti.insert(ti.end(), this);
  }


  template <typename N, typename E>
  graph_type<N, E>::edge::
  ~edge()
  {
    m_source->m_edges_out.erase(m_source_iter);
    m_target->m_edges_in.erase(m_target_iter);
  }

  /*-------------------------- Comparators -----------------------------------*/

  template <typename N, typename E>
  inline
  bool
  graph_type<N, E>::edge::
  operator==(const edge& _e) const noexcept
  {
    return m_property == _e.m_property;
  }


  template <typename N, typename E>
  inline
  bool
  graph_type<N, E>::edge::
  operator!=(const edge& _e) const noexcept
  {
    return m_property != _e.m_property;
  }


  template <typename N, typename E>
  inline
  bool
  graph_type<N, E>::edge::
  operator<(const edge& _e) const noexcept
  {
    return m_property < _e.m_property;
  }


  template <typename N, typename E>
  inline
  bool
  graph_type<N, E>::edge::
  operator<=(const edge& _e) const noexcept
  {
    return m_property <= _e.m_property;
  }


  template <typename N, typename E>
  inline
  bool
  graph_type<N, E>::edge::
  operator>(const edge& _e) const noexcept
  {
    return m_property > _e.m_property;
  }


  template <typename N, typename E>
  inline
  bool
  graph_type<N, E>::edge::
  operator>=(const edge& _e) const noexcept
  {
    return m_property >= _e.m_property;
  }

  /*---------------------------- Queries -------------------------------------*/

  template <typename N, typename E>
  inline
  typename graph_type<N, E>::edge_property&
  graph_type<N, E>::edge::
  property() noexcept
  {
    return m_property;
  }


  template <typename N, typename E>
  inline
  const typename graph_type<N, E>::edge_property&
  graph_type<N, E>::edge::
  property() const noexcept
  {
    return m_property;
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node*
  graph_type<N, E>::edge::
  source() const noexcept
  {
    return m_source;
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node*
  graph_type<N, E>::edge::
  target() const noexcept
  {
    return m_target;
  }


  template <typename N, typename E>
  inline
  typename graph_type<N, E>::node*
  graph_type<N, E>::edge::
  opposite(node* const _o) const noexcept
  {
    return _o == m_source ? m_target : m_source;
  }


  template <typename N, typename E>
  inline
  char&
  graph_type<N, E>::edge::
  label() noexcept
  {
    return m_label;
  }


  template <typename N, typename E>
  inline
  const char&
  graph_type<N, E>::edge::
  label() const noexcept
  {
    return m_label;
  }

  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~ Edge Iterator ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  /*--------------------------- Construction ---------------------------------*/

  template <typename N, typename E>
  template <typename NI, typename EI>
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  edge_iterator_type(node_iter _n)
    : m_node(_n), m_edge(_n->begin())
  {
    if(_n->m_sentinel == 'n')
      validate_forward();
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  edge_iterator_type(node_iter _n, adj_iter _e)
    : m_node(_n), m_edge(_e)
  {
    if(_n->m_sentinel == 'n')
      validate_forward();
  }

  /*------------------------- Conversion to Const ----------------------------*/

  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator const_edge_iterator() const noexcept
  {
    return const_edge_iterator(m_node, m_edge);
  }

  /*----------------------------- Interface ----------------------------------*/

  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  typename graph_type<N, E>::edge&
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator*() noexcept
  {
    return **m_edge;
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  const typename graph_type<N, E>::edge&
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator*() const noexcept
  {
    return **m_edge;
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  typename graph_type<N, E>::edge*
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator->() noexcept
  {
    return *m_edge.operator->();
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  const typename graph_type<N, E>::edge*
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator->() const noexcept
  {
    return *m_edge.operator->();
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  graph_type<N, E>::edge_iterator_type<NI, EI>&
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator++()
  {
    if(m_node->m_sentinel != 'b') {
      ++m_edge;
      validate_forward();
    }
    return *this;
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  graph_type<N, E>::edge_iterator_type<NI, EI>
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator++(int)
  {
    edge_iterator e = *this;
    ++(*this);
    return e;
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  graph_type<N, E>::edge_iterator_type<NI, EI>&
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator--()
  {
    if(m_node->m_sentinel != 'f') {
      validate_backward();
      --m_edge;
    }
    return *this;
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  graph_type<N, E>::edge_iterator_type<NI, EI>
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator--(int)
  {
    edge_iterator e = *this;
    --(*this);
    return e;
  }

  /*---------------------------- Comparison ----------------------------------*/

  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  bool
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator==(const edge_iterator_type& _o) const noexcept
  {
    return m_node == _o.m_node && m_edge == _o.m_edge;
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  inline
  bool
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  operator!=(const edge_iterator_type& _o) const noexcept
  {
    return !(*this == _o);
  }

  /*----------------------------- Helpers ------------------------------------*/

  template <typename N, typename E>
  template <typename NI, typename EI>
  void
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  validate_forward()
  {
    if(m_node->m_sentinel == 'f' ||
        (m_node->m_sentinel == 'n' && m_edge == m_node->end()) ) {
      do {
        ++m_node;
      } while(m_node->m_sentinel != 'b' && m_node->out_degree() == 0);
      m_edge = m_node->begin();
    }
  }


  template <typename N, typename E>
  template <typename NI, typename EI>
  void
  graph_type<N, E>::edge_iterator_type<NI, EI>::
  validate_backward()
  {
    if(m_node->m_sentinel == 'b' ||
        (m_node->m_sentinel == 'n' && m_edge == m_node->begin()) ) {
      do {
        --m_node;
      } while(m_node->m_sentinel != 'f' && m_node->out_degree() == 0);
      m_edge = m_node->end();
    }
  }

  /*--------------------------------------------------------------------------*/

}

#endif
