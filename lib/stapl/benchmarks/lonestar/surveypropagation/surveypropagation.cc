/*
// Copyright (c) 2000-2013, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//Code in part based off of the Galois benchmark
//created at The University of Texas at Austin

#include <stapl/runtime.hpp>
#include <stapl/runtime/location_specific_storage.hpp>

//global timers
stapl::location_specific_storage<stapl::counter<stapl::default_timer> >
  gt1_store;

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/views/repeated_view.hpp>

#include <cstdlib>
#include <cmath>
#include <vector>
#include <cstdio>

using namespace stapl;

static const double epsilon = 0.000001;
static const int tmax = 100;

//update_eta timers
stapl::location_specific_storage<stapl::counter<stapl::default_timer> >
  ut1_store, ut2_store, ut3_store, ut4_store, ut5_store, ut6_store, ut7_store;

//ep_apply_async timers
stapl::location_specific_storage<stapl::counter<stapl::default_timer> >
  mt0_store, mt1_store, mt2_store, mt3_store;

//////////////////////////////////////////////////////////////////////
/// @brief Performs the epsilon test for doubles.
/// @param expression The first value to be compared.
/// @param num The second value to be compared.
/// @return bool Whether or not the values are equivalent.
//////////////////////////////////////////////////////////////////////
static bool equals_num(double expression, double num)
{
  if ((expression - num) < 0.000001 && (expression - num) > -0.000001)
    return true;
  return false;
}


//////////////////////////////////////////////////////////////////////
/// @brief Holds the shared state used in update_bias and decimate.
//////////////////////////////////////////////////////////////////////
struct counter_holder
{
private:
  ///Holds the number of nontrivial edges left
  unsigned int m_nontrivial;

  ///The maximum bias of a node on the graph
  double m_maxBias;

  ///The number of bias' computed in this iteration
  int m_numBias;

  ///The sum of all the computed biases
  double m_sumBias;

public:
  counter_holder()
    : m_nontrivial(0), m_maxBias(0.0), m_numBias(0), m_sumBias(0.0) { }

  counter_holder(unsigned int nt, double mb, int nb, double sb)
    : m_nontrivial(nt), m_maxBias(mb), m_numBias(nb), m_sumBias(sb) { }

  unsigned int nontrivial(void) const
  {
    return m_nontrivial;
  }

  unsigned int incrementNontrivial(void)
  {
    return (m_nontrivial++);
  }

  double maxBias(void) const
  {
    return m_maxBias;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks to see if the new max value is greater than the stored
  /// max value; if so, change to the new max value.
  /// @param newmax The value to check.
  /// @return double The value of m_maxBias at the of the function.
  //////////////////////////////////////////////////////////////////////
  double maxCheck(double newmax)
  {
    if (newmax > m_maxBias)
      m_maxBias = newmax;
    return m_maxBias;
  }

  int numBias(void) const
  {
    return m_numBias;
  }

  int incrementNumBias(void)
  {
    return (m_numBias++);
  }

  double sumBias(void) const
  {
    return m_sumBias;
  }

  double addSumBias(double add)
  {
    m_sumBias += add;
    return m_sumBias;
  }

  void reset(void)
  {
    m_nontrivial = 0;
    m_maxBias = 0.0;
    m_numBias = 0;
    m_sumBias = 0.0;
  }

  void define_type(typer& ty)
  {
    ty.member(m_nontrivial);
    ty.member(m_maxBias);
    ty.member(m_numBias);
    ty.member(m_sumBias);
  }
};


namespace stapl
{
  STAPL_PROXY_HEADER(counter_holder)
  {
    STAPL_PROXY_DEFINES(counter_holder)
    STAPL_PROXY_METHOD_RETURN(nontrivial, unsigned int)
    STAPL_PROXY_METHOD_RETURN(incrementNontrivial, unsigned int)
    STAPL_PROXY_METHOD_RETURN(maxBias, double)
    STAPL_PROXY_METHOD_RETURN(maxCheck, double, double)
    STAPL_PROXY_METHOD_RETURN(numBias, int)
    STAPL_PROXY_METHOD_RETURN(incrementNumBias, int)
    STAPL_PROXY_METHOD_RETURN(sumBias, double)
    STAPL_PROXY_METHOD_RETURN(addSumBias, double, double)
    STAPL_PROXY_METHOD(reset)
  };
}


//////////////////////////////////////////////////////////////////////
/// @brief A function object to merge two counter_holders by adding
/// or maxChecking their relevant values.
//////////////////////////////////////////////////////////////////////
struct holder_union
{
  typedef counter_holder result_type;

  template <typename T>
  counter_holder operator()(T value1, T value2)
  {
    counter_holder ret(value1.nontrivial() + value2.nontrivial(),
      value1.maxBias(), value1.numBias() + value2.numBias(),
      value1.sumBias() + value2.sumBias());
    ret.maxCheck(value2.maxBias());
    return ret;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Holds the edge values of eta, counter, the previous eta,
/// a temporary eta, and whether or not the variable used by the
/// clause is to be negated.
///
/// Eta is to be between 0 and 1, and is initialized randomly.
//////////////////////////////////////////////////////////////////////
struct SPEdge {
private:
  /// The eta value for an edge, randomly initialized.
  double m_eta;

  /// Whether an edge negates the value of the variable its connected to.
  bool m_isNegative;

  /// The number of work functions currently using this edge.
  int m_counter;

  /// The previous eta.
  double m_old_eta;

  /// A temporary eta.
  double m_temp_eta;

public:
  SPEdge(bool isNeg)
    :m_isNegative(isNeg)
  {
    m_eta = (double)rand() / (double)RAND_MAX;
    m_temp_eta = 1.0;
    m_counter = 0;
    m_old_eta = 0.0;
  }

  double getEta(void) const
  {
    return m_eta;
  }

  void setEta(double e)
  {
    m_eta = e;
  }

  double getTempEta(void) const
  {
    return m_temp_eta;
  }

  void setTempEta(double te)
  {
    m_temp_eta = te;
  }

  double getOldEta(void) const
  {
    return m_old_eta;
  }

  void setOldEta(double olde)
  {
    m_old_eta = olde;
  }

  void setCounter(int count)
  {
    m_counter = count;
  }

  int decrementCounter(void)
  {
    m_counter--;
    return m_counter;
  }

  bool getIsNegative(void) const
  {
    return m_isNegative;
  }

  void define_type(typer& ty)
  {
    ty.member(m_eta);
    ty.member(m_isNegative);
    ty.member(m_counter);
    ty.member(m_old_eta);
    ty.member(m_temp_eta);
  }
};


namespace stapl
{
  STAPL_PROXY_HEADER(SPEdge)
  {
    STAPL_PROXY_DEFINES(SPEdge)
    STAPL_PROXY_METHOD_RETURN(getEta, double)
    STAPL_PROXY_METHOD(setEta, double)
    STAPL_PROXY_METHOD_RETURN(getTempEta, double)
    STAPL_PROXY_METHOD(setTempEta, double)
    STAPL_PROXY_METHOD_RETURN(getOldEta, double)
    STAPL_PROXY_METHOD(setOldEta, double)
    STAPL_PROXY_METHOD_RETURN(setCounter, int)
    STAPL_PROXY_METHOD(decrementCounter, int)
    STAPL_PROXY_METHOD_RETURN(getIsNegative, bool)
  };
}

//////////////////////////////////////////////////////////////////////
/// @brief Holds the values of a vertex of the graph.
/// These can be clauses or variables.
//////////////////////////////////////////////////////////////////////
struct SPVertex
{
private:
  /// Whether or not this node is a clause.
  bool m_isClause;

  /// The id of this node.
  int m_name;

  /// Whether or not this node is solved.
  bool m_solved;

  /// The value of a variable.
  bool m_value;

  /// The number of times a clause has been looked at.
  int m_t;

  /// The bias of a variable.
  double m_Bias;

  /// Whether or not a task is working with this clause.
  bool m_update_eta_running;
public:
  SPVertex(void)
    : m_solved(false), m_value(false), m_t(0), m_update_eta_running(false) {}

  SPVertex(int n, bool b)
    :m_isClause(b), m_name(n), m_solved(false), m_value(false), m_t(0),
    m_update_eta_running(false) {}

  bool getIsClause(void) const
  {
    return m_isClause;
  }

  void setIsClause(bool c)
  {
    m_isClause = c;
  }

  int getName(void) const
  {
    return m_name;
  }

  void setName(int n)
  {
    m_name = n;
  }

  bool getSolved(void) const
  {
    return m_solved;
  }

  void setSolved(bool s)
  {
    m_solved = s;
  }

  bool getValue(void) const
  {
    return m_value;
  }

  void setValue(bool v)
  {
    m_value = v;
  }

  int getT(void) const
  {
    return m_t;
  }

  void setT(int nt)
  {
    m_t = nt;
  }

  double getBias(void) const
  {
    return m_Bias;
  }

  void setBias(double b)
  {
    m_Bias = b;
  }

  void setUpdateEtaRunning(bool uer)
  {
    m_update_eta_running = uer;
  }

  bool getUpdateEtaRunning(void) const
  {
    return m_update_eta_running;
  }

  void define_type(typer& ty)
  {
    ty.member(m_isClause);
    ty.member(m_name);
    ty.member(m_solved);
    ty.member(m_value);
    ty.member(m_t);
    ty.member(m_Bias);
    ty.member(m_update_eta_running);
  }
};


namespace stapl
{
  STAPL_PROXY_HEADER(SPVertex)
  {
    STAPL_PROXY_DEFINES(SPVertex)
    STAPL_PROXY_METHOD_RETURN(getIsClause, bool)
    STAPL_PROXY_METHOD(setIsClause, bool)
    STAPL_PROXY_METHOD_RETURN(getName, int)
    STAPL_PROXY_METHOD(setName, int)
    STAPL_PROXY_METHOD_RETURN(getSolved, bool)
    STAPL_PROXY_METHOD(setSolved, bool)
    STAPL_PROXY_METHOD_RETURN(getValue, bool)
    STAPL_PROXY_METHOD(setValue, bool)
    STAPL_PROXY_METHOD_RETURN(getT, int)
    STAPL_PROXY_METHOD(setT, int)
    STAPL_PROXY_METHOD_RETURN(getBias, double)
    STAPL_PROXY_METHOD(setBias, double)
    STAPL_PROXY_METHOD(setUpdateEtaRunning, bool)
    STAPL_PROXY_METHOD_RETURN(getUpdateEtaRunning, bool)
  };
}


//The global data definitions
typedef graph<UNDIRECTED, NONMULTIEDGES, SPVertex, SPEdge> Grid_Type;
typedef stapl::domset1D<typename Grid_Type::vertex_descriptor> gdom_type;
typedef graph_view<Grid_Type, gdom_type> graph_view_type;

//////////////////////////////////////////////////////////////////////
/// @brief Reads the input parameters.
/// @param argc The number of command-line parameters.
/// @param argv An array of C-stlye strings; the parameters.
//////////////////////////////////////////////////////////////////////
std::vector<int> read_parameters(int argc, char** argv)
{
  if (argc != 5)
  {
    if (stapl::get_location_id() == 0)
    {
      printf("Incorrect number of parameters\n");
      printf("Parameters should be:\n");
      printf("<Clauses> <Variables> <Variables per Clause> <Random Seed>\n");
    }
    exit(-1);
  }
  std::vector<int> ret;
  ret.push_back(atoi(argv[1]));
  ret.push_back(atoi(argv[2]));
  ret.push_back(atoi(argv[3]));
  ret.push_back(atoi(argv[4]));
  return ret;
}


//////////////////////////////////////////////////////////////////////
/// @brief This object is used to determine in which
/// indices variables are to be placed.
//////////////////////////////////////////////////////////////////////
struct is_var
{
  int m_clauses;
  int m_vars;
  int m_total;

  /// The ratio of nodes to variables
  double m_block_size;

  //////////////////////////////////////////////////////////////////////
  /// @brief constructor that determines the block size
  /// m_block_size is the number of clauses per variables + 1.
  /// @param m int The number of clauses.
  /// @param n int The number of variables.
  /// @param t int The total number of nodes.
  //////////////////////////////////////////////////////////////////////
  is_var(int m, int n, int t)
    : m_clauses(m), m_vars(n), m_total(t)
  {
    m_block_size = (double)(m_total)/(double)(m_vars);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief As the initializers go through the indices, this function
  /// returns whether or not that index is valid for a variable.
  //////////////////////////////////////////////////////////////////////
  bool is_it_var(int index)
  {
    if ((int)fmod((double)index, m_block_size) == 0)
      return true;
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief As the edge initializer only know how many variables they are
  /// and which n_th on it wants,
  /// this function tells it where that nth one actually is stored.
  //////////////////////////////////////////////////////////////////////
  size_t get_var_for_n(int index)
  {
    return (size_t)ceil((double)index * m_block_size);
  }

  void define_type (typer& t)
  {
    t.member(m_clauses);
    t.member(m_vars);
    t.member(m_total);
    t.member(m_block_size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This work function initializes the vertices of the graph.
//////////////////////////////////////////////////////////////////////
struct init_nodes
{
  typedef void result_type;

  is_var computer;

  init_nodes(is_var c)
    : computer(c) { }

  template <typename T,typename V>
  void operator()(T value, V index)
  {
    value.property().setIsClause(!computer.is_it_var(index));
    value.property().setName(index);
  }

  void define_type (typer& t)
  {
    t.member(computer);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This work function sets the temporary eta and pushes the
/// old value to old eta. It also changes the count to the specified value.
//////////////////////////////////////////////////////////////////////
struct set_temp_eta_wf
{
private:
  double m_new_eta;
  int m_new_count;

public:
  typedef void result_type;

  set_temp_eta_wf(double new_eta, int new_count)
    : m_new_eta(new_eta), m_new_count(new_count){}

  template <typename EdgeProperty>
  result_type operator()(EdgeProperty& ep) const
  {
    stapl::counter<stapl::default_timer>& mt0 = mt0_store.get();
    mt0.start();
    ep.setOldEta(ep.getEta());
    ep.setTempEta(m_new_eta);
    ep.setCounter(m_new_count);
    mt0.stop();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_new_eta);
    t.member(m_new_count);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This work function multiplies eta by a new eta and decrements
/// the edge counter.
//////////////////////////////////////////////////////////////////////
struct mult_temp_eta_wf_and_decrement
{
private:
  double m_new_eta;

public:
  typedef int result_type;

  mult_temp_eta_wf_and_decrement(double new_eta)
    : m_new_eta(new_eta){}

  template <typename EdgeProperty>
  result_type operator()(EdgeProperty& ep) const
  {
    stapl::counter<stapl::default_timer>& mt1 = mt1_store.get();
    mt1.start();
    ep.setTempEta(m_new_eta * ep.getTempEta());
    mt1.stop();
    return ep.decrementCounter();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_new_eta);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class holds the first two functions of the update_eta loop
/// These functions run on the location where the original clause is stored.
//////////////////////////////////////////////////////////////////////
struct update_eta_pt_one
  : public stapl::dynamic_wf
{
  typedef void result_type;

  template<typename TGV, typename vertex_reference,
    typename Grid_Type, typename Descriptor>
  void eta_for_a_i(TGV tgv, vertex_reference& original,
    Grid_Type whole, Descriptor not_to_do);

  template <typename TGV, typename T, typename W>
  void operator()(TGV tgv,T value, W whole);
};


//////////////////////////////////////////////////////////////////////
/// @brief This work function restarts the update_eta loop by adding
/// update_eta_pt_one tasks on every clause connected to this variable
/// except the original one.
/// This function runs on the location containing the variable, not the clause.
//////////////////////////////////////////////////////////////////////
struct update_eta_first_loop
  : public stapl::dynamic_wf
{
  typedef void result_type;

  /// This is a descriptor of the clause to be skipped (The original one).
  Grid_Type::vertex_descriptor dont_do_this_one;

  update_eta_first_loop(Grid_Type::vertex_descriptor original_descriptor)
    : dont_do_this_one(original_descriptor) { }

  template <typename TGV, typename T, typename W>
  void operator()(TGV tgv, T variable, W whole)
  {
    stapl::counter<stapl::default_timer>& ut6 = ut6_store.get();
    ut6.start();
    if (!variable.property().getSolved())
    {
      for (typename W::adj_edge_iterator bii = variable.begin(),
        bee = variable.end(); bii != bee; bii++)
      {
        if (dont_do_this_one != (*bii).source())
          tgv.add_task(update_eta_pt_one(), stapl::localize_ref(tgv),
            stapl::localize_ref(whole, (*bii).source()),
            stapl::localize_ref(whole));
      }
    }
    ut6.stop();
  }

  void define_type(typer& t)
  {
    t.member(dont_do_this_one);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This work function sets eta to the temporary eta and
/// returns whether or not it is different than old eta.
//////////////////////////////////////////////////////////////////////
struct is_diff_and_push_eta
{
  typedef bool result_type;

  template<typename EP>
  result_type operator()(EP edge_property) const
  {
    stapl::counter<stapl::default_timer>& mt2 = mt2_store.get();
    mt2.start();
    double olde = edge_property.getOldEta();
    edge_property.setEta(edge_property.getTempEta());
    double e = edge_property.getEta();
    mt2.stop();
    return (fabs(olde - e) > epsilon);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class holds the fourth and fifth functions
/// of the update_eta loop.
/// These functions run on the location where the original clause is stored.
///
/// @bug find_edge in graph is currently broken.
/// @remarks When it is fixed, is_diff_and_push_eta should be replaced
/// by find_edge and the relevant operations.
//////////////////////////////////////////////////////////////////////
struct fix_edge
  : public stapl::dynamic_wf
{
  typedef void result_type;

  typename Grid_Type::vertex_descriptor clause_descriptor;
  typename Grid_Type::vertex_descriptor variable_descriptor;
  double set_to;

  fix_edge(typename Grid_Type::vertex_descriptor clause,
    typename Grid_Type::vertex_descriptor variable, double answer)
    : clause_descriptor(clause), variable_descriptor(variable),
      set_to(answer) { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Runs if_diff_and_push_eta on the relevant edge
  /// and, if the value is different and t < 100, adds
  /// update_eta_first_loop on the variable location.
  /// @param tgv The task graph view.
  /// @param original_clause The descriptor of the original clause.
  /// @param whole A view of the whole graph.
  /// @return void.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV, typename T, typename W>
  void update_eta_pt_two(TGV tgv, T original_clause, W whole)
  {
    stapl::counter<stapl::default_timer>& ut5 = ut5_store.get();
    stapl::counter<stapl::default_timer>& mt3 = mt3_store.get();
    ut5.start();
    //FIXME
    //2/25/13 - find_edge currently has a bug, so the method is_diff was used.
    //find_edge is the better solution.
    //This bug also relates to graph edge_iterators
    typename Grid_Type::edge_iterator iii;

    typename Grid_Type::vertex_iterator source_iterator;
    whole.container().find_edge(Grid_Type::edge_descriptor(clause_descriptor,
      variable_descriptor), source_iterator, iii);
    double olde = (*iii).property().getOldEta();
    (*iii).property().setEta((*iii).property().getTempEta());
    double e = (*iii).property().getEta();
    bool val = (fabs(olde - e) > epsilon);

    if (val && (original_clause.property().getT() < 100))
    {
      tgv.add_task(update_eta_first_loop(original_clause.descriptor()),
        stapl::localize_ref(tgv),
        stapl::localize_ref(whole, variable_descriptor),
        stapl::localize_ref(whole));
    }

    mt3.start();
    original_clause.property().setUpdateEtaRunning(false);
    mt3.stop();
    ut5.stop();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the changes from update_eta_second_loop and, if it is the
  /// last one on the edge being run, runs update_eta_pt_two.
  /// @param tgv The task graph view.
  /// @param original_clause The descriptor of the original clause.
  /// @param whole A view of the whole graph.
  /// @return void.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV, typename T, typename W>
  void operator()(TGV tgv, T clause, W whole)
  {
    stapl::counter<stapl::default_timer>& ut4 = ut4_store.get();
    ut4.start();
    int count = whole.ep_apply(
      typename Grid_Type::edge_descriptor(clause_descriptor,
      variable_descriptor), mult_temp_eta_wf_and_decrement(set_to));
    whole.ep_apply_async(
      typename Grid_Type::edge_descriptor(variable_descriptor,
      clause_descriptor), mult_temp_eta_wf_and_decrement(set_to));
    ut4.stop();
    if (count == 0)
    {
      update_eta_pt_two(tgv, clause, whole);
    }
  }

  void define_type(typer& t)
  {
    t.member(clause_descriptor);
    t.member(variable_descriptor);
    t.member(set_to);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This work function computes a new eta based on the other clauses
/// connected to this variable, and calls fix_edge to apply che changes.
/// It runs on the variable's location.
//////////////////////////////////////////////////////////////////////
struct update_eta_second_loop
  : public stapl::dynamic_wf
{
  typedef void result_type;

  /// Check to not get values from this node.
  typename Grid_Type::vertex_descriptor check_against_this;

  /// Whether or not the edge is a negator.
  bool edge_is_negative;

  /// Set the end values on this node's location.
  typename Grid_Type::vertex_descriptor set_to_this;

  update_eta_second_loop(typename Grid_Type::vertex_descriptor original,
    bool was_edge_negative, typename Grid_Type::vertex_descriptor set_variable)
    : check_against_this(original), edge_is_negative(was_edge_negative),
      set_to_this(set_variable) { }

  template<typename TGV, typename T, typename W>
  void operator()(TGV tgv, T variable, W whole)
  {
    stapl::counter<stapl::default_timer>& ut3 = ut3_store.get();
    ut3.start();
    double local_eta_new = 1.0;
    if (!variable.property().getSolved())
    {
      double prod0 = 1.0;
      double prodN = 1.0;
      double prodP = 1.0;
      for (typename Grid_Type::vertex_reference::adj_edge_iterator bii =
        variable.begin(), bee = variable.end(); bii != bee; ++bii)
      {
        SPEdge Ebj = (*bii).property();
        double localEta = Ebj.getEta();
        if ((*bii).source() != check_against_this)
          prod0 *= (1.0 - localEta);
        if (Ebj.getIsNegative())
          prodN *= (1.0 - localEta);
        else
          prodP *= (1.0 - localEta);
      }
      double PIu, PIs;
      if (edge_is_negative)
      {
        PIu = (1.0 - prodN) * prodP;
        PIs = (1.0 - prodP) * prodN;
      }
      else
      {
        PIs = (1.0 - prodN) * prodP;
        PIu = (1.0 - prodP) * prodN;
      }
      double PI0 = prod0;
      local_eta_new *= (PIu / (PIu + PIs + PI0));
    }
    tgv.add_task(fix_edge(check_against_this, set_to_this, local_eta_new),
      stapl::localize_ref(tgv), stapl::localize_ref(whole, check_against_this),
      stapl::localize_ref(whole));
    ut3.stop();
  }

  void define_type(typer& t)
  {
    t.member(check_against_this);
    t.member(edge_is_negative);
    t.member(set_to_this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Adds the update_eta_second loop task on the appropriate variables'
/// locations.
/// @param tgv The task graph view.
/// @param original The descriptor of the original clause.
/// @param whole A view of the whole graph.
/// @param not_to_do Descriptor The descriptor of the variable not to add
/// a task to.
/// @return void.
//////////////////////////////////////////////////////////////////////
template<typename TGV, typename vertex_reference, typename Grid_Type,
  typename Descriptor>
void update_eta_pt_one::eta_for_a_i(TGV tgv, vertex_reference& original,
  Grid_Type whole, Descriptor not_to_do)
{
  stapl::counter<stapl::default_timer>& ut2 = ut2_store.get();
  ut2.start();
  for (typename Grid_Type::vertex_reference::adj_edge_iterator jii =
    original.begin(), jee = original.end(); jii != jee; jii++)
  {
    if ((*jii).target() != not_to_do)
    {
      tgv.add_task(update_eta_second_loop(original.descriptor(),
        (*jii).property().getIsNegative(), not_to_do),
        stapl::localize_ref(tgv), stapl::localize_ref(whole, (*jii).target()),
        stapl::localize_ref(whole));
    }
  }
  ut2.stop();
}


//////////////////////////////////////////////////////////////////////
/// @brief The start of the update_eta loop, checking to make sure no other
/// task is in the loop, and adding eta_for_a_i.
/// @param tgv TGV The task graph view.
/// @param original_clause T The descriptor of the original clause.
/// @param whole W A view of the whole graph.
/// @return void.
//////////////////////////////////////////////////////////////////////
template <typename TGV, typename T, typename W>
void update_eta_pt_one::operator()(TGV tgv,T original_clause, W whole)
{
  stapl::counter<stapl::default_timer>& ut1 = ut1_store.get();
  ut1.start();
  if (original_clause.property().getIsClause() &&
    !original_clause.property().getSolved())
  {
    if (original_clause.property().getUpdateEtaRunning())
    {
      printf("Cloned\n");
      tgv.add_task(update_eta_pt_one(), stapl::localize_ref(tgv),
        stapl::localize_ref(whole, original_clause.descriptor()),
        stapl::localize_ref(whole));
      ut1.stop();
    }
    else
    {
      original_clause.property().setUpdateEtaRunning(true);
      original_clause.property().setT(original_clause.property().getT()+1);
      ut1.stop();
      for (typename T::adj_edge_iterator iii = original_clause.begin(),
        iee = original_clause.end(); iii != iee; iii++)
      {
        ut1.start();
        //Updates both edges, as undirected graphs make two, one each way
        whole.ep_apply_async(
          typename Grid_Type::edge_descriptor(original_clause.descriptor(),
          (*iii).target()), set_temp_eta_wf(1.0, original_clause.size() - 1));
        whole.ep_apply_async(
          typename Grid_Type::edge_descriptor((*iii).target(),
          original_clause.descriptor()),
          set_temp_eta_wf(1.0, original_clause.size() - 1));
        ut1.stop();
        eta_for_a_i(tgv, original_clause, whole, (*iii).target());
      }
    }
  }
  else
    ut1.stop();
}


//////////////////////////////////////////////////////////////////////
/// @brief This work function adds update_eta_pt_one to every node.
/// This exists to allow us to time PARAGRAPH creation.
//////////////////////////////////////////////////////////////////////
struct update_eta_start
  : public stapl::dynamic_wf
{
  typedef void return_type;

  template <typename TGV, typename T, typename W>
  return_type operator()(TGV const& tgv, T grid, W whole)
  {
    stapl::counter<stapl::default_timer>& ut7 = ut7_store.get();
    ut7.start();
    // Loop over all source cells
    //   source cell iterator => sc_it
    typename T::iterator sc_it = grid.begin();
    typename T::iterator end = grid.end();
    for ( ; sc_it!=end; ++sc_it )
    {
       tgv.add_task(update_eta_pt_one(), stapl::localize_ref(tgv),
         stapl::localize_ref(whole, (*sc_it).descriptor()),
         stapl::localize_ref(whole));
    }
    ut7.stop();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This work function computes the bias on a node and returns
/// a counter_holder of the new information.
//////////////////////////////////////////////////////////////////////
struct update_bias
{
  typedef counter_holder result_type;

  template <typename T>
  counter_holder operator()(T value)
  {
    counter_holder ret;
    if (value.property().getIsClause())
    {
      return ret;
    }
    if (value.property().getSolved())
    {
      return ret;
    }

    double pp1 = 1.0;
    double pp2 = 1.0;
    double pn1 = 1.0;
    double pn2 = 1.0;
    double p0 = 1.0;

    //for each function a
    if (value.size() == 0)
      return ret;
    for (Grid_Type::vertex_reference::adj_edge_iterator aii = value.begin(),
      aee = value.end(); aii != aee; ++aii)
    {
      SPEdge aie = (*aii).property();

      double etaai = aie.getEta();
      if (!std::isnan(etaai))
      {
        if (etaai > epsilon)
        {
          ret.incrementNontrivial();
        }
        if (aie.getIsNegative())
        {
          pp2 *= (1.0 - etaai);
          pn1 *= (1.0 - etaai);
        }
        else
        {
          pp1 *= (1.0 - etaai);
          pn2 *= (1.0 - etaai);
        }
        p0 *= (1.0 - etaai);
      }
      else
      {
        printf("NAN!\n");
      }
    }
    double pp = (1.0 - pp1) * pp2;
    double pn = (1.0 - pn1) * pn2;

    double BiasP = pp / (pp + pn + p0);
    double BiasN = pn / (pp + pn + p0);

    double d = BiasP - BiasN;
    if (d < 0.0)
    {
      d = BiasN - BiasP;
    }

    value.property().setBias(d);
    value.property().setValue(BiasP > BiasN);
    if (!(equals_num(pp, 0.0) && equals_num(pn, 0.0) && equals_num(p0, 0.0)))
    {
      assert(!std::isnan(d) && !std::isnan(-d));
      ret.maxCheck(d);
      ret.addSumBias(d);
    }
    ret.incrementNumBias();
    return ret;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This work function sets the value and solved of a clause to true.
//////////////////////////////////////////////////////////////////////
struct set_solved_value
{
  typedef void result_type;

  template <typename VP>
  void operator()(VP clause_property) const
  {
    clause_property.setSolved(true);
    clause_property.setValue(true);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief This work function does the following:
/// If a variable's bias is over the calculated limit,
/// then it is set to one value, and the requisite changes and fixings are made
/// to all of the variables attached to its clauses.
//////////////////////////////////////////////////////////////////////
struct fix_variables
{
  typedef void result_type;
  double limit;

  fix_variables(double d) :limit(d) {}

  template <typename T, typename W>
  void operator()(T value, W whole)
  {
    if (!value.property().getIsClause())
    {
      if ((value.property().getBias() >= limit) &&
        !(value.property().getSolved()))
      {
        value.property().setSolved(true);
        for (Grid_Type::vertex_reference::adj_edge_iterator bii = value.begin(),
          bee = value.end(); bii != bee; ++bii)
        {
          whole.vp_apply_async((*bii).source(), set_solved_value());
        }
      }
    }
  }

  void define_type (typer& t)
  {
    t.member(limit);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Executes the decimate portion of the algorithm.
/// @param whole graph_view<Grid_Type>& A view of the whole graph.
/// @param theholder counter_holder& The combined counter_holder.
//////////////////////////////////////////////////////////////////////
void decimate(graph_view<Grid_Type>& whole, counter_holder& theholder)
{
  double average = theholder.numBias() > 0
    ? theholder.sumBias() / theholder.numBias() : 0.0;
  if (stapl::get_location_id() == 0)
  {
    printf("NonTrivial Edges: %u MaxBias %f Average Bias %f\n",
      theholder.nontrivial(), theholder.maxBias(), average);
  }
  double d = ((theholder.maxBias() - average) * 0.25) + average;
  map_func(fix_variables(d), whole, make_repeat_view(whole));
}


//////////////////////////////////////////////////////////////////////
/// @brief This work function returns 1 if a variable is solved, elso 0.
//////////////////////////////////////////////////////////////////////
struct count_fixed
{
  typedef int result_type;
  template <typename T>
  int operator()(T value)
  {
    if (value.property().getIsClause())
      return 0;
    if (value.property().getSolved())
      return 1;
    return 0;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  //Global timers
  extern stapl::location_specific_storage
    <stapl::counter<stapl::default_timer> >
    gt2_store, gt4_store, gt5_store, gt6_store, gt7_store;

  stapl::counter<stapl::default_timer>& ut1 = ut1_store.get();
  stapl::counter<stapl::default_timer>& ut2 = ut2_store.get();
  stapl::counter<stapl::default_timer>& ut3 = ut3_store.get();
  stapl::counter<stapl::default_timer>& ut4 = ut4_store.get();
  stapl::counter<stapl::default_timer>& ut5 = ut5_store.get();
  stapl::counter<stapl::default_timer>& ut6 = ut6_store.get();
  stapl::counter<stapl::default_timer>& ut7 = ut7_store.get();
  stapl::counter<stapl::default_timer>& mt0 = mt0_store.get();
  stapl::counter<stapl::default_timer>& mt1 = mt1_store.get();
  stapl::counter<stapl::default_timer>& mt2 = mt2_store.get();
  stapl::counter<stapl::default_timer>& mt3 = mt3_store.get();
  stapl::counter<stapl::default_timer>& gt1 = gt1_store.get();
  stapl::counter<stapl::default_timer>& gt2 = gt2_store.get();
  //stapl::counter<stapl::default_timer>& gt3 = gt3_store.get();
  stapl::counter<stapl::default_timer>& gt4 = gt4_store.get();
  stapl::counter<stapl::default_timer>& gt5 = gt5_store.get();
  stapl::counter<stapl::default_timer>& gt6 = gt6_store.get();
  stapl::counter<stapl::default_timer>& gt7 = gt7_store.get();

  std::vector<int> mnk = read_parameters(argc, argv);
  counter_holder my_holder;

  int tmax = 100;
  int max_repititions = 5;

  srand(mnk[4]);

  Grid_Type mygraph(mnk[0]+mnk[1]);
  //FIXME
  //As of 2/15/13, there is a bug using partial views on multiple locations
  //Until it is resolved,
  //views of the whole graph will be used with checks for isClause
  /*
  graph_view_type
    clauses(mygraph,gdom_type(0, mnk[0]-1, false, mygraph.size()));
  graph_view_type
    variables(mygraph,gdom_type(mnk[0], mnk[0]+mnk[1]-1,
    false, mygraph.size()));
  */
  graph_view<Grid_Type> wholegraph(mygraph);

  stapl::result_of::native_view<graph_view<Grid_Type> >::type
    partitioned_wholegraph = native_view(wholegraph);

  is_var my_comper(mnk[0], mnk[1], mnk[0]+mnk[1]);

  //Initialize the graph
  map_func(init_nodes(my_comper),wholegraph,
    counting_view<int>(mnk[0]+mnk[1],0));

  //Sequential initialization guarantees the same graph with
  //different numbers of locations.
  if (stapl::get_location_id() == 0)
  {
    for (int i = 0; i < mnk[0] + mnk[1]; i++)
    {
      if (!my_comper.is_it_var(i))
      {
        std::vector<int> touse;
        while (touse.size() != (unsigned)mnk[2]) {
          int newK =(int)(((double)rand()/((double)RAND_MAX + 1))
            * (double)(mnk[1]));
          if (std::find(touse.begin(), touse.end(), newK) == touse.end()) {
            touse.push_back(newK);
            wholegraph.add_edge_async(graph_view<Grid_Type>::edge_descriptor(i,
              my_comper.get_var_for_n(newK)), SPEdge((bool)(rand() % 2)));
          }
        }
      }
    }
  }

  stapl::counter<stapl::default_timer> timer;
  timer.start();
  stapl::counter<stapl::default_timer> unaccounted;

  int step = 0;
  int same_counter = 0;
  unsigned int previous_nontrivial = 0;

  stapl::counter<stapl::default_timer> t1;
  stapl::counter<stapl::default_timer> t2;
  stapl::counter<stapl::default_timer> t3;

  stapl::counter<stapl::default_timer> tb;

  bool converged = false;
  while (!converged)
  {
    //t1.start();
    //map_func(update_eta_pt_one(), wholegraph, make_repeat_view(wholegraph));
    //t1.stop();
    unaccounted.start();
    previous_nontrivial = my_holder.nontrivial();
    unaccounted.stop();

    //To test for paragraph creation time
    tb.start();
    stapl::map_func(update_eta_start(),
                    partitioned_wholegraph,stapl::make_repeat_view(wholegraph));
    tb.stop();

    t2.start();
    my_holder.reset();
    my_holder = map_reduce(update_bias(), holder_union(), wholegraph);
    t2.stop();

    t3.start();
    if (my_holder.nontrivial() > 0)
    {
      if (stapl::get_location_id() == 0)
      {
        printf("DECIMATED\n");
      }
      decimate(wholegraph, my_holder);
    }
    else
      converged = true;
    t3.stop();

    //Check for max number of steps passed
    unaccounted.start();
    step++;
    if (step > tmax)
    {
      break;
    }

    //Check to see if the same number of nontrivials is being returned
    if (previous_nontrivial == my_holder.nontrivial())
      same_counter++;
    else
      same_counter = 0;

    if (same_counter > max_repititions)
      break;
    unaccounted.stop();
  }
  timer.stop();

  if (stapl::get_location_id() == 0)
  {
    if (converged)
      printf("SIMPLIFIED\n");
    else
      printf("DID NOT CONVERGE\n");
  }

  int fixed = map_reduce(count_fixed(), stapl::plus<int>(), wholegraph);

  double accounted = t1.value() + t2.value() + t3.value() + ut1.value()
    + ut2.value() + ut3.value() + ut4.value() + ut5.value() + ut6.value()
    + ut7.value() + mt0.value() + (mt1.value()/2) + (mt2.value()/2)
    + unaccounted.value();
  double percent_unaccounted = 100 * ((timer.value()-accounted)/timer.value());
  if (stapl::get_location_id() == 0)
  {
    printf(
      "Took %f seconds\nACCOUNTED: %f\n%f%% unaccounted for\n"
      "\nExecutor: %f\ntask operator: %f\n"
      "message::construct: %f\ntask2 operator: %f\n"
      "task3: %f\ntask4: %f\n task5: %f\n\nCOMPONENTS"
      "\n----------\nRun paragraph: %f\nRandom time %f\n"
      "Fixed %d variables\nupdate_eta: %f\nupdate_bias: %f\nfix_variables: %f"
      "\neta1: %f\netaforai: %f\nloop2: %f\nfixedge: %f\neta2: %f\nloop1: %f\n"
      "coarse->fine: %f\nep_apply_in_fix_edge: %f\ntimer_count: %d\n"
      "set_temp_eta: %f\nmult_eta_and_decrement: %f\nis_diff: %f\n",
      timer.value(), accounted, percent_unaccounted,
      gt1.value(), gt2.value(), 0.0/*gt3.value()*/, gt4.value(),
      gt5.value(), gt6.value(), gt7.value(), tb.value(),
      unaccounted.value(), fixed, t1.value(), t2.value(), t3.value(),
      ut1.value(), ut2.value(), ut3.value(), ut4.value(), ut5.value(),
      ut6.value(), ut7.value(), mt3.value(), mt3.calls() , mt0.value(),
      mt1.value()/2, mt2.value()/2);
  }

  return EXIT_SUCCESS;
}
