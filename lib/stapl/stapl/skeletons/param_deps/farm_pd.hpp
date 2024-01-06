#ifndef STAPL_SKELETONS_PARAM_DEPS_FARM_PD_HPP
#define STAPL_SKELETONS_PARAM_DEPS_FARM_PD_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>
#include <stapl/skeletons/param_deps/utility.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Wraps the user provided generator for a farm and passes
/// the farm instance upon invocation to the generator.
///
/// @tparam Generator user-provided generator for a farm that creates
///                   the initial seeds for the farm.
//////////////////////////////////////////////////////////////////////
template <typename Generator>
class farm_init
{
private:
  Generator m_generator;

public:
  using result_type = void;

  explicit farm_init(Generator generator)
    : m_generator(std::move(generator))
  { }

  template <typename Farm, typename... Args>
  void operator()(Farm&& farm, Args&&... args)
  {
    m_generator(farm, std::forward<Args>(args)...);
  }

  void define_type(typer& t)
  {
    t.member(m_generator);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Wraps the environment used in the current spawning process
/// and provides farm functionality to the user workfunctions.
///
/// @tparam Env the environment which is used in the current spawning
///             process. This can be any skeleton environment that
///             supports
//////////////////////////////////////////////////////////////////////
template <typename Env>
class farm_stub
{
private:
  Env m_env;

public:
  using result_type = void;

  explicit farm_stub(Env env)
    : m_env(std::move(env))
  { }

  farm_stub(farm_stub const&) = default;
  farm_stub(farm_stub&&) = default;

  template <typename Op, typename Item, typename Arg,
            typename... Args>
  void add(Op&& op, Item&& item, Arg& arg, Args&&... args)
  {
    m_env.spawn_farm_element(
      op,
      reflexive_input(farm_stub<Env>(m_env)),
      view_element_input(&item.first, item.second),
      reflexive_input(&arg),
      constant_input(args)...
    );
  }

  void define_type(typer& t)
  {
    t.member(m_env);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This parametric dependency is used in @c farms skeleton
/// to generate the initial seeds of computation.
///
/// This parametric dependency is similar to the zip parametric dependency,
/// except it sends a @c farm_stub to the user-provided generator as well.
///
/// Example - the inputs to a spawned element created by this skeleton
/// would be:
/// @li farm_stub - which allows adding more work to the farm
/// @li in<0>[idx]
/// @li in<1>[idx]
/// @li in<2>[idx]
/// @li ...
///
/// @tparam Op the workfunction to be applied on each element
/// @tparam i  the number of input flows
/// @see zip
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename Generator>
class farm_pd
  : public param_deps_defaults
{
public:
  static constexpr std::size_t in_port_size = 1;
  static constexpr std::size_t op_arity     = 1;

  using op_type      = farm_init<Generator>;

  op_type m_op;

  explicit farm_pd(Generator generator)
    : m_op(std::move(generator))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief If coord is <idx, ...> it wraps the @c Op with the
  /// following inputs and sends it to the visitor along with the
  /// @c m_op
  /// @li in<0>[idx]
  /// @li in<1>[idx]
  /// @li ...
  ///
  /// @param coord        <i, j, k, ...> where i < n, j < m, k < p
  /// @param visitor      the information about Op and input is passed
  ///                     so that later this information can be converted
  ///                     to a node in the dependence graph
  /// @param in_flow      a tuple of input flows to consume from
  //////////////////////////////////////////////////////////////////////
  template <typename Coord, typename Visitor, typename In>
  void case_of(Coord const& /*skeleton_size*/, Coord const& coord,
               Visitor& visitor, In&& in_flow) const
  {
    apply_case_of(tuple_ops::front(coord), visitor, std::forward<In>(in_flow),
                  stapl::make_index_sequence<
                    stapl::tuple_size<typename std::decay<In>::type>::value>());
  }

private:
  template <typename Index, typename Visitor, typename In,
            std::size_t... Indices>
  void apply_case_of(Index&& index,
                     Visitor& visitor, In&& in_flow,
                     /* ignore the counting view */
                     index_sequence<0, Indices...>&&) const
  {
    using env_type = decltype(visitor.get_env());
    visitor.template operator()<false>(
      m_op,
      no_mapper(),
      reflexive_input(farm_stub<env_type>(visitor.get_env())),
      stapl::get<Indices>(in_flow).consume_from(
        make_tuple(std::forward<Index>(index))
      )...
    );
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief determines how many of the instances of this parametric
  /// dependency will be consuming from a producer with a given coordinate.
  /// This is a reverse query as compared to case_of
  ///
  /// @tparam FlowIndex the flow index to which this request is sent
  //////////////////////////////////////////////////////////////////////
  template <typename Size, typename Coord, typename FlowIndex>
  std::size_t consumer_count(Size const&  /*skeleton_size*/,
                             Coord const& /*producer_coord*/,
                             FlowIndex) const
  {
    return 1;
  }

  op_type get_op() const
  {
    return m_op;
  }

  void define_type(typer& t)
  {
    t.member(m_op);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a farm parametric dependency given a @c generator.
///
/// The given generator is passed a farm instance using which it can
/// add the initial seeds of the computation.
///
/// @copybrief skeletons_impl::farm_pd
///
/// @ingroup skeletonsParamDeps
//////////////////////////////////////////////////////////////////////
template <typename Generator>
skeletons_impl::farm_pd<Generator>
farm_pd(Generator const& generator)
{
  return skeletons_impl::farm_pd<Generator>(generator);
}

} // namespace skeletons
} // namespace stapl


#endif // STAPL_SKELETONS_PARAM_DEPS_FARM_PD_HPP
