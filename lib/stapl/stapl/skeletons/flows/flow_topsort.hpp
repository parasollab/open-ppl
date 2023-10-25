#ifndef STAPL_SKELETONS_FLOW_TOPSORT_HPP
#define STAPL_SKELETONS_FLOW_TOPSORT_HPP

#include <stapl/skeletons/flows/inline_flows.hpp>

namespace stapl {
namespace skeletons {

namespace flows {
namespace inline_flows {

namespace detail {

// Check not(T in Visited || T is input), i.e. T still needs to be visited
template<class T, class Visited>
using needs_visit = std::integral_constant<bool,
        find_first_index<Visited, std::is_same, T>::value == -1 &&
        not placeholders::is_input<T>::value>;

// Check that a ph list doesn't have anything that needs to be visited
template<class Phs, class Visited>
using all_visited =
  std::integral_constant<bool,
      find_first_index<Phs, needs_visit, Visited>::value == -1>;


// Implementation of Kahn's algorithm over ph_flows.
template<class Visited>
struct topsort_impl;

template<class... Visited>
struct topsort_impl<tuple<Visited...>> {

  // Helper that scans through a list, moving nodes that have outstanding
  // dependencies from Queue into Seen until it finds a visitable node.
  template<class Seen, class Queue>
  struct go;

  // Nothing left we can process, either done or error
  template<class T>
  struct go<T, tuple<>> {
    static_assert(tuple_size<T>::value == 0, "Cycle in flows detected");
    using type = tuple<Visited...>;
  };

  // Process the head of the queue
  template<class... Seen, class T, class... Ts>
  struct go<tuple<Seen...>, tuple<T, Ts...>> {
    // Check that none of T's dependencies still need to be visited first
    using eligible = all_visited<typename T::input_t, tuple<Visited...>>;

    // If eligible, visit T, otherwise move along
    using next = typename std::conditional<eligible::value,
      typename topsort_impl<tuple<Visited..., typename T::output_t>>::template
         go<tuple<>, tuple<Seen..., Ts...>>,
      go<tuple<Seen..., T>, tuple<Ts...>>
    >::type;

    using type = typename next::type;
  };
};

} // namespace detail

template<class Flows>
using topsort = typename detail::topsort_impl<tuple<>>::template
  go<tuple<>, Flows>;

} // namespace inline_flows
} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif
