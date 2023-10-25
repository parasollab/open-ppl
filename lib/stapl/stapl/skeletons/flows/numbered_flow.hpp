/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_NUMBERED_FLOW_HPP
#define STAPL_SKELETONS_NUMBERED_FLOW_HPP

namespace stapl {
namespace skeletons {
namespace flows {

//////////////////////////////////////////////////////////////////////
/// @brief Numbering the flows allows the user to identify each flow
/// and perform the appropriate operations based on the given number
///
/// @ingroup skeletonsFlows
//////////////////////////////////////////////////////////////////////
template <std::size_t N, typename Flow>
class numbered_flow
  : public Flow
{
public:
  explicit numbered_flow(Flow const& flow)
    : Flow(flow)
  { }

  template <int M>
  explicit numbered_flow(numbered_flow<M, Flow> const& flow)
    : Flow(static_cast<Flow const&>(flow))
  { }


  template <typename Coord>
  std::size_t consumer_count(Coord const& producer_coord) const
  {
    return get_flow().template consumer_count<N>(producer_coord);
  }

private:
  Flow get_flow() const
  {
    return static_cast<Flow const&>(*this);
  }

};

template <std::size_t N, typename Flow>
numbered_flow<N, Flow>
make_numbered_flow(Flow const& flow)
{
  return numbered_flow<N, Flow>(flow);
}

} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_NUMBERED_FLOW_HPP
