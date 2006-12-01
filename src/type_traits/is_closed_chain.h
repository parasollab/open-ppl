#ifndef IS_CLOSED_CHAIN_H
#define IS_CLOSED_CHAIN_H

class Cfg_reach_cc;
#include "boost/utility/enable_if.hpp"
#include "boost/type_traits/is_same.hpp"
#include "boost/type_traits/is_base_of.hpp"
#include "boost/mpl/or.hpp"

template <typename T>
struct is_closed_chain : boost::mpl::or_<
					 boost::is_same<Cfg_reach_cc, T>,
				   	 boost::is_base_of<Cfg_reach_cc, T>
					>::type {
};

#endif
