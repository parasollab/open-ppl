/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/enum.hpp>
#include <boost/tuple/tuple.hpp>

#define CONT_level_open(z,n,text) text<

#define CONT_level_close(z,n,text) >

#define TEXT(z, n, text) text

#define COMPOSE_CONTAINERS_type(n,container,T)  \
  BOOST_PP_REPEAT(n,CONT_level_open,container)  \
  T                                             \
  BOOST_PP_REPEAT(n,CONT_level_close,container)

#define COMPOSE_CONTAINERS_cntr(n,var,size) \
  var(boost::make_tuple( BOOST_PP_ENUM(n,TEXT,size) ))

#define STR1(x) #x

#define STR(x) STR1(x)

#define TEST_msg(n,container,T) \
  STR(COMPOSE_CONTAINERS_type(n,container,T))

