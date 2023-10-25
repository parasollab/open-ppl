/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_IMPORT_TYPES_HPP
#define STAPL_UTILITY_IMPORT_TYPES_HPP

// Macro to allow importing types from another type.
// For example instead of writing:
//   typedef typename base_type::value_type value_type;
//   typedef typename base_type::reference  reference;
///
// STAPL_IMPORT_TYPES_2(typename base_type, value_type, reference).
//
#define STAPL_IMPORT_TYPE(B, T) \
  typedef B::T T;

#define STAPL_IMPORT_TYPES_1(B, T, ...)\
  typedef B::T T;

#define STAPL_IMPORT_TYPES_2(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_1(B, __VA_ARGS__)

#define STAPL_IMPORT_TYPES_3(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_2(B, __VA_ARGS__)

#define STAPL_IMPORT_TYPES_4(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_3(B, __VA_ARGS__)

#define STAPL_IMPORT_TYPES_5(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_4(B, __VA_ARGS__)

#define STAPL_IMPORT_TYPES_6(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_5(B, __VA_ARGS__)

#define STAPL_IMPORT_TYPES_7(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_6(B, __VA_ARGS__)

#define STAPL_IMPORT_TYPES_8(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_7(B, __VA_ARGS__)

#define STAPL_IMPORT_TYPES_9(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_8(B, __VA_ARGS__)

#define STAPL_IMPORT_TYPES_10(B, T, ...)\
  typedef B::T T;\
  STAPL_IMPORT_TYPES_9(B, __VA_ARGS__)


#define STAPL_IMPORT_DTYPE(B, T) \
  typedef typename B::T T;

#define STAPL_IMPORT_DTYPES_1(B, T, ...)\
  typedef typename B::T T;

#define STAPL_IMPORT_DTYPES_2(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_1(B, __VA_ARGS__)

#define STAPL_IMPORT_DTYPES_3(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_2(B, __VA_ARGS__)

#define STAPL_IMPORT_DTYPES_4(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_3(B, __VA_ARGS__)

#define STAPL_IMPORT_DTYPES_5(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_4(B, __VA_ARGS__)

#define STAPL_IMPORT_DTYPES_6(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_5(B, __VA_ARGS__)

#define STAPL_IMPORT_DTYPES_7(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_6(B, __VA_ARGS__)

#define STAPL_IMPORT_DTYPES_8(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_7(B, __VA_ARGS__)

#define STAPL_IMPORT_DTYPES_9(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_8(B, __VA_ARGS__)

#define STAPL_IMPORT_DTYPES_10(B, T, ...)\
  typedef typename B::T T;\
  STAPL_IMPORT_DTYPES_9(B, __VA_ARGS__)

#endif
