/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VERSION_HPP
#define STAPL_VERSION_HPP

//////////////////////////////////////////////////////////////////////
/// @def STAPL_VERSION
/// @brief STAPL version number based on date of release.
///
/// -# <tt>STAPL_VERSION % 100</tt> is the day of the release.
/// -# <tt>STAPL_VERSION / 100 % 100</tt> is the month of the release.
/// -# <tt>STAPL_VERSION / 10000</tt> is the year of the release.
///
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
#define STAPL_VERSION 20140905


//////////////////////////////////////////////////////////////////////
/// @def STAPL_LIB_VERSION
/// @brief STAPL version number as a string.
///
/// @ref STAPL_LIB_VERSION is the same as @ref STAPL_VERSION in the form
/// <tt>year_month_day</tt>.
///
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
#define STAPL_LIB_VERSION "2014_09_05"

#endif
