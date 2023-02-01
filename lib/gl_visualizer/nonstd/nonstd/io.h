#ifndef NONSTD_IO_H_
#define NONSTD_IO_H_

#include <array>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "nonstd/exception.h"


namespace nonstd {

  ///@name IO
  ///@{

  /// Read a file line-by-line.
  /// @param _filename The name of the file to read.
  /// @return A vector holding the lines of the file.
  std::vector<std::string>
  read_file_lines(
      const std::string& _filename
  );


  /// Read a file into a single string.
  /// @param _filename The name of the file to read.
  /// @return A string holding the file contents.
  std::string
  read_file(
      const std::string& _filename
  );


  /// Get the match for a bracket symbol.
  /// @param _bracket The bracket symbol to match.
  /// @return The matching bracket facing the other way.
  char
  matching_bracket(
      const char _bracket
  );


  /// Print the contents of a container to a std::string.
  /// @param _begin The container's begin iterator.
  /// @param _end   The container's end iterator.
  /// @param _bracket The enclosing brackets to use from ' ', '(', '{', '[', '<',
  ///                 (space for none).
  /// @param _delimiter The delimiter to use between elements (space for none).
  /// @return A string representation of _c's contents.
  template <typename IteratorType>
  std::string
  print_container(
      IteratorType _begin,
      IteratorType _end,
      const char _bracket = '{',
      const char _delimiter = ','
  ) {
    std::ostringstream os;

    // Print opening bracket.
    if(_bracket != ' ')
      os << _bracket;

    // Create the delimiter string.
    std::string delimiter(1, _delimiter);
    if(_delimiter != ' ')
      delimiter += ' ';

    // Print the container contents.
    if(_begin != _end)
    {
      auto j = _begin,
           i = j++;
      for(; j != _end; ++i, ++j)
        os << *i << delimiter;
      os << *i;
    }

    // Print the closing bracket.
    if(_bracket != ' ')
      os << matching_bracket(_bracket);

    return os.str();
  }


  /// Print the contents of a container to a std::string. The container must
  /// support forward iterators.
  /// @param _c The container to print.
  /// @param _bracket The enclosing brackets to use from ' ', '(', '{', '[', '<',
  ///                 (space for none).
  /// @param _delimiter The delimiter to use between elements (space for none).
  /// @return A string representation of _c's contents.
  template <typename ContainerType>
  inline
  std::string
  print_container(
      const ContainerType& _c,
      const char _bracket = '{',
      const char _delimiter = ','
  ) {
    return nonstd::print_container(_c.begin(), _c.end(), _bracket, _delimiter);
  }


  /// Read the contents of an input stream into a container.
  /// @param _is The input stream.
  /// @param _iter An iterator where the incoming elements should be written.
  /// @param _count The number of elements to read, or 0 to read until the end
  ///               bracket or fail.
  /// @param _bracket The enclosing brackets from ' ', '(', '{', '[', '<',
  ///                 (space for none).
  /// @param _delimiter The delimiter between elements (space for none).
  template <typename ElementType, typename IteratorType>
  void
  input_container(
      std::istream& _is,
      IteratorType _iter,
      const size_t _count = 0,
      const char _bracket = '{',
      const char _delimiter = ','
  ) {
    // Store the exception state of the istream and change it to throw exceptions
    // for bad and failed extraction.
    auto original_exceptions = _is.exceptions();
    constexpr std::ios_base::iostate use_exceptions = std::ios_base::badbit
                                                    | std::ios_base::failbit;
    _is.exceptions(use_exceptions);

    char c;
    ElementType buffer;

    const bool using_delimiter = _delimiter != ' ',
               using_bracket   = _bracket != ' ';

    const char close_bracket = using_bracket ? matching_bracket(_bracket) : ' ';

    try
    {
      // Read the opening bracket.
      if(using_bracket)
      {
        _is >> std::ws >> c;
        if(c != _bracket)
          throw nonstd::exception(WHERE) << "Expected opening bracket '"
                                         << _bracket
                                         << "' not present (read '" << c
                                         << "' instead).";
      }

      // Read each of the values.
      for(size_t i = 1; i < _count; ++i)
      {
        _is >> std::ws >> buffer;
        *_iter = buffer;
        ++_iter;

        if(using_delimiter)
        {
          _is >> std::ws >> c;
          if(c != _delimiter)
            throw nonstd::exception(WHERE) << "Expected delimiter '"
                                           << _delimiter << "' not present "
                                           << "(read '" << c << "' instead).";
        }
      }
      _is >> std::ws >> buffer;
      *_iter = buffer;
      ++_iter;

      // If _count is 0, we have only read 1 element yet. Continue until we fail
      // to read an element.
      if(_count == 0)
      {
        try
        {
          while(true)
          {
            if(using_delimiter)
            {
              _is >> std::ws >> c;
              if(c != _delimiter)
              {
                _is.unget();
                throw std::exception();
              }
            }
            _is >> std::ws >> buffer;
            *_iter = buffer;
            ++_iter;
          }
        }
        catch(...) { }
      }

      // Read the closing bracket.
      if(using_bracket)
      {
        _is >> std::ws >> c;
        if(c != close_bracket)
          throw nonstd::exception(WHERE) << "Expected closing bracket '"
                                         << close_bracket
                                         << "' not present (read '" << c
                                         << "' instead).";
      }
    }
    catch(const nonstd::exception& _e)
    {
      _is.exceptions(original_exceptions);
      throw _e;
    }
    catch(const std::exception& _e)
    {
      _is.exceptions(original_exceptions);
      throw nonstd::exception(WHERE) << "Failed to read container data.";
    }

    _is.exceptions(original_exceptions);
  }

  ///@}

}


/*----------------------- overloads for stl containers -----------------------*/

namespace std {

  template <typename T, typename U>
  inline
  ostream&
  operator<<(ostream& _os, const pair<T, U>& _p)
  {
    return _os << "(" << _p.first << ", " << _p.second << ")";
  }


  template <typename T, typename U>
  inline
  istream&
  operator>>(istream& _is, pair<T, U>& _p)
  {
    char c;
    _is >> std::ws >> c;
    if(c != '(')
      throw nonstd::exception(WHERE) << "Expected '(' not present.";

    _is >> std::ws >> _p.first
        >> std::ws >> c;
    if(c != ',')
      throw nonstd::exception(WHERE) << "Expected ',' not present.";

    _is >> std::ws >> _p.second
        >> std::ws >> c;
    if(c != ')')
      throw nonstd::exception(WHERE) << "Expected ')' not present.";

    return _is;
  }


  template <typename... T>
  inline
  ostream&
  operator<<(ostream& _os, const list<T...>& _l)
  {
    return _os << nonstd::print_container(_l);
  }


  template <typename T, typename... U>
  inline
  istream&
  operator>>(istream& _is, list<T, U...>& _l)
  {
    nonstd::input_container<T>(_is, std::back_inserter(_l));
    return _is;
  }


  template <typename... T>
  inline
  ostream&
  operator<<(ostream& _os, const map<T...>& _m)
  {
    return _os << nonstd::print_container(_m);
  }


  template <typename T, typename... U>
  inline
  istream&
  operator>>(istream& _is, map<T, U...>& _m)
  {
    nonstd::input_container<T>(_is, std::back_inserter(_m));
    return _is;
  }


  template <typename... T>
  inline
  ostream&
  operator<<(ostream& _os, const unordered_map<T...>& _m)
  {
    return _os << nonstd::print_container(_m);
  }


  template <typename T, typename... U>
  inline
  istream&
  operator>>(istream& _is, unordered_map<T, U...>& _m)
  {
    nonstd::input_container<T>(_is, std::back_inserter(_m));
    return _is;
  }


  template <typename... T>
  inline
  ostream&
  operator<<(ostream& _os, const set<T...>& _s)
  {
    return _os << nonstd::print_container(_s);
  }


  template <typename T, typename... U>
  inline
  istream&
  operator>>(istream& _is, set<T, U...>& _s)
  {
    nonstd::input_container<T>(_is, std::back_inserter(_s));
    return _is;
  }


  template <typename... T>
  inline
  ostream&
  operator<<(ostream& _os, const unordered_set<T...>& _s)
  {
    return _os << nonstd::print_container(_s);
  }


  template <typename T, typename... U>
  inline
  istream&
  operator>>(istream& _is, unordered_set<T, U...>& _s)
  {
    nonstd::input_container<T>(_is, std::back_inserter(_s));
    return _is;
  }


  template <typename... T>
  inline
  ostream&
  operator<<(ostream& _os, const vector<T...>& _v)
  {
    return _os << nonstd::print_container(_v);
  }


  template <typename T, typename... U>
  inline
  istream&
  operator>>(istream& _is, vector<T, U...>& _v)
  {
    nonstd::input_container<T>(_is, std::back_inserter(_v));
    return _is;
  }


  template <typename T, size_t N>
  inline
  ostream&
  operator<<(ostream& _os, const array<T, N>& _a)
  {
    return _os << nonstd::print_container(_a);
  }


  template <typename T, size_t N>
  inline
  istream&
  operator>>(istream& _is, array<T, N>& _a)
  {
    nonstd::input_container<T>(_is, _a.begin(), N);
    return _is;
  }

}

/*----------------------------------------------------------------------------*/

#endif
