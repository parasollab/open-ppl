/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Provides serialization support for libstdc++ (GNU C++) STL classes.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////

#include <cstddef>
#include <cstring>
#include <cstdint>
#include <stapl/runtime/stapl_assert.hpp>
#include <stapl/runtime/serialization/typer_traits.hpp>

#ifdef _STAPL

// do a preliminary version check here
//
# ifndef STAPL__GNUC__
#  ifdef __GNUC__
#    define STAPL__GNUC__ __GNUC__
#  endif
# endif

# ifndef STAPL__GNUC_MINOR__
#  ifdef __GNUC_MINOR__
#    define STAPL__GNUC_MINOR__ __GNUC_MINOR__
#  endif
# endif

# ifndef STAPL__GNUC_PATCHLEVEL__
#  ifdef __GNUC_PATCHLEVEL__
#   define STAPL__GNUC_PATCHLEVEL__ __GNUC_PATCHLEVEL__
#  endif
# endif

# if defined(STAPL__GNUC__) && defined(STAPL__GNUC_MINOR__)
#  if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
       (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
       (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
#  else
#   error "Only select versions of gcc 4.x, 5.x, and 6.x are supported"
#  endif
# else
#  error "Unable to determine libstdc++ version... aborting"
# endif

  // std::bitset
  #ifndef STAPL_BITSET_DEFINE_TYPE
    #if defined(_GLIBCXX_BITSET)
      #if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
           (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
           (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
        #define STAPL_BITSET_DEFINE_TYPE
        template<size_t _Nw>
        inline void std::_Base_bitset<_Nw>::define_type(stapl::typer& t)
        {
          t.member(_M_w);
        }

        inline void std::_Base_bitset<0>::define_type(stapl::typer&)
        { }

        inline void std::_Base_bitset<1>::define_type(stapl::typer& t)
        {
          t.member(_M_w);
        }

        template<size_t _Nb>
        inline void std::bitset<_Nb>::define_type(stapl::typer& t)
        {
          t.base<_Base>(*this);
        }
      #endif
    #endif
  #endif


  // std::bvector<>
  #ifndef STAPL_BVECTOR_DEFINE_TYPE
    #if defined(_STL_BVECTOR_H)
      #if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
           (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
           (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
        #define STAPL_BVECTOR_DEFINE_TYPE
        template <typename Alloc>
        inline void std::_Bvector_base<Alloc>::define_type(stapl::typer& t)
        {
          //we have to pack finish - start +
          //  0 if the offset for finish is zero
          //  1 otherwise
          int size =
            (this->_M_impl._M_finish._M_p - this->_M_impl._M_start._M_p) ;
          if (this->_M_impl._M_finish._M_offset>0) {
            ++size;
          }
          t.member(this->_M_impl._M_start._M_offset);
          t.member(this->_M_impl._M_finish._M_offset);
          t.member(this->_M_impl._M_start._M_p, size_t(size));
          if (this->_M_impl._M_finish._M_offset>0) {
            t.pointer_to_member(this->_M_impl._M_finish._M_p,
                                this->_M_impl._M_start._M_p,
                                size_t(size-1));
            t.pointer_to_member(this->_M_impl._M_end_of_storage,
                                this->_M_impl._M_start._M_p,
                                size_t(size-1));
          }
          else{
            t.pointer_to_member(this->_M_impl._M_finish._M_p,
                                this->_M_impl._M_start._M_p,
                                size_t(size));
            t.pointer_to_member(this->_M_impl._M_end_of_storage,
                                this->_M_impl._M_start._M_p,
                                size_t(size));
          }
        }

        template <typename Alloc>
        inline void std::vector<bool, Alloc>::define_type(stapl::typer& t)
        {
          t.base<_Bvector_base<Alloc> >(*this);
        }
      #endif
    #endif
  #endif // STAPL_BVECTOR_DEFINE_TYPE


  // std::list<>
  #ifndef STAPL_LIST_DEFINE_TYPE
    #if defined(_STL_LIST_H)
      #if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 5))
        #define STAPL_LIST_DEFINE_TYPE
        template <typename T>
        inline void std::_List_iterator<T>::define_type(stapl::typer& t)
        {
          stapl_assert(1, "_List_iterator define_type() used");
          long tmp = (long) _M_node;
          t.member(tmp);
        }

        template <typename T>
        inline std::_List_node_base* std::_List_iterator<T>::get_address(void)
        {
          stapl_assert(0, "_List_iterator get_address() used but not allowed");
          return _M_node;
        }

        template <typename T, typename Alloc>
        inline void std::list<T, Alloc>::define_type(stapl::typer &t)
        {
          stapl_assert(0, "list define_type() used.");
        }
      #elif ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
             (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
             (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
        #define STAPL_LIST_DEFINE_TYPE
        template <typename T>
        inline void std::_List_iterator<T>::define_type(stapl::typer& t)
        {
          long tmp = (long) _M_node;
          t.member(tmp);
        }

        template <typename T>
        inline std::__detail::_List_node_base*
        std::_List_iterator<T>::get_address(void)
        {
          stapl_assert(0, "_List_iterator get_address() used but not allowed");
          return _M_node;
        }

        template <typename T, typename Alloc>
        inline void std::list<T, Alloc>::define_type(stapl::typer &t)
        {
          stapl_assert(0, "list define_type() used.");
        }
      #endif
    #endif
  #endif // STAPL_LIST_DEFINE_TYPE


  #if !defined(STAPL_DONT_USE_BOOST_SERIALIZATION)

  // std::map<>
  #ifndef STAPL_MAP_DEFINE_TYPE
  # if defined(_STL_MAP_H)
  #  define STAPL_MAP_DEFINE_TYPE
  #  include <boost/serialization/map.hpp>
  # endif
  #endif // STAPL_MAP_DEFINE_TYPE

  // std::set<>
  #ifndef STAPL_SET_DEFINE_TYPE
  # if defined(_STL_SET_H)
  #  define STAPL_SET_DEFINE_TYPE
  #  include <boost/serialization/serialization.hpp>
  #  include <boost/serialization/set.hpp>
  # endif
  #endif // STAPL_SET_DEFINE_TYPE

  // unordered_map and unordered_multimap (std and boost)
  #ifndef STAPL_UNORDERED_MAP_DEFINE_TYPE
  # if defined(BOOST_UNORDERED_UNORDERED_MAP_HPP_INCLUDED) || \
       defined(_UNORDERED_MAP_H)
  #  define STAPL_UNORDERED_MAP_DEFINE_TYPE
  #  include <boost/serialization/unordered_map.hpp>
  # endif
  #endif // STAPL_UNORDERED_MAP_DEFINE_TYPE

  // unordered_set and unordered_multiset (std and boost)
  #ifndef STAPL_UNORDERED_SET_DEFINE_TYPE
  # if defined(BOOST_UNORDERED_UNORDERED_SET_HPP_INCLUDED) || \
       defined(_UNORDERED_SET_H)
  #  define STAPL_UNORDERED_SET_DEFINE_TYPE
  #  include <boost/serialization/unordered_set.hpp>
  # endif
  #endif // STAPL_UNORDERED_SET_DEFINE_TYPE

  #endif // STAPL_DONT_USE_BOOST_SERIALIZATION


  // __normal_iterator
  #ifndef STAPL_NORMAL_ITERATOR_DEFINE_TYPE
    #if defined(_STL_ITERATOR_H)
      #if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
           (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
           (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
        #define STAPL_NORMAL_ITERATOR_DEFINE_TYPE
        template<typename Iterator, typename Container>
        inline void
        __gnu_cxx::__normal_iterator<
          Iterator, Container
        >::define_type(stapl::typer&)
        {
          // This should be just a pointer, higher level parts of stapl promote
          // this to a distributed iterator if necessary
        }
      #endif
    #endif
  #endif // STAPL_NORMAL_ITERATOR_DEFINE_TYPE


  // std::reverse_iterator<>
  #ifndef STAPL_REVERSE_ITERATOR_DEFINE_TYPE
    #if defined(_ITERATOR_H) || defined(_STL_ITERATOR_H)
      #if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
           (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
           (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
        #define STAPL_REVERSE_ITERATOR_DEFINE_TYPE
        template<typename Iterator>
        inline void
        std::reverse_iterator<Iterator>::define_type(stapl::typer& t)
        {
          t.member(current);
        }
      #endif
    #endif
  #endif // STAPL_REVERSE_ITERATOR_DEFINE_TYPE


  // std::valarray<>
  #ifndef STAPL_VALARRAY_DEFINE_TYPE
    #if defined(_GLIBCXX_VALARRAY)
      #if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
           (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
           (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
        #define STAPL_VALARRAY_DEFINE_TYPE
        template <typename T>
        inline void std::valarray<T>::define_type(stapl::typer& t)
        {
          t.member(_M_size);
          t.member(const_cast<T*&>(_M_data), _M_size);
        }
      #endif
    #endif
  #endif // STAPL_VALARRAY_DEFINE_TYPE


  // std::vector<>
  #ifndef STAPL_VECTOR_DEFINE_TYPE
    #if defined(_STL_VECTOR_H)
      #if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
           (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
           (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
        #define STAPL_VECTOR_DEFINE_TYPE
        template <typename T, typename Alloc>
        inline void std::vector<T, Alloc>::define_type(stapl::typer& t)
        {
          int size=this->_M_impl._M_finish-this->_M_impl._M_start;
          t.member(this->_M_impl._M_start, size_t(size));
          t.pointer_to_member(this->_M_impl._M_finish,
                              this->_M_impl._M_start,
                              size_t(size));
          t.pointer_to_member(this->_M_impl._M_end_of_storage,
                              this->_M_impl._M_start,
                              size_t(size));
        }
      #endif
    #endif
  #endif // STAPL_VECTOR_DEFINE_TYPE


  // std::basic_string<char> (string)
  #ifndef STAPL_BASIC_STRING_DEFINE_TYPE
    #if defined(_BASIC_STRING_H)
      #if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
           (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
           (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4) && \
           (_GLIBCXX_USE_CXX11_ABI == 0))
        #define STAPL_BASIC_STRING_DEFINE_TYPE
          namespace stapl {

          //////////////////////////////////////////////////////////////////////
          /// @brief Specialization of @ref typer_traits for @c std::string.
          ///
          /// @ingroup serialization
          //////////////////////////////////////////////////////////////////////
          template<>
          class typer_traits<std::basic_string<
                               char,
                               std::char_traits<char>,
                               std::allocator<char>
                             >>
          {
          public:
            typedef std::basic_string<
                      char, std::char_traits<char>, std::allocator<char>
                    > value_type;

          private:
            typedef value_type::_Rep _Rep;

          public:
            static std::size_t packed_size(value_type const& t) noexcept
            {
              const _Rep* rep = reinterpret_cast<_Rep*>(t._M_dataplus._M_p);
              return (rep[-1]._M_length + 1 + sizeof(_Rep));
            }

            static std::pair<bool, std::size_t>
            meets_requirements(const typer::pass_type,
                               value_type const& t) noexcept
            { return std::make_pair(true, packed_size(t)); }

            static void prepack(value_type* dest,
                                value_type const* src,
                                const std::size_t num = 1) noexcept
            { std::memcpy(dest, src, (sizeof(value_type)*num)); }

            static std::size_t pack(value_type& dest,
                                    void* base,
                                    const std::size_t offset,
                                    value_type const&) noexcept
            {
              char*& buf = dest._M_dataplus._M_p;
              // rewind pointer to begin of _Rep struct
              buf -= sizeof(_Rep);
              const _Rep* rep = reinterpret_cast<_Rep*>(buf);
              std::size_t length = (rep->_M_length + 1 + sizeof(_Rep));
              std::memcpy(static_cast<char*>(base) + offset, buf, length);
              _Rep* rep2 =
                (reinterpret_cast<_Rep*>(static_cast<char*>(base) + offset));
              // should keep copies on remote side from trying to share
              rep2->_M_refcount = -1;
              rep2->_M_capacity = rep->_M_length;
              buf = reinterpret_cast<char*>(offset);
              return length;
            }

            static std::size_t unpack(value_type& t, void* base) noexcept
            {
              t._M_dataplus._M_p =
                (static_cast<char*>(base) +
                 reinterpret_cast<uintptr_t>(t._M_dataplus._M_p));
              // fast forward pointer to after _Rep struct
              t._M_dataplus._M_p += sizeof(_Rep);
              // calculate the size to be returned
              const _Rep* rep = reinterpret_cast<_Rep*>(t._M_dataplus._M_p);
              return (rep[-1]._M_length + 1 + sizeof(_Rep));
            }

            static void destroy(value_type&) noexcept
            { }
          };


          //////////////////////////////////////////////////////////////////////
          /// @internal
          /// @brief Specialization of @ref typer_traits_specialization for
          ///        @c std::string.
          ///
          /// @ingroup serialization
          //////////////////////////////////////////////////////////////////////
          template<>
          struct typer_traits_specialization<std::basic_string<
                                               char,
                                               std::char_traits<char>,
                                               std::allocator<char>
                                            >>
          : public std::true_type
          { };

          } // namespace stapl
      #endif
    #endif
  #endif // STAPL_BASIC_STRING_DEFINE_TYPE


#endif // _STAPL
