/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_BENCHMARKS_KRIPKE_CONTIGUOUS_VECTOR_H
#define STAPL_BENCHMARKS_KRIPKE_CONTIGUOUS_VECTOR_H


/*!
  \file contiguous_vector.h
  \brief This file offers basic data structures for viewing any contiguous
  memory space as an n-dimensional vector.
 */

#include <vector>
#include <stapl/runtime.hpp>
#include <stdexcept>

/*!
  \class contiguous_vector
  \brief Views a contiguous memory area as n-dimensional vector.

  The memory area is seen as an n-dimensional space with given dimensions
  dim1, dim2, ... First dimension is the major one, so the strides will
  be decreasing.

  We assume there is enough space in the given memory area (dim1*dim2*...).
  This class allows easy indexing.

  For example:
  \code
  int buffer[3*4*5]

  contiguous_vector my_array(buffer,3,4,5);
                          // Declares an array with three dimensions of
                          // sizes 3, 4, 5 respectively.

  value = my_array(0,0,0) // Reads value from the array for element (0,0,0)

  ptr = &my_array(2)      // Sets pointer ptr to the third 2D sub-array of
                          // size 4*5.

  contiguous_array a3(my_array(2), 4, 5);
                          // Create a 2D structure on the third sub-array
                          // of size 4*5.

  \endcode

  \warning This structure does not interfere with the memory management of
  the buffer containing the array.  It stores additional structures to allow
  easy indexing.

  \warning This implementation works for up to 3 dimensions.  We have chosen
  to implement it to avoid assimilating a whole library for this purpose,
  but, in the future, plan to incorporate an array management package.
 */
template <class T>
class contiguous_vector
{
 private:
  /*!
    \brief Address of memory buffer.
   */
  T* m_content;
  /*!
    \brief Size of addressable space.
   */
  size_t m_size;
  /*!
    \brief List of strides.
   */
  std::vector<int> m_strides;
  /*!
    \brief Ownership flag.
    \warning A value of true triggers deallocation of m_content in
    the destructor.
   */
  bool m_is_owner;

 protected:
  /*!
    \brief Check for sufficient space and allocates if needed.
   */
  void fix_space(size_t needed_space);
  /*!
    \brief Sets ownership flag.
   */
  void set_owner(T*);

 public:
  /*!
    \brief Default constructor.
   */
  contiguous_vector();
  /*!
    \brief Copy constructor.
  */
  contiguous_vector(const contiguous_vector<T>& other);
  /*!
    \brief Move constructor.
  */
  contiguous_vector(contiguous_vector<T>&& other) noexcept;
  /*!
    \brief Builds contiguous vector given a buffer.
   */
  contiguous_vector(T*);
  /*!
    \brief Builds contiguous vector given a buffer (does NOT allocate buffer
    space).
   */
  contiguous_vector(std::vector<int> dimensions, T*);
  /*!
    \brief Builds contiguous vector (allocates buffer space).
   */
  contiguous_vector(std::vector<int> dimensions);
  /*!
    \brief Builds contiguous vector given a buffer (does NOT allocate buffer
    space).
   */
  contiguous_vector(int dim1, T*);
  /*!
    \brief Builds contiguous vector (allocates buffer space).
   */
  contiguous_vector(int dim1);
  /*!
    \brief Builds contiguous vector given a buffer (does NOT allocate buffer
    space).
   */
  contiguous_vector(int dim1, int dim2, T*);
  /*!
    \brief Builds contiguous vector (allocates buffer space).
   */
  contiguous_vector(int dim1, int dim2);
  /*!
    \brief Builds contiguous vector given a buffer (does NOT allocate buffer
    space).
   */
  contiguous_vector(int dim1, int dim2, int dim3, T*);
  /*!
    \brief Builds contiguous vector (allocates buffer space).
   */
  contiguous_vector(int dim1, int dim2, int dim3);
  /*!
    \brief Destroys the vector.  Deallocates buffer space if owner.
   */
  ~contiguous_vector() noexcept;
  /*!
    \brief Assignement operator.
   */
  contiguous_vector<T>& operator=(const contiguous_vector<T>& other);
  /*!
    \brief Move assignement operator.
   */
  contiguous_vector<T>& operator=(contiguous_vector<T>&& other) noexcept;
  /*!
    \brief Dynamic re-dimensioning.
   */
  void dimension(int dim1);
  /*!
    \brief Dynamic re-dimensioning.
   */
  void dimension(int dim1, int dim2);
  /*!
    \brief Dynamic re-dimensioning.
   */
  void dimension(int dim1, int dim2, int dim3);
  /*!
    \brief Dynamic re-dimensioning.
   */
  void dimension(std::vector<int> dimensions);
  /*!
    \brief Dynamic re-dimensioning.
   */
  void dimension(int dim1, T* nb);
  /*!
    \brief Dynamic re-dimensioning.
   */
  void dimension(int dim1, int dim2, T* nb);
  /*!
    \brief Dynamic re-dimensioning.
   */
  void dimension(int dim1, int dim2, int dim3, T* nb);
  /*!
    \brief Dynamic re-dimensioning.
   */
  void dimension(std::vector<int> dimensions, T* nb);
  /*!
    \brief Returns vector size.
   */
  size_t size() const;
  /*!
    \brief Returns vector dimensions.
   */
  std::vector<int> dimensions() const;

  void switch_buffer(T* nb);
  /*!
    \brief Iterators
   */
  T* begin() { return m_content; };
  T* end() { return m_content+size(); };
  T const* begin() const { return m_content; };
  T const* end() const { return m_content+size(); };
  /*!
    \brief Access function, 1 index.
   */
  T& operator ()(int index0);
  /*!
    \brief Access function, 2 indices.
   */
  T& operator ()(int index0, int index1);
  /*!
    \brief Access function, 3 indices.
   */
  T& operator ()(int index0, int index1, int index2);

  void define_type(stapl::typer &t)
  {
    t.member(m_content, m_size);
    t.member(m_size);
    t.member(m_strides);
    t.member(m_is_owner);
  }
};

template <class T>
inline contiguous_vector<T>::contiguous_vector(const contiguous_vector<T>& other)
{
  if (other.m_is_owner)
  {
    m_strides = other.m_strides;
    m_is_owner = true;
    m_size = other.m_size;
    if( 0 == m_size )
    {
      m_content = 0;
    }
    else
    {
      m_content = new T[m_size];
      T *p = m_content;
      T *q = other.m_content;
      for( ;p<m_content+m_size; ++p, ++q)
      {
        *p = *q;
      }
    }
  }
  else
  {
    m_is_owner = false;
    m_strides = other.m_strides;
    m_size = other.m_size;
    m_content = other.m_content;
  }
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(contiguous_vector<T>&& other) noexcept
  : m_content(other.m_content), m_size(other.m_size),
    m_strides(std::move(other.m_strides)), m_is_owner(true)
{
  other.m_content = NULL;
  other.m_size = 0;
}

template <class T>
inline contiguous_vector<T>&
contiguous_vector<T>::operator=(const contiguous_vector<T>& other)
{
  if( this != &other )
  {
    if (other.m_is_owner)
    {
      m_strides = other.m_strides;
      m_is_owner = true;
      if( 0 != other.m_size )
      {
        this->fix_space(other.m_size);
        T *p = m_content;
        T *q = other.m_content;
        for( ;p<m_content+m_size; ++p, ++q)
        {
          *p = *q;
        }
      }
    }
    else
    {
      m_is_owner = false;
      m_strides = other.m_strides;
      m_size = other.m_size;
      m_content = other.m_content;
    }
  }
  return *this;
}

template <class T>
inline contiguous_vector<T>&
contiguous_vector<T>::operator=(contiguous_vector<T>&& other) noexcept
{
  if ( this != &other )
  {
    if (m_content != 0) delete[] m_content;
    m_content = other.m_content;
    m_size = other.m_size;
    m_strides = std::move(other.m_strides);
    other.m_content = NULL;
    other.m_size = 0;
  }
  return *this;
}

template <class T>
inline void contiguous_vector<T>::set_owner(T* content)
{
  m_size = 0;
  m_content = content;
  if (content == 0)
  {
    m_is_owner = true;
  }
  else
  {
    m_is_owner = false;
  }
}

template <class T>
inline void contiguous_vector<T>::fix_space(size_t needed_space)
{
  if( m_is_owner )
  {
    if( needed_space > m_size )
    {
      if( m_content != 0 )  delete [] m_content;
      m_content = new T[needed_space];
    }
  }
  m_size = needed_space;
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(T* content)
{
  set_owner(content);
  fix_space(0);
}

template <class T>
inline contiguous_vector<T>::contiguous_vector():
  m_content(0), m_size(0), m_strides(), m_is_owner(true)
{}

template <class T>
inline contiguous_vector<T>::contiguous_vector(std::vector<int> dimensions, T* content)
{
  set_owner(content);
  m_strides.resize(dimensions.size());
  int stride = 1;
  std::vector<int>::iterator sit=m_strides.end()-1;
  for(std::vector<int>::iterator dit=dimensions.end()-1;
      dit>=dimensions.begin(); --dit,--sit)
  {
    *sit=stride;
    stride = stride * (*dit);
  }
  int size = stride;
  fix_space(size);
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(std::vector<int> dimensions)
{
  T* content = 0;
  set_owner(content);
  m_strides.resize(dimensions.size());
  int stride = 1;
  std::vector<int>::iterator sit=m_strides.end()-1;
  for(std::vector<int>::iterator dit=dimensions.end()-1;
      dit>=dimensions.begin(); --dit,--sit)
  {
    *sit = stride;
    stride = stride * (*dit);
  }
  int size = stride;
  fix_space(size);
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(int dim1, T* content)
{
  set_owner(content);
  fix_space(dim1);
  m_strides.resize(1);
  m_strides[0] = 1;
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(int dim1)
{
  T* content = 0;
  set_owner(content);
  fix_space(dim1);
  m_strides.resize(1);
  m_strides[0] = 1;
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(int dim1, int dim2, T* content)
{
  set_owner(content);
  fix_space(dim1*dim2);
  m_strides.resize(2);
  m_strides[1] = 1;
  m_strides[0] = dim2;
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(int dim1, int dim2)
{
  T* content = 0;
  set_owner(content);
  fix_space(dim1*dim2);
  m_strides.resize(2);
  m_strides[1] = 1;
  m_strides[0] = dim2;
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(int dim1, int dim2, int dim3,
                                        T* content)
{
  set_owner(content);
  fix_space(dim1*dim2*dim3);
  m_strides.resize(3);
  m_strides[2] = 1;
  m_strides[1] = dim3;
  m_strides[0] = dim3*dim2;
}

template <class T>
inline contiguous_vector<T>::contiguous_vector(int dim1, int dim2, int dim3)
{
  T* content = 0;
  set_owner(content);
  fix_space(dim1*dim2*dim3);
  m_strides.resize(3);
  m_strides[2] = 1;
  m_strides[1] = dim3;
  m_strides[0] = dim3*dim2;
}

template <class T>
inline contiguous_vector<T>::~contiguous_vector() noexcept
{
  if (m_is_owner)
  {
    if (m_content != 0)
    {
      delete[] m_content;
    }
    m_content = 0;
  }
}

template <class T>
inline size_t contiguous_vector<T>::size() const
{
  return m_size;
}

template <class T>
inline std::vector<int> contiguous_vector<T>::dimensions() const
{
  std::vector<int> dims(m_strides.size());
  std::vector<int>::iterator d = dims.end()-1;
  std::vector<int>::const_iterator s = m_strides.end()-2;
  int outer_dim = m_size;
  for (; s >= m_strides.begin(); --d, --s)
  {
    *d = *s;
    outer_dim = outer_dim / *s;
  }
  dims[0] = outer_dim;
  return dims;
}

template <class T>
inline void contiguous_vector<T>::dimension(std::vector<int> dimensions)
{
  m_strides.resize(dimensions.size());
  int stride = 1;
  std::vector<int>::iterator sit=m_strides.end()-1;
  for(std::vector<int>::iterator dit=dimensions.end()-1;
      dit>=dimensions.begin(); --dit,--sit)
  {
    *sit = stride;
    stride = stride * (*dit);
  }
  int size = stride;
  fix_space(size);

}

template <class T>
inline void contiguous_vector<T>::dimension(int dim1)
{
  fix_space(dim1);
  m_strides.resize(1);
  m_strides[0] = 1;
}

template <class T>
inline void contiguous_vector<T>::dimension(int dim1, int dim2)
{
  fix_space(dim1*dim2);
  m_strides.resize(2);
  m_strides[1] = 1;
  m_strides[0] = dim2;
}

template <class T>
inline void contiguous_vector<T>::dimension(int dim1, int dim2, int dim3)
{
  fix_space(dim1*dim2*dim3);
  m_strides.resize(3);
  m_strides[2] = 1;
  m_strides[1] = dim3;
  m_strides[0] = dim3*dim2;
}

template <class T>
inline void contiguous_vector<T>::dimension(std::vector<int> dimensions, T* nb)
{
  m_strides.resize(dimensions.size());
  int stride = 1;
  std::vector<int>::iterator sit=m_strides.end()-1;
  for(std::vector<int>::iterator dit=dimensions.end()-1;
      dit>=dimensions.begin(); --dit,--sit)
  {
    *sit = stride;
    stride = stride * (*dit);
  }
  int size = stride;
  m_size = size;
  m_is_owner = false;
  m_content = nb;
}

template <class T>
inline void contiguous_vector<T>::dimension(int dim1, T* nb)
{
  m_size = dim1;
  m_is_owner = false;
  m_content = nb;

  m_strides.resize(1);
  m_strides[0] = 1;
}

template <class T>
inline void contiguous_vector<T>::dimension(int dim1, int dim2, T* nb)
{
  m_size = dim1*dim2;
  m_is_owner = false;
  m_content = nb;

  m_strides.resize(2);
  m_strides[1] = 1;
  m_strides[0] = dim2;
}

template <class T>
inline void contiguous_vector<T>::dimension(int dim1, int dim2, int dim3, T* nb)
{
  m_size = dim1*dim2*dim3;
  m_is_owner = false;
  m_content = nb;
  m_strides.resize(3);
  m_strides[2] = 1;
  m_strides[1] = dim3;
  m_strides[0] = dim3*dim2;
}

template <class T>
inline void contiguous_vector<T>::switch_buffer(T* nb)
{
  m_is_owner = false;
  m_content = nb;
}

template <class T>
inline T& contiguous_vector<T>::operator ()(int index0)
{
  return *(m_content+index0*m_strides[0]);
}

template <class T>
inline T& contiguous_vector<T>::operator ()(int index0, int index1)
{
  return *(m_content+index0*m_strides[0]+index1*m_strides[1]);
}

template <class T>
inline T& contiguous_vector<T>::operator ()(int index0, int index1, int index2)
{
  return *(m_content+index0*m_strides[0]+index1*m_strides[1]+index2*m_strides[2]);
}

#endif
