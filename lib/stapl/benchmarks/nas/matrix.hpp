/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef NAS_MATRIX_HPP_HPP
#define NAS_MATRIX_HPP_HPP

namespace stapl {

template<typename BaseContainer>
class my_matrix
  : public p_object
{
private:
  BaseContainer* m_base_ct_ptr;

  std::size_t m_nprows;
  std::size_t m_npcols;
  std::size_t m_my_row;
  std::size_t m_my_col;

public:
  typedef BaseContainer b_container_t;

  my_matrix(BaseContainer* base_ct_ptr,
            std::size_t nprows, std::size_t npcols,
            std::size_t my_row, std::size_t my_col)
    : m_base_ct_ptr(base_ct_ptr),
      m_nprows(nprows), m_npcols(npcols),
      m_my_row(my_row), m_my_col(my_col)
  { }

  BaseContainer& bcontainer() const
  {
    return *m_base_ct_ptr;
  }

  std::size_t nprows() const
  {
    return m_nprows;
  }

  std::size_t npcols() const
  {
    return m_npcols;
  }

  std::size_t my_prow() const
  {
    return m_my_row;
  }

  std::size_t my_pcol() const
  {
    return m_my_col;
  }

  ~my_matrix()
  {
    stapl_assert(0, "my_matrix destructor actually called, how?");

    delete m_base_ct_ptr;
  }
}; // class my_matrix

} // namespace stapl

#endif // ifndef NAS_MATRIX_HPP_HPP

