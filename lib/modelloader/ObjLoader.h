#ifndef OBJ_LOADER_H_
#define OBJ_LOADER_H_

#include <iostream>
#include <string>

#include "IModel.h"


////////////////////////////////////////////////////////////////////////////////
/// \ingroup DeadCode
/// \brief Describes the properties of a given material. Not yet used.
////////////////////////////////////////////////////////////////////////////////
struct CObjmaterial {
  std::string m_name;        ///< Name of material.
  float m_diffuse[4];        ///< Diffuse component.
  float m_ambient[4];        ///< Ambient component.
  float m_specular[4];       ///< Specular component.
  float m_emmissive[4];      ///< Emmissive component.
  float m_shininess;         ///< Specular exponent.
};


////////////////////////////////////////////////////////////////////////////////
/// \ingroup DeadCode
/// \brief Describes a triangle(?) group. Not yet used.
////////////////////////////////////////////////////////////////////////////////
struct CObjGroup {
  std::string m_name;        ///< Name of this group.
  unsigned int m_tris_start; ///< The index of the first triangle in the group.
  unsigned int m_tris_end;   ///< The index of the last triangle in the group.
  unsigned int m_material;   ///< Index to material for group.
};


////////////////////////////////////////////////////////////////////////////////
/// \brief A loader for wavefront .obj files.
////////////////////////////////////////////////////////////////////////////////
class CObjLoader : public IModel {

  public:

    ///\name IModel Overrides
    ///@{

    virtual bool ParseFile(bool _silent = false) override;

    ///@}

  protected:

    ///\name Parsing Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Read a model description from an OBJ file.
    /// \param[in] _in A filestream that is reading the OBJ file.
    /// \return True on success, false otherwise.
    bool ReadOBJ(std::istream& _in);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief First pass parses object counts, such as number of vertices,
    ///        normals, etc., and reserves space.
    /// \param[in] _in A filestream that is reading the OBJ file.
    /// \return True if all of the tokens in the file were recognized, false
    ///         otherwise.
    bool FirstPass(std::istream& _in);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Second pass reads data into reserved space.
    /// \param[in] _in A filestream that is reading the OBJ file.
    /// \return True if all of the tokens in the file were recognized, false
    ///         otherwise.
    bool SecondPass(std::istream& _in);

    ///@}

//
// Unfinished code for groups and materials.
//
//    CObjGroup& FindGroup(const std::string& name);
//    CObjGroup& AddGroup(const std::string& name);
//
//  private:
//
//    std::vector<CObjGroup>  groups;
//    std::vector<CObjmaterial> material;
//

};

#endif
