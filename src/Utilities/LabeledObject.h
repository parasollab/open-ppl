#ifndef _LABELED_OBJECT_H_
#define _LABELED_OBJECT_H_

#include "IOUtils.h"
#include <string>

/// Base class for Labeled Objects.
///
/// Provides access methods to the object's label data, and a
/// private function to easily get the label from an XML node.
class LabeledObject {
public:
  ///Main Constructor
  ///\param in_strLabel the label of this object
  LabeledObject(const std::string& in_strLabel) { m_strLabel = in_strLabel; }
  
  ///Default empty constructor
  LabeledObject() { }

  ///\name Access Methods
  //@{
  ///Returns the object's label as a string
  inline const std::string& GetObjectLabel() const { return m_strLabel; }
  ///Sets the object's label manually
  inline void SetLabel(const std::string& in_strLabel) { m_strLabel = in_strLabel; }
  //@}

protected:
  ///Returns the "Label" attribute of the XML node
  /// \param in_Node XML node to parse
  /// \note This function requires that the XML node has an attribute called 
  ///  "Label."  If this is missing, then the program will warn and exit.
  std::string ParseLabelXML (XMLNodeReader& in_Node) const {
    return in_Node.stringXMLParameter(std::string("Label"),
                                      true,
                                      std::string(""),
                                      std::string("Label Identifier"));
  }
   

private:
  std::string m_strLabel; ///< The label stored for this object
};


#endif //_LABELED_OBJECT_H_
